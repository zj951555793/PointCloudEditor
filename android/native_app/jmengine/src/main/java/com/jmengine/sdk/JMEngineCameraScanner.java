package com.jmengine.sdk;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.ImageFormat;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.HandlerThread;
import android.view.Surface;

import java.nio.ByteBuffer;
import java.util.Collections;
import java.util.concurrent.atomic.AtomicInteger;

/** JMEngine-owned dual-camera source: A=structured-light grayscale, B=RGB. */
public final class JMEngineCameraScanner implements AutoCloseable {
    public interface Listener {
        void onStatus(String text);

        void onFrame(int frameId);
    }

    private final Context context;
    private final JMEngineNative engine;
    private final Listener listener;
    private final HandlerThread thread = new HandlerThread("JMEngine-Camera");
    private final Handler handler;
    private final AtomicInteger frameIds = new AtomicInteger();

    private ImageReader cameraAReader;
    private ImageReader cameraBReader;
    private CameraDevice cameraA;
    private CameraDevice cameraB;
    private CameraCaptureSession cameraASession;
    private CameraCaptureSession cameraBSession;
    private byte[] codeFrame;
    private byte[] rgbFrame;
    private long cameraATimestampUs;
    private long cameraBTimestampUs;
    private long toleranceUs = 50_000;
    private int width;
    private int height;

    public JMEngineCameraScanner(Context context, JMEngineNative engine, Listener listener) {
        this.context = context.getApplicationContext();
        this.engine = engine;
        this.listener = listener;
        thread.start();
        handler = new Handler(thread.getLooper());
    }

    public String[] cameraIds() throws Exception {
        return cameraManager().getCameraIdList();
    }

    public void start(
            String cameraAId,
            String cameraBId,
            int width,
            int height,
            long toleranceUs) throws Exception {
        if (context.checkSelfPermission(Manifest.permission.CAMERA)
                != PackageManager.PERMISSION_GRANTED) {
            throw new SecurityException("CAMERA permission not granted");
        }

        this.width = width;
        this.height = height;
        this.toleranceUs = Math.max(0, toleranceUs);

        cameraAReader = ImageReader.newInstance(width, height, ImageFormat.YUV_420_888, 2);
        cameraBReader = ImageReader.newInstance(width, height, ImageFormat.YUV_420_888, 2);
        cameraAReader.setOnImageAvailableListener(this::onCameraAImageAvailable, handler);
        cameraBReader.setOnImageAvailableListener(this::onCameraBImageAvailable, handler);

        cameraManager().openCamera(cameraAId, new DeviceState(true), handler);
        cameraManager().openCamera(cameraBId, new DeviceState(false), handler);
        status("opening A=" + cameraAId + " B=" + cameraBId);
    }

    private CameraManager cameraManager() {
        return (CameraManager) context.getSystemService(Context.CAMERA_SERVICE);
    }

    private void onCameraAImageAvailable(ImageReader reader) {
        try (Image image = reader.acquireLatestImage()) {
            if (image != null) {
                onCameraAFrame(image);
            }
        }
    }

    private void onCameraBImageAvailable(ImageReader reader) {
        try (Image image = reader.acquireLatestImage()) {
            if (image != null) {
                onCameraBFrame(image);
            }
        }
    }

    private final class DeviceState extends CameraDevice.StateCallback {
        private final boolean cameraADevice;

        DeviceState(boolean cameraADevice) {
            this.cameraADevice = cameraADevice;
        }

        @Override
        public void onOpened(CameraDevice device) {
            if (cameraADevice) {
                cameraA = device;
            } else {
                cameraB = device;
            }
            ImageReader reader = cameraADevice ? cameraAReader : cameraBReader;
            createSession(device, reader.getSurface(), cameraADevice);
        }

        @Override
        public void onDisconnected(CameraDevice device) {
            device.close();
            status("camera disconnected");
        }

        @Override
        public void onError(CameraDevice device, int error) {
            device.close();
            status("camera error=" + error);
        }
    }

    private void createSession(CameraDevice device, Surface surface, boolean cameraADevice) {
        try {
            device.createCaptureSession(
                    Collections.singletonList(surface),
                    new CameraCaptureSession.StateCallback() {
                        @Override
                        public void onConfigured(CameraCaptureSession session) {
                            try {
                                CaptureRequest.Builder request = device.createCaptureRequest(
                                        CameraDevice.TEMPLATE_PREVIEW);
                                request.addTarget(surface);
                                session.setRepeatingRequest(request.build(), null, handler);
                                if (cameraADevice) {
                                    cameraASession = session;
                                } else {
                                    cameraBSession = session;
                                }
                                status("camera " + (cameraADevice ? "A" : "B") + " started");
                            } catch (Exception exception) {
                                status(exception.toString());
                            }
                        }

                        @Override
                        public void onConfigureFailed(CameraCaptureSession session) {
                            status("capture session failed");
                        }
                    },
                    handler);
        } catch (Exception exception) {
            status(exception.toString());
        }
    }

    private synchronized void onCameraAFrame(Image image) {
        codeFrame = extractLuma(image);
        cameraATimestampUs = image.getTimestamp() / 1_000L;
        submitPairIfReady();
    }

    private synchronized void onCameraBFrame(Image image) {
        rgbFrame = convertYuvToRgb(image);
        cameraBTimestampUs = image.getTimestamp() / 1_000L;
        submitPairIfReady();
    }

    private void submitPairIfReady() {
        if (codeFrame == null || rgbFrame == null) {
            return;
        }

        if (Math.abs(cameraATimestampUs - cameraBTimestampUs) > toleranceUs) {
            if (cameraATimestampUs < cameraBTimestampUs) {
                codeFrame = null;
            } else {
                rgbFrame = null;
            }
            return;
        }

        int frameId = frameIds.getAndIncrement();
        long timestampUs = Math.max(cameraATimestampUs, cameraBTimestampUs);
        if (engine.submitStructuredFrame(
                rgbFrame, codeFrame, width, height, timestampUs, frameId)
                && listener != null) {
            listener.onFrame(frameId);
        }
        codeFrame = null;
        rgbFrame = null;
    }

    private static byte[] extractLuma(Image image) {
        Image.Plane plane = image.getPlanes()[0];
        ByteBuffer buffer = plane.getBuffer();
        int width = image.getWidth();
        int height = image.getHeight();
        byte[] output = new byte[width * height];

        for (int row = 0; row < height; ++row) {
            for (int column = 0; column < width; ++column) {
                output[row * width + column] = buffer.get(
                        row * plane.getRowStride() + column * plane.getPixelStride());
            }
        }
        return output;
    }

    private static byte[] convertYuvToRgb(Image image) {
        int width = image.getWidth();
        int height = image.getHeight();
        Image.Plane[] planes = image.getPlanes();
        byte[] output = new byte[width * height * 3];

        for (int row = 0; row < height; ++row) {
            for (int column = 0; column < width; ++column) {
                int y = samplePlane(planes[0], row, column) & 0xff;
                int u = samplePlane(planes[1], row / 2, column / 2) & 0xff;
                int v = samplePlane(planes[2], row / 2, column / 2) & 0xff;
                int c = y - 16;
                int d = u - 128;
                int e = v - 128;
                int offset = (row * width + column) * 3;
                output[offset] = (byte) clamp((298 * c + 409 * e + 128) >> 8);
                output[offset + 1] =
                        (byte) clamp((298 * c - 100 * d - 208 * e + 128) >> 8);
                output[offset + 2] = (byte) clamp((298 * c + 516 * d + 128) >> 8);
            }
        }
        return output;
    }

    private static byte samplePlane(Image.Plane plane, int row, int column) {
        return plane.getBuffer().get(
                row * plane.getRowStride() + column * plane.getPixelStride());
    }

    private static int clamp(int value) {
        return Math.max(0, Math.min(255, value));
    }

    private void status(String message) {
        if (listener != null) {
            listener.onStatus(message);
        }
    }

    @Override
    public synchronized void close() {
        if (cameraASession != null) {
            cameraASession.close();
        }
        if (cameraBSession != null) {
            cameraBSession.close();
        }
        if (cameraA != null) {
            cameraA.close();
        }
        if (cameraB != null) {
            cameraB.close();
        }
        if (cameraAReader != null) {
            cameraAReader.close();
        }
        if (cameraBReader != null) {
            cameraBReader.close();
        }
        thread.quitSafely();
    }
}
