package com.jmengine.sdk;

import android.content.Context;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbInterface;
import android.os.Handler;
import android.os.Looper;
import android.view.Surface;

import com.jiangdg.usb.USBMonitor;
import com.jiangdg.uvc.UVCCamera;
import com.jiangdg.utils.Size;

import java.util.List;

public class CameraCapture implements AutoCloseable {
    public static final int REALTEK_VENDOR_ID = 0x0BDA;
    public static final int CAMERA_A_PRODUCT_ID = 0x300A;
    public static final int CAMERA_B_PRODUCT_ID = 0x300B;

    private static final int PREVIEW_WIDTH = 1280;
    private static final int PREVIEW_HEIGHT = 720;
    private static final int MAX_FIND_RETRIES = 10;
    private static final long FIND_RETRY_DELAY_MS = 300;

    private final Context context;
    private final Listener listener;
    private final int targetProductId;
    private final Surface previewSurface;
    private USBMonitor usbMonitor;
    private UVCCamera camera;
    private UsbDevice pendingDevice;
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private int findAttempts;

    public interface Listener {
        void onStatus(String text);
    }

    public CameraCapture(Context context, int targetProductId, Surface previewSurface, Listener listener) {
        this.context = context.getApplicationContext();
        this.targetProductId = targetProductId;
        this.previewSurface = previewSurface;
        this.listener = listener;
    }

    public static boolean isUvcDevice(UsbDevice device) {
        if (device == null) {
            return false;
        }
        if (device.getDeviceClass() == 14) {
            return true;
        }
        for (int i = 0; i < device.getInterfaceCount(); ++i) {
            UsbInterface usbInterface = device.getInterface(i);
            if (usbInterface != null && usbInterface.getInterfaceClass() == 14) {
                return true;
            }
        }
        return false;
    }

    public void start() {
        if (previewSurface == null || !previewSurface.isValid()) {
            status("preview surface is not ready");
            return;
        }
        stopCameraOnly();
        usbMonitor = new USBMonitor(context, new USBMonitor.OnDeviceConnectListener() {
            @Override public void onAttach(UsbDevice device) {
                if (matches(device)) {
                    pendingDevice = device;
                    status("attached " + nameOf(device) + ", requesting USB permission");
                    usbMonitor.requestPermission(device);
                }
            }

            @Override public void onDetach(UsbDevice device) {
                if (matches(device)) {
                    status("detached " + nameOf(device));
                    stopCameraOnly();
                }
            }

            @Override public void onConnect(UsbDevice device, USBMonitor.UsbControlBlock ctrlBlock, boolean createNew) {
                if (!matches(device)) {
                    return;
                }
                openUvc(device, ctrlBlock);
            }

            @Override public void onDisconnect(UsbDevice device, USBMonitor.UsbControlBlock ctrlBlock) {
                if (matches(device)) {
                    status("disconnected " + nameOf(device));
                    stopCameraOnly();
                }
            }

            @Override public void onCancel(UsbDevice device) {
                if (matches(device)) {
                    status("USB permission denied/cancelled " + nameOf(device));
                }
            }
        });
        usbMonitor.register();
        findAttempts = 0;
        findAndRequestTargetDevice();
    }

    private void findAndRequestTargetDevice() {
        UsbDevice device = findTargetDevice();
        if (device == null) {
            if (findAttempts < MAX_FIND_RETRIES) {
                ++findAttempts;
                status("waiting USB external camera PID=" + hex4(targetProductId)
                        + " retry=" + findAttempts + "/" + MAX_FIND_RETRIES);
                mainHandler.postDelayed(this::findAndRequestTargetDevice, FIND_RETRY_DELAY_MS);
            } else {
                status("need USB external camera PID=" + hex4(targetProductId) + ", found=0");
            }
            return;
        }
        pendingDevice = device;
        status("found " + nameOf(device) + ", requesting USB permission");
        usbMonitor.requestPermission(device);
    }

    private UsbDevice findTargetDevice() {
        if (usbMonitor == null) {
            return null;
        }
        List<UsbDevice> devices = usbMonitor.getDeviceList();
        for (UsbDevice device : devices) {
            if (matches(device)) {
                return device;
            }
        }
        return null;
    }

    private boolean matches(UsbDevice device) {
        return device != null
                && device.getVendorId() == REALTEK_VENDOR_ID
                && device.getProductId() == targetProductId
                && isUvcDevice(device);
    }

    private void openUvc(UsbDevice device, USBMonitor.UsbControlBlock ctrlBlock) {
        try {
            stopCameraOnly();
            UVCCamera uvc = new UVCCamera();
            uvc.open(ctrlBlock);
            try {
                uvc.setPreviewSize(PREVIEW_WIDTH, PREVIEW_HEIGHT, UVCCamera.FRAME_FORMAT_MJPEG);
            } catch (Exception mjpegFailed) {
                List<Size> sizes = uvc.getSupportedSizeList();
                if (!sizes.isEmpty()) {
                    Size first = sizes.get(0);
                    uvc.setPreviewSize(first.width, first.height, UVCCamera.FRAME_FORMAT_YUYV);
                }
            }
            uvc.setPreviewDisplay(previewSurface);
            uvc.startPreview();
            camera = uvc;
            status("UVC preview started " + nameOf(device));
        } catch (Exception e) {
            status("UVC open failed " + nameOf(device) + ": " + e.getMessage());
            stopCameraOnly();
        }
    }

    private void stopCameraOnly() {
        if (camera != null) {
            try {
                camera.stopPreview();
            } catch (Exception ignored) {
            }
            try {
                camera.close();
                camera.destroy();
            } catch (Exception ignored) {
            }
            camera = null;
        }
    }

    private void status(String text) {
        if (listener != null) {
            listener.onStatus(text);
        }
    }

    private static String nameOf(UsbDevice device) {
        if (device == null) {
            return "null";
        }
        return "VID=" + hex4(device.getVendorId())
                + " PID=" + hex4(device.getProductId())
                + " name=" + device.getDeviceName();
    }

    private static String hex4(int value) {
        return String.format("%04X", value & 0xFFFF);
    }

    @Override
    public void close() {
        mainHandler.removeCallbacksAndMessages(null);
        stopCameraOnly();
        if (usbMonitor != null) {
            try {
                usbMonitor.unregister();
            } catch (Exception ignored) {
            }
            try {
                usbMonitor.destroy();
            } catch (Exception ignored) {
            }
            usbMonitor = null;
        }
        pendingDevice = null;
    }
}
