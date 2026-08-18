package com.jmengine.sdk;

import android.content.Context;
import android.opengl.GLSurfaceView;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;

import java.util.concurrent.atomic.AtomicReference;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/** Reusable native GLES 3.1 point-cloud view backed by the same JMEngine handle. */
public final class JMEngineGlesView extends GLSurfaceView implements GLSurfaceView.Renderer {
    public enum InteractionMode { ORBIT, SURFACE_RECT, THROUGH_RECT }

    public interface SelectionListener { void onSelection(long selectedCount, boolean surfaceOnly); }

    private final JMEngineNative engine;
    private final ScaleGestureDetector scaleDetector;
    private final AtomicReference<SelectionListener> selectionListener = new AtomicReference<>();
    private InteractionMode mode = InteractionMode.ORBIT;
    private float downX, downY, lastX, lastY;
    private boolean moving;

    public JMEngineGlesView(Context context, JMEngineNative engine) {
        super(context);
        this.engine = engine;
        setEGLContextClientVersion(3);
        setPreserveEGLContextOnPause(true);
        setRenderer(this);
        setRenderMode(RENDERMODE_WHEN_DIRTY);
        scaleDetector = new ScaleGestureDetector(context, new ScaleGestureDetector.SimpleOnScaleGestureListener() {
            @Override public boolean onScale(ScaleGestureDetector detector) {
                final float scale = detector.getScaleFactor();
                queueEvent(() -> engine.glZoom(scale));
                requestRender();
                return true;
            }
        });
    }

    public void setInteractionMode(InteractionMode mode) { this.mode = mode; }
    public InteractionMode interactionMode() { return mode; }
    public void setSelectionListener(SelectionListener listener) { selectionListener.set(listener); }
    public void fitView() { queueEvent(engine::glFit); requestRender(); }
    public void notifyModelChanged() { requestRender(); }
    public void clearSelection() { queueEvent(engine::glClearSelection); requestRender(); }

    @Override public void onSurfaceCreated(GL10 gl, EGLConfig config) { engine.glSurfaceCreated(); }
    @Override public void onSurfaceChanged(GL10 gl, int width, int height) { engine.glResize(width, height); }
    @Override
    public void onDrawFrame(GL10 gl) {
        engine.glRender();
        // Camera submission and SLAM publication are asynchronous. Keep drawing
        // while scanning so a cloud published after the last camera callback is
        // uploaded without waiting for another UI event.
        if (engine.scanState() == JMEngineNative.SCAN_SCANNING) {
            requestRender();
        }
    }

    @Override public boolean onTouchEvent(MotionEvent event) {
        scaleDetector.onTouchEvent(event);
        if (event.getPointerCount() > 1) return true;
        final float x = event.getX(), y = event.getY();
        switch (event.getActionMasked()) {
            case MotionEvent.ACTION_DOWN:
                downX = lastX = x; downY = lastY = y; moving = false; return true;
            case MotionEvent.ACTION_MOVE:
                if (mode == InteractionMode.ORBIT && !scaleDetector.isInProgress()) {
                    final float dx = x - lastX, dy = y - lastY;
                    if (Math.abs(dx) + Math.abs(dy) > 0.5f) moving = true;
                    queueEvent(() -> engine.glOrbit(dx, dy));
                    requestRender();
                }
                lastX = x; lastY = y; return true;
            case MotionEvent.ACTION_UP:
                if (mode != InteractionMode.ORBIT) {
                    final int x0 = Math.round(downX), y0 = Math.round(downY);
                    final int x1 = Math.round(x), y1 = Math.round(y);
                    final boolean surface = mode == InteractionMode.SURFACE_RECT;
                    queueEvent(() -> {
                        long count = engine.glSelectRect(x0, y0, x1, y1, surface);
                        post(() -> {
                            SelectionListener listener = selectionListener.get();
                            if (listener != null) listener.onSelection(count, surface);
                        });
                        requestRender();
                    });
                }
                return true;
            default:
                return true;
        }
    }
}
