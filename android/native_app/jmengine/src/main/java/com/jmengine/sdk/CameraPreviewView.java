package com.jmengine.sdk;

import android.content.Context;
import android.graphics.SurfaceTexture;
import android.view.Surface;
import android.view.TextureView;

public class CameraPreviewView extends TextureView implements TextureView.SurfaceTextureListener {
    private Surface surface;
    private Listener listener;

    public interface Listener {
        void onSurfaceReady(Surface surface);
    }

    public CameraPreviewView(Context context) {
        super(context);
        setSurfaceTextureListener(this);
    }

    public void setListener(Listener listener) {
        this.listener = listener;
    }

    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture texture, int width, int height) {
        surface = new Surface(texture);
        if (listener != null) {
            listener.onSurfaceReady(surface);
        }
    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture texture) {
        if (surface != null) {
            surface.release();
        }
        surface = null;
        return true;
    }

    @Override public void onSurfaceTextureSizeChanged(SurfaceTexture texture, int width, int height) {}
    @Override public void onSurfaceTextureUpdated(SurfaceTexture texture) {}

    public Surface getSurface() {
        return surface;
    }
}
