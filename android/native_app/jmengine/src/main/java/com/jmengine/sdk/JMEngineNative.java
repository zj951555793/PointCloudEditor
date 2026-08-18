package com.jmengine.sdk;

/**
 * Java owner around one native JMEngine context.
 *
 * The same opaque handle owns both the edited point cloud and the GLES renderer,
 * so rendering/selection always sees the exact same soft-delete state as the
 * algorithm library.
 */
public final class JMEngineNative implements AutoCloseable {
    public static final int SCAN_IDLE = 0;
    public static final int SCAN_INITIALIZING = 1;
    public static final int SCAN_SCANNING = 2;
    public static final int SCAN_STOPPING = 3;
    public static final int SCAN_READY_FOR_RECONSTRUCTION = 4;
    public static final int SCAN_RECONSTRUCTING = 5;
    public static final int SCAN_ERROR = 6;
    static {
        System.loadLibrary("JMEngine_android");
    }

    private long handle;

    public JMEngineNative() {
        handle = nativeCreate();
        if (handle == 0L) {
            throw new IllegalStateException("Failed to create JMEngine native context");
        }
    }

    public static native String version();

    public boolean loadPointCloud(String path) {
        return nativeLoadPointCloud(requireHandle(), path);
    }

    public boolean savePly(String path) {
        return nativeSavePly(requireHandle(), path);
    }

    public long pointCount() {
        return nativePointCount(requireHandle());
    }

    public long activePointCount() {
        return nativeActivePointCount(requireHandle());
    }

    public long deletedPointCount() {
        return nativeDeletedPointCount(requireHandle());
    }

    public long selectFirst(long count) {
        return nativeSelectFirst(requireHandle(), count);
    }

    public boolean deleteSelection() {
        return nativeDeleteSelection(requireHandle());
    }

    public boolean undo() {
        return nativeUndo(requireHandle());
    }

    public boolean redo() {
        return nativeRedo(requireHandle());
    }

    public void clearSelection() {
        nativeClearSelection(requireHandle());
    }

    public String lastError() {
        return nativeLastError(requireHandle());
    }

    public boolean initializeScan(
            String calibrationPath,
            String vocabularyPath,
            int registrationMode,
            int maxInflight) {
        return nativeScanInitialize(
                requireHandle(),
                calibrationPath,
                vocabularyPath,
                registrationMode,
                maxInflight);
    }

    public boolean startScan() {
        return nativeScanStart(requireHandle());
    }

    public void stopScan() {
        nativeScanStop(requireHandle());
    }

    public boolean reconstructScan() {
        return nativeScanReconstruct(requireHandle());
    }

    public int scanState() {
        return nativeScanState(requireHandle());
    }

    public long[] scanStatistics() {
        return nativeScanStatistics(requireHandle());
    }
    /** Packed marker tuples: localId,x,y,z in the current live frame. */
    public float[] scanMarkers() {
        return nativeScanMarkers(requireHandle());
    }

    boolean submitStructuredFrame(
            byte[] rgb,
            byte[] code,
            int width,
            int height,
            long timestampUs,
            int frameId) {
        return nativeScanSubmitStructured(
                requireHandle(), rgb, code, width, height, timestampUs, frameId);
    }

    // GLES calls must be issued on the GLSurfaceView render thread, except
    // orbit/zoom which are guarded natively and can be queued by the view.
    void glSurfaceCreated() {
        nativeGlSurfaceCreated(requireHandle());
    }

    void glResize(int width, int height) {
        nativeGlResize(requireHandle(), width, height);
    }

    void glRender() {
        nativeGlRender(requireHandle());
    }

    void glOrbit(float dx, float dy) {
        nativeGlOrbit(requireHandle(), dx, dy);
    }

    void glZoom(float scale) {
        nativeGlZoom(requireHandle(), scale);
    }

    void glFit() {
        nativeGlFit(requireHandle());
    }
    long glSelectRect(int x0, int y0, int x1, int y1, boolean surfaceOnly) {
        return nativeGlSelectRect(requireHandle(), x0, y0, x1, y1, surfaceOnly);
    }
    void glClearSelection() {
        nativeGlClearSelection(requireHandle());
    }

    boolean glComputeAvailable() {
        return nativeGlComputeAvailable(requireHandle());
    }

    String glStatus() {
        return nativeGlStatus(requireHandle());
    }

    @Override
    public synchronized void close() {
        if (handle != 0L) {
            nativeDestroy(handle);
            handle = 0L;
        }
    }

    private long requireHandle() {
        if (handle == 0L) {
            throw new IllegalStateException("JMEngineNative is already closed");
        }
        return handle;
    }

    private static native long nativeCreate();
    private static native void nativeDestroy(long handle);
    private static native boolean nativeLoadPointCloud(long handle, String path);
    private static native boolean nativeSavePly(long handle, String path);
    private static native long nativePointCount(long handle);
    private static native long nativeActivePointCount(long handle);
    private static native long nativeDeletedPointCount(long handle);
    private static native long nativeSelectFirst(long handle, long count);
    private static native boolean nativeDeleteSelection(long handle);
    private static native boolean nativeUndo(long handle);
    private static native boolean nativeRedo(long handle);
    private static native void nativeClearSelection(long handle);
    private static native String nativeLastError(long handle);
    private static native boolean nativeScanInitialize(
            long handle,
            String calibration,
            String vocabulary,
            int mode,
            int maxInflight);
    private static native boolean nativeScanStart(long handle);
    private static native void nativeScanStop(long handle);
    private static native boolean nativeScanReconstruct(long handle);
    private static native int nativeScanState(long handle);
    private static native long[] nativeScanStatistics(long handle);
    private static native float[] nativeScanMarkers(long handle);
    private static native boolean nativeScanSubmitStructured(
            long handle,
            byte[] rgb,
            byte[] code,
            int width,
            int height,
            long timestampUs,
            int frameId);

    private static native void nativeGlSurfaceCreated(long handle);
    private static native void nativeGlResize(long handle, int width, int height);
    private static native void nativeGlRender(long handle);
    private static native void nativeGlOrbit(long handle, float dx, float dy);
    private static native void nativeGlZoom(long handle, float scale);
    private static native void nativeGlFit(long handle);
    private static native long nativeGlSelectRect(
            long handle,
            int x0,
            int y0,
            int x1,
            int y1,
            boolean surfaceOnly);
    private static native void nativeGlClearSelection(long handle);
    private static native boolean nativeGlComputeAvailable(long handle);
    private static native String nativeGlStatus(long handle);
}
