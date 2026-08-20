#include <jni.h>

#include <JMEngine/JMEngine.h>
#include <JMEngine/JMScanner.h>
#include <JMEngine/PointCloudIO.h>

#include "JMEngineGlesRenderer.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <vector>

namespace {

struct AndroidEngineContext {
    JMEngine::Engine engine;
    jmengine_android::GlesPointCloudRenderer renderer;
    std::mutex mutex;
    std::string lastError;
    std::uint64_t revision{1};
    std::vector<JMEngine::ScanMarker> markers;
};

AndroidEngineContext* context(jlong handle) {
    return reinterpret_cast<AndroidEngineContext*>(static_cast<std::uintptr_t>(handle));
}

jlong handleOf(AndroidEngineContext* ctx) {
    return static_cast<jlong>(reinterpret_cast<std::uintptr_t>(ctx));
}

std::string toUtf8(JNIEnv* env, jstring text) {
    if (!text)
        return {};
    const char* chars = env->GetStringUTFChars(text, nullptr);
    if (!chars)
        return {};
    std::string out(chars);
    env->ReleaseStringUTFChars(text, chars);
    return out;
}

jstring toJString(JNIEnv* env, const std::string& value) {
    return env->NewStringUTF(value.c_str());
}

} // namespace

extern "C" JNIEXPORT jstring JNICALL
Java_com_jmengine_sdk_JMEngineNative_version(JNIEnv* env, jclass) {
    return env->NewStringUTF("JMEngine 2.4.0 / Android Native Scan V20");
}

extern "C" JNIEXPORT jlong JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeCreate(JNIEnv*, jclass) {
    auto* ctx = new (std::nothrow) AndroidEngineContext();
    if (ctx) {
        ctx->engine.scanner()->setFrameCallback(
                [ctx](
                        int,
                        const JMEngine::Pose&,
                        std::shared_ptr<JMEngine::PointCloud> cloud,
                        std::shared_ptr<JMEngine::PointCloud>,
                        bool trackingOk) {
                    std::lock_guard<std::mutex> guard(ctx->mutex);
                    if (trackingOk && cloud) {
                        ctx->engine.setPointCloud(std::move(cloud));
                        ++ctx->revision;
                    }
                });
        ctx->engine.scanner()->setMarkerCallback([ctx](const JMEngine::ScanMarkerFrame& frame) {
            std::lock_guard<std::mutex> guard(ctx->mutex); ctx->markers=frame.markers;
        });
    }
    return ctx ? handleOf(ctx) : 0;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeScanInitialize(
        JNIEnv* env,
        jclass,
        jlong handle,
        jstring calibration,
        jstring vocabulary,
        jint mode,
        jint maxInflight) {
    auto* ctx = context(handle);
    if (!ctx)
        return JNI_FALSE;

    JMEngine::ScanConfig config;
    config.calibrationPath = toUtf8(env, calibration);
    config.vocabularyPath = toUtf8(env, vocabulary);
    config.maxInflightFrames = std::max(1, static_cast<int>(maxInflight));
    config.registrationMode = static_cast<JMEngine::ScanRegistrationMode>(
            std::max(0, std::min(2, static_cast<int>(mode))));

    const bool ok = ctx->engine.scanner()->initialize(config);
    if (!ok) {
        std::lock_guard<std::mutex> guard(ctx->mutex);
        ctx->lastError = ctx->engine.scanner()->lastError();
    }
    return ok ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeScanStart(JNIEnv*, jclass, jlong handle) {
    auto* ctx = context(handle);
    if (!ctx || !ctx->engine.scanner()->start())
        return JNI_FALSE;
    {
        std::lock_guard<std::mutex> guard(ctx->mutex);
        ctx->renderer.fitNextFrame();
    }
    return JNI_TRUE;
}

extern "C" JNIEXPORT void JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeScanStop(JNIEnv*, jclass, jlong handle) {
    if (auto* ctx = context(handle))
        ctx->engine.scanner()->stop();
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeScanReconstruct(
        JNIEnv*, jclass, jlong handle) {
    auto* ctx = context(handle);
    if (!ctx)
        return JNI_FALSE;

    auto* scanner = ctx->engine.scanner();
    const bool ok = scanner->reconstruct();
    if (ok) {
        auto cloud = scanner->resultCloud();
        std::lock_guard<std::mutex> guard(ctx->mutex);
        if (cloud) {
            ctx->engine.setPointCloud(std::move(cloud));
            ++ctx->revision;
        }
    }
    return ok ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jint JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeScanState(JNIEnv*, jclass, jlong handle) {
    auto* ctx = context(handle);
    return ctx ? static_cast<jint>(ctx->engine.scanner()->state()) : -1;
}

extern "C" JNIEXPORT jlongArray JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeScanStatistics(
        JNIEnv* env, jclass, jlong handle) {
    jlongArray result = env->NewLongArray(5);
    jlong values[5]{};
    if (auto* ctx = context(handle)) {
        const auto statistics = ctx->engine.scanner()->statistics();
        values[0] = statistics.submittedFrames;
        values[1] = statistics.processedFrames;
        values[2] = statistics.replacedFrames;
        values[3] = statistics.rejectedFrames;
        values[4] = statistics.livePoints;
    }
    env->SetLongArrayRegion(result, 0, 5, values);
    return result;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeScanSubmitStructured(
        JNIEnv* env,
        jclass,
        jlong handle,
        jbyteArray rgb,
        jbyteArray code,
        jint width,
        jint height,
        jlong timestampUs,
        jint frameId) {
    auto* ctx = context(handle);
    if (!ctx || !rgb || !code || width <= 0 || height <= 0)
        return JNI_FALSE;

    const std::size_t pixels =
            static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    if (static_cast<std::size_t>(env->GetArrayLength(rgb)) < pixels * 3u
            || static_cast<std::size_t>(env->GetArrayLength(code)) < pixels) {
        return JNI_FALSE;
    }

    JMEngine::CameraFrame frame;
    frame.width = width;
    frame.height = height;
    frame.timestampUs = static_cast<std::uint64_t>(timestampUs);
    frame.frameId = frameId;
    frame.rgb = std::make_shared<std::vector<std::uint8_t>>(pixels * 3u);
    frame.code = std::make_shared<std::vector<std::uint8_t>>(pixels);
    env->GetByteArrayRegion(
            rgb,
            0,
            static_cast<jsize>(pixels * 3u),
            reinterpret_cast<jbyte*>(frame.rgb->data()));
    env->GetByteArrayRegion(
            code,
            0,
            static_cast<jsize>(pixels),
            reinterpret_cast<jbyte*>(frame.code->data()));
    return ctx->engine.scanner()->submit(std::move(frame)) ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jfloatArray JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeScanMarkers(
        JNIEnv* env, jclass, jlong handle) {
    std::vector<JMEngine::ScanMarker> markers;
    if (auto* ctx = context(handle)) {
        std::lock_guard<std::mutex> guard(ctx->mutex);
        markers = ctx->markers;
    }

    jfloatArray result =
            env->NewFloatArray(static_cast<jsize>(markers.size() * 4u));
    std::vector<jfloat> values;
    values.reserve(markers.size() * 4u);
    for (const auto& marker : markers) {
        values.push_back(static_cast<float>(marker.localId));
        values.push_back(marker.point3d[0]);
        values.push_back(marker.point3d[1]);
        values.push_back(marker.point3d[2]);
    }
    if (!values.empty()) {
        env->SetFloatArrayRegion(
                result, 0, static_cast<jsize>(values.size()), values.data());
    }
    return result;
}

extern "C" JNIEXPORT void JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeDestroy(JNIEnv*, jclass, jlong handle) {
    delete context(handle);
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeLoadPointCloud(
        JNIEnv* env, jclass, jlong handle, jstring path) {
    auto* ctx = context(handle);
    if (!ctx)
        return JNI_FALSE;
    std::lock_guard<std::mutex> guard(ctx->mutex);
    ctx->lastError.clear();
    auto cloud = JMEngine::PointCloudIO::load(toUtf8(env, path), &ctx->lastError);
    if (!cloud)
        return JNI_FALSE;
    ctx->engine.setPointCloud(std::move(cloud));
    ++ctx->revision;
    ctx->renderer.fitNextFrame();
    return JNI_TRUE;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeSavePly(
        JNIEnv* env, jclass, jlong handle, jstring path) {
    auto* ctx = context(handle);
    if (!ctx)
        return JNI_FALSE;
    std::lock_guard<std::mutex> guard(ctx->mutex);
    ctx->lastError.clear();
    const auto cloud = ctx->engine.pointCloud();
    if (!cloud) {
        ctx->lastError = "No point cloud loaded";
        return JNI_FALSE;
    }
    return JMEngine::PointCloudIO::savePly(*cloud, toUtf8(env, path), &ctx->lastError)
            ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jlong JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativePointCount(JNIEnv*, jclass, jlong handle) {
    auto* ctx = context(handle);
    if (!ctx)
        return 0;
    std::lock_guard<std::mutex> guard(ctx->mutex);
    const auto cloud = ctx->engine.pointCloud();
    return cloud ? static_cast<jlong>(cloud->size()) : 0;
}

extern "C" JNIEXPORT jlong JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeActivePointCount(JNIEnv*, jclass, jlong handle) {
    auto* ctx = context(handle);
    if (!ctx)
        return 0;
    std::lock_guard<std::mutex> guard(ctx->mutex);
    const auto cloud = ctx->engine.pointCloud();
    return cloud ? static_cast<jlong>(cloud->activeCount()) : 0;
}

extern "C" JNIEXPORT jlong JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeDeletedPointCount(JNIEnv*, jclass, jlong handle) {
    auto* ctx = context(handle);
    if (!ctx)
        return 0;
    std::lock_guard<std::mutex> guard(ctx->mutex);
    const auto cloud = ctx->engine.pointCloud();
    return cloud ? static_cast<jlong>(cloud->deletedCount()) : 0;
}

extern "C" JNIEXPORT jlong JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeSelectFirst(
        JNIEnv*, jclass, jlong handle, jlong requested) {
    auto* ctx = context(handle);
    if (!ctx || requested <= 0)
        return 0;
    std::lock_guard<std::mutex> guard(ctx->mutex);
    const auto cloud = ctx->engine.pointCloud();
    if (!cloud)
        return 0;

    const std::size_t limit = static_cast<std::size_t>(requested);
    std::vector<JMEngine::PointId> ids;
    ids.reserve(std::min(limit, cloud->activeCount()));
    for (JMEngine::PointId id = 0;
         static_cast<std::size_t>(id) < cloud->size() && ids.size() < limit;
         ++id) {
        const auto& p = cloud->points()[id];
        if ((p.flags & JMEngine::PointDeleted) == 0)
            ids.push_back(id);
    }
    ctx->engine.select(ids);
    return static_cast<jlong>(ctx->engine.selection().size());
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeDeleteSelection(JNIEnv*, jclass, jlong handle) {
    auto* ctx = context(handle);
    if (!ctx)
        return JNI_FALSE;
    std::lock_guard<std::mutex> guard(ctx->mutex);
    const bool ok=ctx->engine.deleteSelection();
    if(ok) ++ctx->revision;
    return ok ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeUndo(JNIEnv*, jclass, jlong handle) {
    auto* ctx = context(handle);
    if (!ctx)
        return JNI_FALSE;
    std::lock_guard<std::mutex> guard(ctx->mutex);
    const bool ok=ctx->engine.undo();
    if(ok) ++ctx->revision;
    return ok ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeRedo(JNIEnv*, jclass, jlong handle) {
    auto* ctx = context(handle);
    if (!ctx)
        return JNI_FALSE;
    std::lock_guard<std::mutex> guard(ctx->mutex);
    const bool ok=ctx->engine.redo();
    if(ok) ++ctx->revision;
    return ok ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT void JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeClearSelection(JNIEnv*, jclass, jlong handle) {
    auto* ctx = context(handle);
    if (!ctx)
        return;
    std::lock_guard<std::mutex> guard(ctx->mutex);
    ctx->engine.clearSelection();
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeLastError(JNIEnv* env, jclass, jlong handle) {
    auto* ctx = context(handle);
    if (!ctx)
        return env->NewStringUTF("Invalid JMEngine handle");
    std::lock_guard<std::mutex> guard(ctx->mutex);
    return toJString(env, ctx->lastError);
}

// -----------------------------------------------------------------------------
// Native GLES renderer / selector.  These entry points are intentionally kept
// on the same opaque engine handle, so a Java app never has to synchronize a
// second native object with JMEngine's edited point cloud.
// -----------------------------------------------------------------------------
extern "C" JNIEXPORT void JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeGlSurfaceCreated(JNIEnv*, jclass, jlong handle) {
    if(auto* ctx=context(handle)) { std::lock_guard<std::mutex> guard(ctx->mutex); ctx->renderer.onSurfaceCreated(); }
}

extern "C" JNIEXPORT void JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeGlResize(JNIEnv*, jclass, jlong handle, jint w, jint h) {
    if(auto* ctx=context(handle)) { std::lock_guard<std::mutex> guard(ctx->mutex); ctx->renderer.onResize(w,h); }
}

extern "C" JNIEXPORT void JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeGlRender(JNIEnv*, jclass, jlong handle) {
    if(auto* ctx=context(handle)) {
        std::lock_guard<std::mutex> guard(ctx->mutex);
        const auto cloud=ctx->engine.pointCloud();
        ctx->renderer.render(cloud.get(),ctx->revision);
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeGlOrbit(JNIEnv*, jclass, jlong handle, jfloat dx, jfloat dy) {
    if(auto* ctx=context(handle)) { std::lock_guard<std::mutex> guard(ctx->mutex); ctx->renderer.orbit(dx,dy); }
}

extern "C" JNIEXPORT void JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeGlZoom(JNIEnv*, jclass, jlong handle, jfloat scale) {
    if(auto* ctx=context(handle)) { std::lock_guard<std::mutex> guard(ctx->mutex); ctx->renderer.zoom(scale); }
}

extern "C" JNIEXPORT void JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeGlFit(JNIEnv*, jclass, jlong handle) {
    if(auto* ctx=context(handle)) { std::lock_guard<std::mutex> guard(ctx->mutex); ctx->renderer.fitNextFrame(); }
}

extern "C" JNIEXPORT jlong JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeGlSelectRect(
        JNIEnv*, jclass, jlong handle, jint x0, jint y0, jint x1, jint y1, jboolean surfaceOnly) {
    auto* ctx=context(handle); if(!ctx) return 0;
    std::lock_guard<std::mutex> guard(ctx->mutex);
    const auto cloud=ctx->engine.pointCloud(); if(!cloud) return 0;
    auto ids=ctx->renderer.selectRectangle(*cloud,x0,y0,x1,y1,surfaceOnly==JNI_TRUE);
    ctx->engine.select(ids);
    return static_cast<jlong>(ids.size());
}

extern "C" JNIEXPORT void JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeGlClearSelection(JNIEnv*, jclass, jlong handle) {
    if(auto* ctx=context(handle)) {
        std::lock_guard<std::mutex> guard(ctx->mutex);
        ctx->engine.clearSelection();
        ctx->renderer.clearSelection();
    }
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeGlComputeAvailable(JNIEnv*, jclass, jlong handle) {
    auto* ctx=context(handle); if(!ctx) return JNI_FALSE;
    std::lock_guard<std::mutex> guard(ctx->mutex);
    return ctx->renderer.gles31Available()?JNI_TRUE:JNI_FALSE;
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeGlStatus(JNIEnv* env, jclass, jlong handle) {
    auto* ctx=context(handle); if(!ctx) return env->NewStringUTF("invalid handle");
    std::lock_guard<std::mutex> guard(ctx->mutex);
    return env->NewStringUTF(ctx->renderer.statusText());
}
