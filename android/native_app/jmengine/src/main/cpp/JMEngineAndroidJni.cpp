#include <jni.h>

#include <JMEngine/JMEngine.h>
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
    return env->NewStringUTF("JMEngine 2.3.6 / Android Native GLES V14");
}

extern "C" JNIEXPORT jlong JNICALL
Java_com_jmengine_sdk_JMEngineNative_nativeCreate(JNIEnv*, jclass) {
    auto* ctx = new (std::nothrow) AndroidEngineContext();
    return ctx ? handleOf(ctx) : 0;
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
