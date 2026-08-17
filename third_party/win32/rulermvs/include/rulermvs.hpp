// generated automatic, no modification recommended.
#ifndef _RULERMVS_RULERMVS_HPP_
#define _RULERMVS_RULERMVS_HPP_

#ifdef RULERMVS_VERSION
#undef RULERMVS_VERSION
#endif
#define RULERMVS_VERSION "1.4.30"

//* 是否包含数据接口模块 */
#define RULERMVS_ENABLE_CORE

//* 是否包含三维配准模块 */
#define RULERMVS_ENABLE_MATCH

//* 是否包含标志点识别模块 */
#define RULERMVS_ENABLE_MARKEREXTRACTOR

//* 是否包含Tracker模块 */
#define RULERMVS_ENABLE_TRACKER

//* 是否包含RGBD_MarkerFusion模块 */
#define RULERMVS_ENABLE_RGBD_MARKERFUSION

//* 是否包含多帧滤波模块 */
#define RULERMVS_ENABLE_MULTIFRAMEFILTER

//* 是否包含结构光单帧解码模块 */
#define RULERMVS_ENABLE_ONESHOT

//* 是否包含纹理映射模块 */
/* #undef RULERMVS_ENABLE_MESHRECON */

//* 是否包含相移结构光解码模块 */
#define RULERMVS_ENABLE_PHASESHIFT

#ifndef RULERMVS_ENABLE_CORE
static_assert(false, "Error: Please Check The Setting Of Project.");
#else
#include "rulermvs/core.hpp"
#include "rulermvs/size.hpp"
#include "rulermvs/util.hpp"
#include "rulermvs/pose.hpp"
#include "rulermvs/rgbd.hpp"
#include "rulermvs/rect.hpp"
#include "rulermvs/pixel.hpp"
#include "rulermvs/point.hpp"
#include "rulermvs/image.hpp"
#include "rulermvs/calib.hpp"
#include "rulermvs/camera.hpp"
#include "rulermvs/scalar.hpp"
#include "rulermvs/raster.hpp"
#include "rulermvs/trimesh.hpp"
#include "rulermvs/singleton.hpp"
#include "rulermvs/threadpool.hpp"
#include "rulermvs/pointcloud.hpp"
#endif

#ifdef RULERMVS_ENABLE_MATCH
#include "rulermvs/match.hpp"
//#include "rulermvs/corner.hpp"
#include "rulermvs/posegraph.hpp"
// #include "rulermvs/rgbdfusion.hpp"
#endif

#ifdef RULERMVS_ENABLE_MARKEREXTRACTOR
#include "rulermvs/MarkerExtractor.hpp"
#include "rulermvs/corner.hpp"
#endif

#ifdef RULERMVS_ENABLE_TRACKER
#include "rulermvs/Tracker.hpp"
#include "rulermvs/Frame.hpp"
#include "rulermvs/CameraTracker.hpp"
#include "rulermvs/GlobalMap.hpp"
#include "rulermvs/MapPoint.hpp"
#endif

#ifdef RULERMVS_ENABLE_RGBD_MARKERFUSION
#include "rulermvs/RGBD_MarkerFusion.hpp"
#endif

#ifdef RULERMVS_ENABLE_MULTIFRAMEFILTER
#include "rulermvs/multiframefilter.hpp"
#endif

#ifdef RULERMVS_ENABLE_ONESHOT
#include "rulermvs/oneshot.hpp"
#endif

#ifdef RULERMVS_ENABLE_PHASESHIFT
#include "rulermvs/phaseshift.hpp"
#endif

// 辅助宏定义
#ifdef RULERMVS_USE_VLD
#include <vld.h>
#endif

//* 测试数据路径 */
#define MVS_DATADIR "G:/rulermvs0907/rulermvs_cicle/data"

#endif  // _RULERMVS_RULERMVS_HPP_
