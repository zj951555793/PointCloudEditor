#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rulermvs_core" for configuration "Release"
set_property(TARGET rulermvs_core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rulermvs_core PROPERTIES
  IMPORTED_IMPLIB_RELEASE "E:/rulermvs/install/lib/rulermvs_core.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "opencv_calib3d;opencv_core;opencv_features2d;opencv_flann;opencv_gapi;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_stitching;opencv_video;opencv_videoio;opencv_alphamat;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_dpm;opencv_face;opencv_fuzzy;opencv_hfs;opencv_img_hash;opencv_intensity_transform;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_quality;opencv_rapid;opencv_reg;opencv_rgbd;opencv_saliency;opencv_shape;opencv_stereo;opencv_structured_light;opencv_superres;opencv_surface_matching;opencv_tracking;opencv_videostab;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto"
  IMPORTED_LOCATION_RELEASE "E:/rulermvs/install/bin/rulermvs_core.dll"
  )

list(APPEND _cmake_import_check_targets rulermvs_core )
list(APPEND _cmake_import_check_files_for_rulermvs_core "E:/rulermvs/install/lib/rulermvs_core.lib" "E:/rulermvs/install/bin/rulermvs_core.dll" )

# Import target "rulermvs_match" for configuration "Release"
set_property(TARGET rulermvs_match APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rulermvs_match PROPERTIES
  IMPORTED_IMPLIB_RELEASE "E:/rulermvs/install/lib/rulermvs_match.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "rulermvs_core;FPFH::fpfh_shared;RGBSIFT::rgbsift_shared;opencv_calib3d;opencv_core;opencv_features2d;opencv_flann;opencv_gapi;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_stitching;opencv_video;opencv_videoio;opencv_alphamat;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_dpm;opencv_face;opencv_fuzzy;opencv_hfs;opencv_img_hash;opencv_intensity_transform;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_quality;opencv_rapid;opencv_reg;opencv_rgbd;opencv_saliency;opencv_shape;opencv_stereo;opencv_structured_light;opencv_superres;opencv_surface_matching;opencv_tracking;opencv_videostab;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto"
  IMPORTED_LOCATION_RELEASE "E:/rulermvs/install/bin/rulermvs_match.dll"
  )

list(APPEND _cmake_import_check_targets rulermvs_match )
list(APPEND _cmake_import_check_files_for_rulermvs_match "E:/rulermvs/install/lib/rulermvs_match.lib" "E:/rulermvs/install/bin/rulermvs_match.dll" )

# Import target "rulermvs_phaseshift" for configuration "Release"
set_property(TARGET rulermvs_phaseshift APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rulermvs_phaseshift PROPERTIES
  IMPORTED_IMPLIB_RELEASE "E:/rulermvs/install/lib/rulermvs_phaseshift.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "rulermvs_core;opencv_calib3d;opencv_core;opencv_features2d;opencv_flann;opencv_gapi;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_stitching;opencv_video;opencv_videoio;opencv_alphamat;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_dpm;opencv_face;opencv_fuzzy;opencv_hfs;opencv_img_hash;opencv_intensity_transform;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_quality;opencv_rapid;opencv_reg;opencv_rgbd;opencv_saliency;opencv_shape;opencv_stereo;opencv_structured_light;opencv_superres;opencv_surface_matching;opencv_tracking;opencv_videostab;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto"
  IMPORTED_LOCATION_RELEASE "E:/rulermvs/install/bin/rulermvs_phaseshift.dll"
  )

list(APPEND _cmake_import_check_targets rulermvs_phaseshift )
list(APPEND _cmake_import_check_files_for_rulermvs_phaseshift "E:/rulermvs/install/lib/rulermvs_phaseshift.lib" "E:/rulermvs/install/bin/rulermvs_phaseshift.dll" )

# Import target "rulermvs_rgbdfusion" for configuration "Release"
set_property(TARGET rulermvs_rgbdfusion APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rulermvs_rgbdfusion PROPERTIES
  IMPORTED_IMPLIB_RELEASE "E:/rulermvs/install/lib/rulermvs_rgbdfusion.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "rulermvs_core;rulermvs_match;opencv_calib3d;opencv_core;opencv_features2d;opencv_flann;opencv_gapi;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_stitching;opencv_video;opencv_videoio;opencv_alphamat;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_dpm;opencv_face;opencv_fuzzy;opencv_hfs;opencv_img_hash;opencv_intensity_transform;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_quality;opencv_rapid;opencv_reg;opencv_rgbd;opencv_saliency;opencv_shape;opencv_stereo;opencv_structured_light;opencv_superres;opencv_surface_matching;opencv_tracking;opencv_videostab;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto"
  IMPORTED_LOCATION_RELEASE "E:/rulermvs/install/bin/rulermvs_rgbdfusion.dll"
  )

list(APPEND _cmake_import_check_targets rulermvs_rgbdfusion )
list(APPEND _cmake_import_check_files_for_rulermvs_rgbdfusion "E:/rulermvs/install/lib/rulermvs_rgbdfusion.lib" "E:/rulermvs/install/bin/rulermvs_rgbdfusion.dll" )

# Import target "rulermvs_FaceScan" for configuration "Release"
set_property(TARGET rulermvs_FaceScan APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rulermvs_FaceScan PROPERTIES
  IMPORTED_IMPLIB_RELEASE "E:/rulermvs/install/lib/rulermvs_FaceScan.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "rulermvs_core;rulermvs_match;opencv_calib3d;opencv_core;opencv_features2d;opencv_flann;opencv_gapi;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_stitching;opencv_video;opencv_videoio;opencv_alphamat;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_dpm;opencv_face;opencv_fuzzy;opencv_hfs;opencv_img_hash;opencv_intensity_transform;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_quality;opencv_rapid;opencv_reg;opencv_rgbd;opencv_saliency;opencv_shape;opencv_stereo;opencv_structured_light;opencv_superres;opencv_surface_matching;opencv_tracking;opencv_videostab;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto"
  IMPORTED_LOCATION_RELEASE "E:/rulermvs/install/bin/rulermvs_FaceScan.dll"
  )

list(APPEND _cmake_import_check_targets rulermvs_FaceScan )
list(APPEND _cmake_import_check_files_for_rulermvs_FaceScan "E:/rulermvs/install/lib/rulermvs_FaceScan.lib" "E:/rulermvs/install/bin/rulermvs_FaceScan.dll" )

# Import target "rulermvs_Lines_MarkerFusion" for configuration "Release"
set_property(TARGET rulermvs_Lines_MarkerFusion APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rulermvs_Lines_MarkerFusion PROPERTIES
  IMPORTED_IMPLIB_RELEASE "E:/rulermvs/install/lib/rulermvs_Lines_MarkerFusion.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "rulermvs_core;rulermvs_match;rulermvs_MarkerExtractor;rulermvs_Tracker;rulermvs_multilines;opencv_calib3d;opencv_core;opencv_features2d;opencv_flann;opencv_gapi;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_stitching;opencv_video;opencv_videoio;opencv_alphamat;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_dpm;opencv_face;opencv_fuzzy;opencv_hfs;opencv_img_hash;opencv_intensity_transform;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_quality;opencv_rapid;opencv_reg;opencv_rgbd;opencv_saliency;opencv_shape;opencv_stereo;opencv_structured_light;opencv_superres;opencv_surface_matching;opencv_tracking;opencv_videostab;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto"
  IMPORTED_LOCATION_RELEASE "E:/rulermvs/install/bin/rulermvs_Lines_MarkerFusion.dll"
  )

list(APPEND _cmake_import_check_targets rulermvs_Lines_MarkerFusion )
list(APPEND _cmake_import_check_files_for_rulermvs_Lines_MarkerFusion "E:/rulermvs/install/lib/rulermvs_Lines_MarkerFusion.lib" "E:/rulermvs/install/bin/rulermvs_Lines_MarkerFusion.dll" )

# Import target "rulermvs_MarkerExtractor" for configuration "Release"
set_property(TARGET rulermvs_MarkerExtractor APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rulermvs_MarkerExtractor PROPERTIES
  IMPORTED_IMPLIB_RELEASE "E:/rulermvs/install/lib/rulermvs_MarkerExtractor.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "rulermvs_core;opencv_calib3d;opencv_core;opencv_features2d;opencv_flann;opencv_gapi;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_stitching;opencv_video;opencv_videoio;opencv_alphamat;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_dpm;opencv_face;opencv_fuzzy;opencv_hfs;opencv_img_hash;opencv_intensity_transform;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_quality;opencv_rapid;opencv_reg;opencv_rgbd;opencv_saliency;opencv_shape;opencv_stereo;opencv_structured_light;opencv_superres;opencv_surface_matching;opencv_tracking;opencv_videostab;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto"
  IMPORTED_LOCATION_RELEASE "E:/rulermvs/install/bin/rulermvs_MarkerExtractor.dll"
  )

list(APPEND _cmake_import_check_targets rulermvs_MarkerExtractor )
list(APPEND _cmake_import_check_files_for_rulermvs_MarkerExtractor "E:/rulermvs/install/lib/rulermvs_MarkerExtractor.lib" "E:/rulermvs/install/bin/rulermvs_MarkerExtractor.dll" )

# Import target "rulermvs_OralScan" for configuration "Release"
set_property(TARGET rulermvs_OralScan APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rulermvs_OralScan PROPERTIES
  IMPORTED_IMPLIB_RELEASE "E:/rulermvs/install/lib/rulermvs_OralScan.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "rulermvs_core;rulermvs_match;opencv_calib3d;opencv_core;opencv_features2d;opencv_flann;opencv_gapi;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_stitching;opencv_video;opencv_videoio;opencv_alphamat;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_dpm;opencv_face;opencv_fuzzy;opencv_hfs;opencv_img_hash;opencv_intensity_transform;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_quality;opencv_rapid;opencv_reg;opencv_rgbd;opencv_saliency;opencv_shape;opencv_stereo;opencv_structured_light;opencv_superres;opencv_surface_matching;opencv_tracking;opencv_videostab;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto"
  IMPORTED_LOCATION_RELEASE "E:/rulermvs/install/bin/rulermvs_OralScan.dll"
  )

list(APPEND _cmake_import_check_targets rulermvs_OralScan )
list(APPEND _cmake_import_check_files_for_rulermvs_OralScan "E:/rulermvs/install/lib/rulermvs_OralScan.lib" "E:/rulermvs/install/bin/rulermvs_OralScan.dll" )

# Import target "rulermvs_RGBD_MarkerFusion" for configuration "Release"
set_property(TARGET rulermvs_RGBD_MarkerFusion APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rulermvs_RGBD_MarkerFusion PROPERTIES
  IMPORTED_IMPLIB_RELEASE "E:/rulermvs/install/lib/rulermvs_RGBD_MarkerFusion.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "rulermvs_core;rulermvs_match;rulermvs_MarkerExtractor;rulermvs_Tracker;opencv_calib3d;opencv_core;opencv_features2d;opencv_flann;opencv_gapi;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_stitching;opencv_video;opencv_videoio;opencv_alphamat;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_dpm;opencv_face;opencv_fuzzy;opencv_hfs;opencv_img_hash;opencv_intensity_transform;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_quality;opencv_rapid;opencv_reg;opencv_rgbd;opencv_saliency;opencv_shape;opencv_stereo;opencv_structured_light;opencv_superres;opencv_surface_matching;opencv_tracking;opencv_videostab;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto"
  IMPORTED_LOCATION_RELEASE "E:/rulermvs/install/bin/rulermvs_RGBD_MarkerFusion.dll"
  )

list(APPEND _cmake_import_check_targets rulermvs_RGBD_MarkerFusion )
list(APPEND _cmake_import_check_files_for_rulermvs_RGBD_MarkerFusion "E:/rulermvs/install/lib/rulermvs_RGBD_MarkerFusion.lib" "E:/rulermvs/install/bin/rulermvs_RGBD_MarkerFusion.dll" )

# Import target "rulermvs_Tracker" for configuration "Release"
set_property(TARGET rulermvs_Tracker APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rulermvs_Tracker PROPERTIES
  IMPORTED_IMPLIB_RELEASE "E:/rulermvs/install/lib/rulermvs_Tracker.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "rulermvs_MarkerExtractor;rulermvs_core;opencv_calib3d;opencv_core;opencv_features2d;opencv_flann;opencv_gapi;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_stitching;opencv_video;opencv_videoio;opencv_alphamat;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_dpm;opencv_face;opencv_fuzzy;opencv_hfs;opencv_img_hash;opencv_intensity_transform;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_quality;opencv_rapid;opencv_reg;opencv_rgbd;opencv_saliency;opencv_shape;opencv_stereo;opencv_structured_light;opencv_superres;opencv_surface_matching;opencv_tracking;opencv_videostab;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto"
  IMPORTED_LOCATION_RELEASE "E:/rulermvs/install/bin/rulermvs_Tracker.dll"
  )

list(APPEND _cmake_import_check_targets rulermvs_Tracker )
list(APPEND _cmake_import_check_files_for_rulermvs_Tracker "E:/rulermvs/install/lib/rulermvs_Tracker.lib" "E:/rulermvs/install/bin/rulermvs_Tracker.dll" )

# Import target "rulermvs_multiframefilter" for configuration "Release"
set_property(TARGET rulermvs_multiframefilter APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rulermvs_multiframefilter PROPERTIES
  IMPORTED_IMPLIB_RELEASE "E:/rulermvs/install/lib/rulermvs_multiframefilter.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "rulermvs_core;rulermvs_match;opencv_calib3d;opencv_core;opencv_features2d;opencv_flann;opencv_gapi;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_stitching;opencv_video;opencv_videoio;opencv_alphamat;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_dpm;opencv_face;opencv_fuzzy;opencv_hfs;opencv_img_hash;opencv_intensity_transform;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_quality;opencv_rapid;opencv_reg;opencv_rgbd;opencv_saliency;opencv_shape;opencv_stereo;opencv_structured_light;opencv_superres;opencv_surface_matching;opencv_tracking;opencv_videostab;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto"
  IMPORTED_LOCATION_RELEASE "E:/rulermvs/install/bin/rulermvs_multiframefilter.dll"
  )

list(APPEND _cmake_import_check_targets rulermvs_multiframefilter )
list(APPEND _cmake_import_check_files_for_rulermvs_multiframefilter "E:/rulermvs/install/lib/rulermvs_multiframefilter.lib" "E:/rulermvs/install/bin/rulermvs_multiframefilter.dll" )

# Import target "rulermvs_multilines" for configuration "Release"
set_property(TARGET rulermvs_multilines APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rulermvs_multilines PROPERTIES
  IMPORTED_IMPLIB_RELEASE "E:/rulermvs/install/lib/rulermvs_multilines.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "rulermvs_core;opencv_calib3d;opencv_core;opencv_features2d;opencv_flann;opencv_gapi;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_stitching;opencv_video;opencv_videoio;opencv_alphamat;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_dpm;opencv_face;opencv_fuzzy;opencv_hfs;opencv_img_hash;opencv_intensity_transform;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_quality;opencv_rapid;opencv_reg;opencv_rgbd;opencv_saliency;opencv_shape;opencv_stereo;opencv_structured_light;opencv_superres;opencv_surface_matching;opencv_tracking;opencv_videostab;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto"
  IMPORTED_LOCATION_RELEASE "E:/rulermvs/install/bin/rulermvs_multilines.dll"
  )

list(APPEND _cmake_import_check_targets rulermvs_multilines )
list(APPEND _cmake_import_check_files_for_rulermvs_multilines "E:/rulermvs/install/lib/rulermvs_multilines.lib" "E:/rulermvs/install/bin/rulermvs_multilines.dll" )

# Import target "rulermvs_oneshot" for configuration "Release"
set_property(TARGET rulermvs_oneshot APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rulermvs_oneshot PROPERTIES
  IMPORTED_IMPLIB_RELEASE "E:/rulermvs/install/lib/rulermvs_oneshot.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "rulermvs_core;ONESHOT::oneshot_shared;opencv_calib3d;opencv_core;opencv_features2d;opencv_flann;opencv_gapi;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_stitching;opencv_video;opencv_videoio;opencv_alphamat;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_dpm;opencv_face;opencv_fuzzy;opencv_hfs;opencv_img_hash;opencv_intensity_transform;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_quality;opencv_rapid;opencv_reg;opencv_rgbd;opencv_saliency;opencv_shape;opencv_stereo;opencv_structured_light;opencv_superres;opencv_surface_matching;opencv_tracking;opencv_videostab;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto"
  IMPORTED_LOCATION_RELEASE "E:/rulermvs/install/bin/rulermvs_oneshot.dll"
  )

list(APPEND _cmake_import_check_targets rulermvs_oneshot )
list(APPEND _cmake_import_check_files_for_rulermvs_oneshot "E:/rulermvs/install/lib/rulermvs_oneshot.lib" "E:/rulermvs/install/bin/rulermvs_oneshot.dll" )

# Import target "rulermvs_rgbslam" for configuration "Release"
set_property(TARGET rulermvs_rgbslam APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rulermvs_rgbslam PROPERTIES
  IMPORTED_IMPLIB_RELEASE "E:/rulermvs/install/lib/rulermvs_rgbslam.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "rulermvs_core;RGBSIFT::rgbsift_shared;RGBICP::rgbicp_shared;opencv_calib3d;opencv_core;opencv_features2d;opencv_flann;opencv_gapi;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_stitching;opencv_video;opencv_videoio;opencv_alphamat;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_dpm;opencv_face;opencv_fuzzy;opencv_hfs;opencv_img_hash;opencv_intensity_transform;opencv_line_descriptor;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_quality;opencv_rapid;opencv_reg;opencv_rgbd;opencv_saliency;opencv_shape;opencv_stereo;opencv_structured_light;opencv_superres;opencv_surface_matching;opencv_tracking;opencv_videostab;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto"
  IMPORTED_LOCATION_RELEASE "E:/rulermvs/install/bin/rulermvs_rgbslam.dll"
  )

list(APPEND _cmake_import_check_targets rulermvs_rgbslam )
list(APPEND _cmake_import_check_files_for_rulermvs_rgbslam "E:/rulermvs/install/lib/rulermvs_rgbslam.lib" "E:/rulermvs/install/bin/rulermvs_rgbslam.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
