//
// Created by saki on 15/11/05.
//

#ifndef PUPILMOBILE_CONVERTPIPELINE_H
#define PUPILMOBILE_CONVERTPIPELINE_H

#include "libUVCCamera.h"
#include "AbstractBufferedPipeline.h"

class ConvertPipeline : virtual public AbstractBufferedPipeline {
  rivate:
    const int target_pixel_format;
    convFunc_t mFrameConvFunc;
    void updateConvFunc();

  rotected:
    virtual void on_start();
    virtual void on_stop();
    virtual int handle_frame(uvc_frame_t* frame);

  ublic:
    ConvertPipeline(const size_t& _data_bytes, const int& target_pixel_format = PIXEL_FORMAT_RAW);
    virtual ~ConvertPipeline();
};

endif // PUPILMOBILE_CONVERTPIPELINE_H
