/**
 ********************************************************************************
 * Copyright (c) 2023, Li Yunqiang, walkfish8@hotmail.com.
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the organization nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ''AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTOR BE
 * LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************
 */
#pragma once

#ifndef _RGBDSLAM_RGBDMEM_H_
#include <stack>
#include <vector>
#include <cassert>
namespace rgbdslam
{
struct MemStorage {
    MemStorage() : filePtr_(NULL), fileNum_(0), filePos_ {sizeof(size_t)} {}
    ~MemStorage() { close(); }
    bool open(const char* filePath)
    {
        close();
#ifdef _MSC_VER
        fopen_s(&filePtr_, filePath, "wb+");
#else
        filePtr_ = fopen(filePath, "wb+");
#endif
        // 文件的总数量, 这里占位, 后续在结束时覆盖该值.
        if (filePtr_) fwrite(&fileNum_, sizeof(size_t), 1, filePtr_);
        return isOpened();
    }
    void close()
    {
        if (isOpened()) {
            fseek(filePtr_, 0, SEEK_SET);
            fwrite(&fileNum_, sizeof(size_t), 1, filePtr_);
            fclose(filePtr_);
        }
        fileNum_ = 0, filePtr_ = NULL;
    }
    bool isOpened() { return filePtr_ != NULL; }
    // bool read(_Index fileID, void* data) const
    // {
    //     if (filePtr_ == NULL || data == NULL) return false;
    //     if (!(fileID >= 0 && fileID < fileNum_)) return false;
    //     fseek(filePtr_, (long)(fileNum_ * nElementByte_), SEEK_SET);
    //     fread(data, nElementByte_, 1, filePtr_);
    //     return true;
    // }
    size_t write(void* data, size_t nByte)
    {
        if (data == NULL) nByte = 0;
        if (filePtr_ == NULL) return -1;
        fseek(filePtr_, 0, SEEK_END);
        fwrite(&nByte, sizeof(size_t), 1, filePtr_);
        if (nByte > 0) fwrite(data, nByte, 1, filePtr_);
        filePos_.push_back(filePos_.back() + sizeof(size_t) + nByte);
        return fileNum_++;
    }
    // bool overwrite(_Index fileID, void*& data)
    // {
    //     if (filePtr_ == NULL || data == NULL) return false;
    //     if (!(fileID >= 0 && fileID < fileNum_)) return false;
    //     fseek(filePtr_, fileNum_ * nElementByte_, SEEK_SET);
    //     fwrite(data, nElementByte_, 1, filePtr_);
    //     return true;
    // }
    size_t getByteNum(size_t ind)
    {
        size_t byteNum = 0;
        if (!(filePtr_ && ind < fileNum_)) return byteNum;
        fseek(filePtr_, (long)filePos_[ind], SEEK_SET);
        fread(&byteNum, sizeof(size_t), 1, filePtr_);
        return byteNum;
    }
    bool getByteData(size_t ind, void* data, size_t nByte)
    {
        if (!(filePtr_ && ind < fileNum_)) return false;
        size_t byteCnt = 0;
        fseek(filePtr_, (long)filePos_[ind], SEEK_SET);
        fread(&byteCnt, sizeof(size_t), 1, filePtr_);
        if (!(data && nByte > 0 && nByte == byteCnt)) return false;
        fread(data, byteCnt, 1, filePtr_);
        return true;
    }

protected:
    FILE* filePtr_;
    size_t fileNum_;
    std::vector<size_t> filePos_;
};

struct MemAllocator {
    MemAllocator(size_t nElementByte, size_t nBlockSize,
        void* (*creator)(size_t) = ::malloc, void (*destroyer)(void*) = ::free)
        : cur_(nullptr)
        , end_(nullptr)
        , creator_(creator)
        , destroyer_(destroyer)
        , nBlockSize_(nBlockSize)
        , nElementByte_(nElementByte)
        , nBlockMemByte_(nElementByte * nBlockSize)
    {}
    MemAllocator(const MemAllocator&) = delete;
    ~MemAllocator() { free(); }
    void* alloc()
    {
        if (!lost_.empty()) {
            void* ptr = lost_.top();
            assert(ptr);
            lost_.pop();
            return ptr;
        }
        if (cur_ == end_) {
            cur_ = (char*)creator_(nBlockMemByte_);
            assert(cur_);
            end_ = cur_ + nBlockMemByte_;
            mem_.push((char*)cur_);
        }
        assert(cur_);
        auto p = cur_;
        cur_ += nElementByte_;
        return p;
    }
    void free(void* ptr)
    {
        if (ptr) lost_.push((char*)ptr);
    }
    void free()
    {
        cur_ = end_ = nullptr;
        std::stack<char*>().swap(lost_);
        while (!mem_.empty()) destroyer_(mem_.top()), mem_.pop();
    }
    void freeWhenPossible()
    {
        assert(!(mem_.empty() && cur_));
        if (mem_.empty()) return;
        size_t useNum = (mem_.size() - 1) * nBlockSize_;
        useNum += (cur_ - (char*)mem_.top()) / nElementByte_;
        if (useNum == lost_.size()) free();
    }
    char *cur_, *end_;

    std::stack<char*> mem_;
    std::stack<char*> lost_;

    void* (*creator_)(size_t);
    void (*destroyer_)(void*);

    const size_t nBlockSize_;
    const size_t nElementByte_;
    const size_t nBlockMemByte_;
};
}  // namespace rgbdslam

#endif  // _RGBDSLAM_RGBDMEM_H_
