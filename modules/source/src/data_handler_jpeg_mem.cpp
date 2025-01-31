/*************************************************************************
:a
 * Copyright (C) [2020] by Cambricon, Inc. All rights reserved
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *************************************************************************/
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "data_handler_jpeg_mem.hpp"
#include "profiler/module_profiler.hpp"
#include "profiler/pipeline_profiler.hpp"

namespace cnstream {

std::shared_ptr<SourceHandler> ESJpegMemHandler::Create(DataSource *module, const std::string &stream_id, int max_width,
                                                        int max_height) {
  if (!module || stream_id.empty()) {
    LOGE(SOURCE) << "source module or stream id must not be empty";
    return nullptr;
  }
  std::shared_ptr<ESJpegMemHandler> handler(new (std::nothrow)
                                                ESJpegMemHandler(module, stream_id, max_width, max_height));
  return handler;
}

ESJpegMemHandler::ESJpegMemHandler(DataSource *module, const std::string &stream_id, int max_width, int max_height)
    : SourceHandler(module, stream_id) {
  impl_ = new (std::nothrow) ESJpegMemHandlerImpl(module, this, max_width, max_height);
}

ESJpegMemHandler::~ESJpegMemHandler() {
  if (impl_) {
    delete impl_, impl_ = nullptr;
  }
}

bool ESJpegMemHandler::Open() {
  if (!this->module_) {
    LOGE(SOURCE) << "[" << stream_id_ << "]: "
                 << "module_ null";
    return false;
  }
  if (!impl_) {
    LOGE(SOURCE) << "[" << stream_id_ << "]: "
                 << "ESJpegMemHandler open failed, no memory left";
    return false;
  }

  if (stream_index_ == cnstream::INVALID_STREAM_IDX) {
    LOGE(SOURCE) << "[" << stream_id_ << "]: "
                 << "invalid stream_idx";
    return false;
  }

  return impl_->Open();
}

void ESJpegMemHandler::Close() {
  if (impl_) {
    impl_->Close();
  }
}

int ESJpegMemHandler::Write(ESPacket *pkt) {
  if (impl_) {
    return impl_->Write(pkt);
  }
  return -1;
}

bool ESJpegMemHandlerImpl::Open() {
  RwLockWriteGuard guard(running_lock_);
  DataSource *source = dynamic_cast<DataSource *>(module_);
  if (nullptr != source) {
    param_ = source->GetSourceParam();
  } else {
    LOGE(SOURCE) << "[" << stream_id_ << "]: "
                 << "source module is null";
    return false;
  }
  int ret = InitDecoder();
  if (ret) {
    running_ = true;
    eos_reached_ = false;
  }
  return ret;
}

void ESJpegMemHandlerImpl::Close() {
  RwLockWriteGuard guard(running_lock_);
  if (decoder_) {
    decoder_->Destroy();
    decoder_ = nullptr;
  }
  running_ = false;
}

int ESJpegMemHandlerImpl::Write(ESPacket *pkt) {
  if (pkt && decoder_) {
    if (ProcessImage(pkt)) return 0;
  }

  return -1;
}

bool ESJpegMemHandlerImpl::ProcessImage(ESPacket *in_pkt) {
  RwLockReadGuard guard(running_lock_);
  if (eos_reached_ || !running_) {
    return false;
  }
  if (in_pkt->flags & static_cast<size_t>(ESPacket::FLAG::FLAG_EOS)) {
    LOGI(SOURCE) << "[" << stream_id_ << "]: "
                 << "EOS reached in ESJpegMemHandler";
    decoder_->Process(nullptr);
    return false;
  }

  VideoEsPacket pkt;
  pkt.data = in_pkt->data;
  pkt.len = in_pkt->size;
  pkt.pts = in_pkt->pts;

  if (module_ && module_->GetProfiler()) {
    auto record_key = std::make_pair(stream_id_, pkt.pts);
    module_->GetProfiler()->RecordProcessStart(kPROCESS_PROFILER_NAME, record_key);
    if (module_->GetContainer() && module_->GetContainer()->GetProfiler()) {
      module_->GetContainer()->GetProfiler()->RecordInput(record_key);
    }
  }

  if (!decoder_->Process(&pkt)) {
    return false;
  }

  return true;
}

bool ESJpegMemHandlerImpl::InitDecoder() {
  if (param_.decoder_type_ == DecoderType::DECODER_MLU) {
    decoder_ = std::make_shared<MluDecoder>(stream_id_, this);
  } else if (param_.decoder_type_ == DecoderType::DECODER_CPU) {
    decoder_ = std::make_shared<FFmpegCpuDecoder>(stream_id_, this);
  } else {
    LOGE(SOURCE) << "unsupported decoder_type";
    return false;
  }
  if (!decoder_) {
    return false;
  }

  // FIXME, fill info, a parser is needed?
  VideoInfo info;
  info.codec_id = AV_CODEC_ID_MJPEG;

  ExtraDecoderInfo extra;
  extra.apply_stride_align_for_scaler = param_.apply_stride_align_for_scaler_;
  extra.device_id = param_.device_id_;
  extra.input_buf_num = param_.input_buf_number_;
  extra.output_buf_num = param_.output_buf_number_;
  extra.max_width = max_width_;
  extra.max_height = max_height_;
  bool ret = decoder_->Create(&info, &extra);
  if (!ret) {
    return false;
  }

  MluDeviceGuard guard(param_.device_id_);
  return true;
}

// IDecodeResult methods
void ESJpegMemHandlerImpl::OnDecodeError(DecodeErrorCode error_code) {
  // FIXME,  handle decode error ...
  if (nullptr != module_) {
    Event e;
    e.type = EventType::EVENT_STREAM_ERROR;
    e.module_name = module_->GetName();
    e.message = "Decode failed.";
    e.stream_id = stream_id_;
    e.thread_id = std::this_thread::get_id();
    module_->PostEvent(e);
  }
  interrupt_.store(true);
}

void ESJpegMemHandlerImpl::OnDecodeFrame(DecodeFrame *frame) {
  if (frame_count_++ % param_.interval_ != 0) {
    return;  // discard frames
  }
  if (!frame) return;
  std::shared_ptr<CNFrameInfo> data = this->CreateFrameInfo();
  if (!data) {
    return;
  }

  data->timestamp = frame->pts;  // FIXME
  if (!frame->valid) {
    data->flags = static_cast<size_t>(CNFrameFlag::CN_FRAME_FLAG_INVALID);
    this->SendFrameInfo(data);
    return;
  }
  int ret = SourceRender::Process(data, frame, frame_id_++, param_);
  if (ret < 0) {
    return;
  }
  this->SendFrameInfo(data);
}
void ESJpegMemHandlerImpl::OnDecodeEos() {
  this->SendFlowEos();
}

}  // namespace cnstream
