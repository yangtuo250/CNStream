/*************************************************************************
 * Copyright (C) [2021] by Cambricon, Inc. All rights reserved
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

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "cnstream_frame_va.hpp"
#include "cnstream_logging.hpp"
#include "postproc.hpp"

static inline size_t ArgMax(float* begin, int len) { return std::distance(begin, std::max_element(begin, begin + len)); }

static float softmaxResult(float* begin, const int len, int max)
{
    float m = 1.0;  // e^(begin[max] - begin[max]) == e^0, prevent overflow or underflow

    float sum = 0.0;
    for (int i = 0; i < len; i++) {
        sum += std::exp(begin[i] - begin[max]);
    }

    return m / sum;
}

class PostprocLprnetOri : public cnstream::ObjPostproc
{
   public:
    int Execute(const std::vector<float*>& net_outputs, const std::shared_ptr<edk::ModelLoader>& model,
                const cnstream::CNFrameInfoPtr& finfo, const std::shared_ptr<cnstream::CNInferObject>& obj) override;

    DECLARE_REFLEX_OBJECT_EX(PostprocLprnetOri, cnstream::ObjPostproc)
};  // classd PostprocLprnetOri

IMPLEMENT_REFLEX_OBJECT_EX(PostprocLprnetOri, cnstream::ObjPostproc)

int PostprocLprnetOri::Execute(const std::vector<float*>& net_outputs, const std::shared_ptr<edk::ModelLoader>& model,
                               const cnstream::CNFrameInfoPtr& finfo, const std::shared_ptr<cnstream::CNInferObject>& obj)
{
    // clang-format off
    static const std::string kChars[] = {
        "京", "沪", "津", "渝", "冀", "晋", "蒙", "辽", "吉", "黑",
        "苏", "浙", "皖", "闽", "赣", "鲁", "豫", "鄂", "湘", "粤",
        "桂", "琼", "川", "贵", "云", "藏", "陕", "甘", "青", "宁",
        "新",  // 31
        "0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
        "A", "B", "C", "D", "E", "F", "G", "H", "J", "K",
        "L", "M", "N", "P", "Q", "R", "S", "T", "U", "V",
        "W", "X", "Y", "Z", "I", "O", "-"  // 37
    };
    // clang-format on
    static constexpr int kNChar = 67;  // NHWC 4 18 1 68, so max index == 68 - 1
    static constexpr int kPlateLen = 8;
    const int seq_len = model->OutputShape(0).H();  // 18
    float* data = net_outputs[0];
    const int nlabel = model->OutputShape(0).C();  // 68
    LOGF_IF(POSTPROC_LPRNET_ORI, nlabel <= kNChar) << "Can not deal with this lprnet model!";
    LOGI(POSTPROC_LPRNET_ORI) << "Model H(seq_len) & C(nlabel): " << seq_len << " & " << nlabel;
    std::string plate_number = "";
    int plate_number_idx_all[seq_len], plate_number_idx[kPlateLen];
    int pre_ch_idx;
    float curr_score, score = 0.0f;
    int len = 0;
    for (int label_idx = 0; label_idx < seq_len; ++label_idx) {
        plate_number_idx_all[label_idx] = ArgMax(data + label_idx * nlabel, nlabel);
    }
    pre_ch_idx = plate_number_idx_all[0];
    if (pre_ch_idx != kNChar) {
        plate_number += kChars[pre_ch_idx];
        plate_number_idx[len] = pre_ch_idx;
        len++;
    }
    for (int label_idx = 0; label_idx < seq_len && len < kPlateLen; ++label_idx) {
        int ch_idx = plate_number_idx_all[label_idx];
        if (pre_ch_idx == ch_idx || ch_idx == kNChar) {
            if (ch_idx == kNChar) {
                pre_ch_idx = ch_idx;
            }
            continue;
        }
        curr_score = softmaxResult(data + label_idx * nlabel, nlabel, ch_idx);
        // if (kNChar - 2 == ch_idx) ch_idx = 32;  // I -> 1
        // if (kNChar - 1 == ch_idx) ch_idx = 31;  // O -> 0
        // if (curr_score < threshold_) {
        //     pre_ch_idx = ch_idx;
        //     continue;
        // }
        plate_number += kChars[ch_idx];
        plate_number_idx[len] = ch_idx;
        score += curr_score;
        len++;
        pre_ch_idx = ch_idx;
    }

    score /= len;
    std::cout << "############################plate/len/score/threshold: " << plate_number << "/" << len << "/" << score << "/"
              << threshold_ << std::endl;
    // 7 or 8
    if (len != kPlateLen && len != kPlateLen - 1) return 0;
    if (score < threshold_) return 0;
    // 1st. place must be province and 2nd. must not
    if (plate_number_idx[0] > 30 || plate_number_idx[1] <= 40) return 0;
    if (obj->collection.HasValue("plate_container")) {
        // plate_container set in PostprocMSSDPlateDetection
        // see CNStream/samples/common/postprocess/postprocess_mobilenet_ssd_plate_detection.cpp
        obj->collection.Get<decltype(obj)>("plate_container")->AddExtraAttribute("plate_number", plate_number);
        obj->collection.Get<decltype(obj)>("plate_container")->AddExtraAttribute("plate_ocr_score", std::to_string(score));
    }
    return 0;
}
