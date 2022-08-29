#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "cnstream_frame_va.hpp"
#include "cnstream_logging.hpp"
#include "video_postproc.hpp"

/**
 * @brief Video postprocessing for YOLOv5 neural network
 * The input frame of the model should keep aspect ratio.
 */
class VideoPostprocYolov5Mono : public cnstream::VideoPostproc
{
   public:
    /**
     * @brief User process. Postprocess on outputs of YOLOv5 neural network and fill data to frame.
     *
     * @param output_data: the raw output data from neural network
     * @param model_output: the raw neural network output data
     * @param model_info: model information, e.g., input/output number, shape and etc.
     *
     * @return return true if succeed
     */
    virtual bool Init(const std::unordered_map<std::string, std::string>& params) override;
    bool Execute(infer_server::InferData* output_data, const infer_server::ModelIO& model_output,
                 const infer_server::ModelInfo* model_info) override;

   private:
    // camera basic parameters
    float sourceImageHeight, sourceImageWidth;
    Eigen::Matrix3f cameraInternalParam, cameraInternalParamInv;
    Eigen::Matrix4f cameraExtrinsicParam, cameraExtrinsicParamInv;
    float cameraHeight;
    float cameraAngle;
    float cameraFocalLengthX, cameraFocalLengthY;

    template <typename M, int n>
    M matrixFromString(const std::string& str);
    bool estimateDistanceHeight(cnstream::CNInferBoundingBox& bbox, float& distance, float& height, float& width);
    DECLARE_REFLEX_OBJECT_EX(VideoPostprocYolov5Mono, cnstream::VideoPostproc);
};  // class VideoPostprocYolov5Mono

IMPLEMENT_REFLEX_OBJECT_EX(VideoPostprocYolov5Mono, cnstream::VideoPostproc);

/**
 * @brief convert camera_internal(extrinsic)_param in postprocess json param custom_postproc_params to Eigen Matrix
 *
 * @tparam M Eigen Matrix
 * @tparam n number lines of matrix
 * @param str e.g. "1.0,3.55,-0.71231,0.0,0.1,0.1,0.0,0.0,1.0"
 * @return M Eigen Matrix with n rows
 */
template <typename M, int n>
M VideoPostprocYolov5Mono::matrixFromString(const std::string& str)
{
    std::vector<float> values;
    std::stringstream ss(str);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        values.push_back(std::stof(cell));
    }

    return Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(
        values.data(), n, values.size() / n);
}

bool VideoPostprocYolov5Mono::Init(const std::unordered_map<std::string, std::string>& params)
{
    if (params.end() == params.find("camera_internal_param") || params.end() == params.find("camera_extrinsic_param") ||
        params.end() == params.find("camera_height") || params.end() == params.find("camera_angle") ||
        params.end() == params.find("source_image_height") || params.end() == params.find("source_image_width")) {
        LOGE(DEMO)
            << "\e[31mEither camera_internal_param, camera_extrinsic_param, camera_height, camera_angle source_image_height "
               "and source_image_width are needed to be in custom_postproc_params\e[0m";
        return false;
    }
    // camera parameter
    try {
        cameraInternalParam = matrixFromString<Eigen::Matrix3f, 3>(params.at("camera_internal_param"));
    } catch (...) {
        LOGE(DEMO) << "\e[31mError parsing camera_internal_param\e[0m";
        return false;
    }
    cameraFocalLengthX = cameraInternalParam(0, 0);
    cameraFocalLengthY = cameraInternalParam(1, 1);
    try {
        cameraExtrinsicParam = matrixFromString<Eigen::Matrix4f, 4>(params.at("camera_extrinsic_param"));
    } catch (...) {
        LOGE(DEMO) << "\e[31mError parsing camera_extrinsic_param\e[0m";
        return false;
    }
    // inverse of camera parameter
    cameraInternalParamInv = cameraInternalParam.inverse();
    cameraExtrinsicParamInv = cameraExtrinsicParam.inverse();
    // camera pose parameter
    try {
        cameraHeight = std::stof(params.at("camera_height"));
    } catch (std::invalid_argument& e) {
        LOGE(DEMO) << "\e[31mError converting camera_height from " << params.at("camera_height") << " to float\e[0m";
        return false;
    }
    try {
        cameraAngle = std::stof(params.at("camera_angle"));
    } catch (std::invalid_argument& e) {
        LOGE(DEMO) << "\e[31mError converting camera_angle from " << params.at("camera_angle") << " to float\e[0m";
        return false;
    }
    // source image size
    try {
        sourceImageHeight = std::stof(params.at("source_image_height"));
    } catch (std::invalid_argument& e) {
        LOGE(DEMO) << "\e[31mError converting source_image_height from " << params.at("camera_height") << " to float\e[0m";
        return false;
    }
    try {
        sourceImageWidth = std::stof(params.at("source_image_width"));
    } catch (std::invalid_argument& e) {
        LOGE(DEMO) << "\e[31mError converting source_image_width from " << params.at("source_image_width") << " to float\e[0m";
        return false;
    }
    std::cout << "\e[36mcameraInternalParam:" << std::endl
              << cameraInternalParam << std::endl
              << "cameraInternalParamInv:" << std::endl
              << cameraInternalParamInv << std::endl
              << "cameraExtrinsicParam:" << std::endl
              << cameraExtrinsicParam << std::endl
              << "cameraExtrinsicParamInv:" << std::endl
              << cameraExtrinsicParamInv << std::endl
              << "sourceImageHeight: " << sourceImageHeight << std::endl
              << "sourceImageWidth: " << sourceImageWidth << std::endl
              << "cameraHeight: " << cameraHeight << std::endl
              << "cameraAngle: " << cameraAngle << "\e[0m" << std::endl;

    return true;
}

/**
 * @brief estimate object distance and height by geometric method
 *
 * @param bbox object up left corner coordinate, width and height by percentage of image(scaled to 1)
 * @param distance distance from projection of IPC on extrinsic parameter x-y plane
 * @param height height from extrinsic parameter x-y plane
 * @return true succeed
 * @return false TODO(liuyk): add circumstances for failure estimating depth or height
 */
bool VideoPostprocYolov5Mono::estimateDistanceHeight(cnstream::CNInferBoundingBox& bbox, float& distance, float& height,
                                                     float& width)
{
    float angleB, angleC;
    float depth;  // Depth estimated of object on camera coordinate system
    float x, y, w, h;
    Eigen::Vector3f coordinatePhotoHomogeneous;  // Key point(bottom center object bounding box) coordinate on photo
    Eigen::Vector3f coordinateCamera;            // Key point on camera coordinate system
    Eigen::Vector4f coordinateCameraHomogeneous;
    Eigen::Vector4f
        coordinateWorldHomogeneous;  // Key point on world coordinate system(according to camera extrinsic parameter)

    // Center coordinate of bounding box on photo
    x = sourceImageWidth * (bbox.x + bbox.w / 2);
    y = sourceImageHeight * (bbox.y + bbox.h);
    w = sourceImageWidth * bbox.w;
    h = sourceImageHeight * bbox.h;
    // Estimate depth
    angleB = std::atan((y - sourceImageHeight / 2) / cameraFocalLengthY);
    angleC = cameraAngle + angleB;
    depth = (cameraHeight / std::sin(angleC)) * std::cos(angleB);
    // From photo to camera coordinate system
    coordinatePhotoHomogeneous << depth * x, depth * y, depth * 1;
    coordinateCamera = cameraInternalParamInv * coordinatePhotoHomogeneous;
    // From camera to world coordinate system
    coordinateCameraHomogeneous << coordinateCamera(0, 0), coordinateCamera(1, 0), coordinateCamera(2, 0), 1;
    coordinateWorldHomogeneous = cameraExtrinsicParamInv * coordinateCameraHomogeneous;
    // calculate distance from projection of IPC on floor
    distance = std::sqrt(std::pow(coordinateWorldHomogeneous(0, 0), 2) + std::pow(coordinateWorldHomogeneous(1, 0), 2));
    // calculate height from floor(extrinsic param)
    // TODO(yangtuo250): precisely solid geometry needed, but costly
    height = h * distance / cameraFocalLengthY / std::cos(cameraAngle);
    width = w * distance / cameraFocalLengthX;
    // std::cout << "x, y:" << x << " " << y << " angleB, angleC:" << angleB << " " << angleC << " depth:" << depth
    //           << " distance, height:" << distance << " " << height << std::endl;

    return true;
}

bool VideoPostprocYolov5Mono::Execute(infer_server::InferData* output_data, const infer_server::ModelIO& model_output,
                                      const infer_server::ModelInfo* model_info)
{
    LOGF_IF(DEMO, model_info->InputNum() != 1) << "VideoPostprocYolov5Mono: model input number is not equal to 1";
    LOGF_IF(DEMO, model_info->OutputNum() != 1) << "VideoPostprocYolov5Mono: model output number is not equal to 1";
    LOGF_IF(DEMO, model_output.buffers.size() != 1) << "VideoPostprocYolov5Mono: model result size is not equal to 1";

    cnstream::CNFrameInfoPtr frame = output_data->GetUserData<cnstream::CNFrameInfoPtr>();
    cnstream::CNInferObjsPtr objs_holder = frame->collection.Get<cnstream::CNInferObjsPtr>(cnstream::kCNInferObjsTag);
    cnstream::CNObjsVec& objs = objs_holder->objs_;

    const auto input_sp = model_info->InputShape(0);
    const int img_w = frame->collection.Get<cnstream::CNDataFramePtr>(cnstream::kCNDataFrameTag)->width;
    const int img_h = frame->collection.Get<cnstream::CNDataFramePtr>(cnstream::kCNDataFrameTag)->height;

    int w_idx = 2;
    int h_idx = 1;
    if (model_info->InputLayout(0).order == infer_server::DimOrder::NCHW) {
        w_idx = 3;
        h_idx = 2;
    }
    const int model_input_w = static_cast<int>(input_sp[w_idx]);
    const int model_input_h = static_cast<int>(input_sp[h_idx]);

    const float* net_output = reinterpret_cast<const float*>(model_output.buffers[0].Data());

    // scaling factors
    const float scaling_factors = std::min(1.0 * model_input_w / img_w, 1.0 * model_input_h / img_h);

    // The input frame of the model should keep aspect ratio.
    // If mlu resize and convert operator is used as preproc, parameter keep_aspect_ratio of Inferencer2 module
    // should be set to true in config json file.
    // If cpu preproc is used as preproc, please make sure keep aspect ratio in custom preproc.
    // Scaler does not support keep aspect ratio.
    // If the input frame does not keep aspect ratio, set scaled_w = model_input_w and scaled_h = model_input_h

    // scaled size
    const int scaled_w = scaling_factors * img_w;
    const int scaled_h = scaling_factors * img_h;

    // bounding boxes
    const int box_num = static_cast<int>(net_output[0]);
    int box_step = 7;
    auto range_0_1 = [](float num) { return std::max(.0f, std::min(1.0f, num)); };

    for (int box_idx = 0; box_idx < box_num; ++box_idx) {
        float left = net_output[64 + box_idx * box_step + 3];
        float right = net_output[64 + box_idx * box_step + 5];
        float top = net_output[64 + box_idx * box_step + 4];
        float bottom = net_output[64 + box_idx * box_step + 6];

        // rectify
        left = (left - (model_input_w - scaled_w) / 2) / scaled_w;
        right = (right - (model_input_w - scaled_w) / 2) / scaled_w;
        top = (top - (model_input_h - scaled_h) / 2) / scaled_h;
        bottom = (bottom - (model_input_h - scaled_h) / 2) / scaled_h;
        left = range_0_1(left);
        right = range_0_1(right);
        top = range_0_1(top);
        bottom = range_0_1(bottom);

        auto obj = std::make_shared<cnstream::CNInferObject>();
        obj->id = std::to_string(static_cast<int>(net_output[64 + box_idx * box_step + 1]));
        // detect the first class(person for COCO) only
        if ("0" != obj->id) continue;
        obj->score = net_output[64 + box_idx * box_step + 2];

        obj->bbox.x = left;
        obj->bbox.y = top;
        obj->bbox.w = std::min(1.0f - obj->bbox.x, right - left);
        obj->bbox.h = std::min(1.0f - obj->bbox.y, bottom - top);

        if (obj->bbox.h <= 0 || obj->bbox.w <= 0 || (obj->score < threshold_ && threshold_ > 0)) continue;
        // calculate object distance and height
        float distance, height, width;
        LOGF_IF(DEMO, !estimateDistanceHeight(obj->bbox, distance, height, width))
            << "\e[33mCannot get object distance and height\e[0m";
        obj->collection.Add<float>("distance", distance);
        obj->collection.Add<float>("height", height);
        obj->collection.Add<float>("width", width);

        std::lock_guard<std::mutex> objs_mutex(objs_holder->mutex_);
        objs.push_back(obj);
    }

    return true;
}
