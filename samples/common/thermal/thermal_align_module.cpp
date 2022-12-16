#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
// #include <tr1/random>
#include <utility>
#include <vector>

#include "cnstream_frame_va.hpp"
#include "cnstream_module.hpp"
#include "curl/curl.h"
#include "string_operations/join.h"
#include "string_operations/split.h"

#define CLIP(x) x < 0 ? 0 : (x > 1 ? 1 : x)
#define ABZOLUTEZERO -273.15

using sensorDataMatrix = std::vector<std::vector<float>>;
using sensorData = std::pair<int64_t, sensorDataMatrix>;

namespace cnstream
{
class ThermalAlign : public cnstream::Module, public cnstream::ModuleCreator<ThermalAlign>
{
   public:
    explicit ThermalAlign(const std::string& name) : cnstream::Module(name) {}
    bool Open(cnstream::ModuleParamSet param_set) override
    {
        // initializing parameters
        if (param_set.find("get_thermal_data_url") == param_set.end() ||
            param_set.find("user_and_passwd") == param_set.end() ||
            param_set.find("thermal_sensor_height") == param_set.end() ||
            param_set.find("thermal_sensor_width") == param_set.end() ||
            param_set.find("frame_delay_milliseconds") == param_set.end() ||
            param_set.find("max_queue_size") == param_set.end()) {
            LOGE(THERMAL_ALIGN) << "\e[31mget_thermal_data_url user_and_passwd, max_queue_size, thermal_sensor_height, "
                                   "thermal_sensor_width, frame_delay_milliseconds are required for thermal align module\e[0m";
            return false;
        }
        try {
            strUrl = param_set["get_thermal_data_url"];
        } catch (const char* msg) {
            LOGE(THERMAL_ALIGN) << "\e[31mParse get_thermal_data_url failed: " << msg << "\e[0m";
            return false;
        }
        try {
            strUserPwd = param_set["user_and_passwd"];
        } catch (const char* msg) {
            LOGE(THERMAL_ALIGN) << "\e[31mParse user_and_passwd failed: " << msg << "\e[0m";
            return false;
        }
        try {
            maxQueueSize = std::stoi(param_set["max_queue_size"]);
        } catch (std::invalid_argument& e) {
            LOGE(THERMAL_ALIGN) << "\e[31mError parsing max_queue_size from " << param_set["max_queue_size"] << " to int\e[0m";
            return false;
        }
        try {
            thermalSensorHeight = std::stoi(param_set["thermal_sensor_height"]);
        } catch (std::invalid_argument& e) {
            LOGE(THERMAL_ALIGN) << "\e[31mError parsing thermal_sensor_height from " << param_set["thermal_sensor_height"]
                                << " to int\e[0m";
            return false;
        }
        try {
            thermalSensorWidth = std::stoi(param_set["thermal_sensor_width"]);
        } catch (std::invalid_argument& e) {
            LOGE(THERMAL_ALIGN) << "\e[31mError parsing thermal_sensor_width from " << param_set["thermal_sensor_width"]
                                << " to int\e[0m";
            return false;
        }
        try {
            frameDelayMilliseconds = std::stoi(param_set["frame_delay_milliseconds"]);
        } catch (std::invalid_argument& e) {
            LOGE(THERMAL_ALIGN) << "\e[31mError parsing frame_delay_milliseconds from "
                                << param_set["frame_delay_milliseconds"] << " to int\e[0m";
            return false;
        }
        // Thermal roi different from rgb main camera in most case
        // left and right
        try {
            roiThermalLeft =
                static_cast<int>(static_cast<float>(thermalSensorWidth) * std::stof(param_set["roi_thermal_left"]));
        } catch (const char* msg) {
            LOGW(THERMAL_ALIGN) << msg << "\e[0m";
        } catch (std::invalid_argument& e) {
            LOGW(THERMAL_ALIGN) << "\e[33mroi_thermal_left not set or not float\e[0m";
        }
        if (roiThermalLeft <= -thermalSensorWidth) {
            LOGE(THERMAL_ALIGN) << "\e[31mroi_thermal_left must larger than -1.0\e[0m";
            return false;
        }
        try {
            if (param_set.find("roi_thermal_right") == param_set.end() && 0 != roiThermalLeft) {
                roiThermalRight = roiThermalLeft;
                throw "roi_thermal_right not found, set to same with roi_thermal_left by default\e[0m";
            } else {
                roiThermalRight =
                    static_cast<int>(static_cast<float>(thermalSensorWidth) * std::stof(param_set["roi_thermal_right"]));
            }
        } catch (const char* msg) {
            LOGW(THERMAL_ALIGN) << "\e[33m" << msg << "\e[0m";
        } catch (std::invalid_argument& e) {
            LOGW(THERMAL_ALIGN) << "\e[33mroi_thermal_right not set or not float\e[0m";
        }
        if (roiThermalRight <= -thermalSensorWidth) {
            LOGE(THERMAL_ALIGN) << "\e[31mroi_thermal_right must larger than -1.0\e[0m";
            return false;
        }
        thermalSensorWidthNew = roiThermalLeft + roiThermalRight + thermalSensorWidth;
        if (thermalSensorWidthNew <= 0) {
            LOGE(THERMAL_ALIGN) << "\e[31mInvalid roi_thermal_left and roi_thermal_right, sum must larger than -1.0\e[0m";
            return false;
        }
        // top and bottom
        try {
            roiThermalTop =
                static_cast<int>(static_cast<float>(thermalSensorHeight) * std::stof(param_set["roi_thermal_top"]));
        } catch (const char* msg) {
            LOGW(THERMAL_ALIGN) << msg << "\e[0m";
        } catch (std::invalid_argument& e) {
            LOGW(THERMAL_ALIGN) << "\e[33mroi_thermal_top not set or not float\e[0m";
        }
        if (roiThermalTop <= -thermalSensorHeight) {
            LOGE(THERMAL_ALIGN) << "\e[31mroi_thermal_top must larger than -1.0\e[0m";
            return false;
        }
        try {
            if (param_set.find("roi_thermal_bottom") == param_set.end() && 0 != roiThermalTop) {
                roiThermalBottom = roiThermalTop;
                throw "roi_thermal_bottom not found, set to same with roi_thermal_top by default";
            } else {
                roiThermalBottom =
                    static_cast<int>(static_cast<float>(thermalSensorHeight) * std::stof(param_set["roi_thermal_bottom"]));
            }
        } catch (const char* msg) {
            LOGW(THERMAL_ALIGN) << "\e[33m" << msg << "\e[0m";
        } catch (std::invalid_argument& e) {
            LOGW(THERMAL_ALIGN) << "\e[33mroi_thermal_bottom not set or not float\e[0m";
        }
        if (roiThermalBottom <= -thermalSensorHeight) {
            LOGE(THERMAL_ALIGN) << "\e[31mroi_thermal_bottom must larger than -1.0\e[0m";
            return false;
        }
        thermalSensorHeightNew = roiThermalTop + roiThermalBottom + thermalSensorHeight;
        if (thermalSensorHeightNew <= 0) {
            LOGE(THERMAL_ALIGN) << "\e[31mInvalid roi_thermal_top and roi_thermal_bottom, sum must larger than -1.0\e[0m";
            return false;
        }
        std::cout << "\e[36mNew thermal matrix height: " << thermalSensorHeightNew << ", new width: " << thermalSensorWidthNew
                  << "\e[0m" << std::endl;
        // other unnecessary params
        // frequncy of thermal POST request
        try {
            POSTIntervalMilliseconds = std::stoi(param_set["post_interval"]);
        } catch (std::invalid_argument& e) {
            LOGW(THERMAL_ALIGN) << "\e[33mpost_interval unset or not int, falling back to default value("
                                << POSTIntervalMilliseconds << ")!\e[0m";
        }
        // starting thermal data acquiring thread
        running_.store(true);
        curlThread = std::thread(&ThermalAlign::fetchThermalData, this);

        return true;
    }
    void Close() override
    {
        running_.store(false);
        if (curlThread.joinable()) {
            curlThread.join();
        }
    }
    int Process(std::shared_ptr<cnstream::CNFrameInfo> package) override
    {
        auto frame = package->collection.Get<cnstream::CNDataFramePtr>(cnstream::kCNDataFrameTag);
        CNInferObjsPtr objs_holder = nullptr;
        if (package->collection.HasValue(kCNInferObjsTag)) {
            objs_holder = package->collection.Get<CNInferObjsPtr>(kCNInferObjsTag);
        } else {
            return 0;
        }

        int64_t frameTimeMillisecond = package->timestamp / 90 - frameDelayMilliseconds;  // 1/90000pts(1/tbn)*1000ms/s
        // std::cout << "[Process DEBUG] Frame time: " << frameTimeMillisecond << "ms" << std::endl;
        sensorDataMatrix frameTempMatrix(thermalSensorHeightNew, std::vector<float>(thermalSensorWidthNew, 0.));
        if (!getTempMatrixByTime(frameTimeMillisecond, frameTempMatrix)) {
            std::cerr << "[WARNING] Get frame temperature failed! Current Frame " << frameTimeMillisecond
                      << "ms, oldest Temperature time: " << thermalDatas.front().first << "ms" << std::endl;
        }

        for (uint32_t i = 0; i < objs_holder->objs_.size(); ++i) {
            std::shared_ptr<cnstream::CNInferObject> object = objs_holder->objs_[i];
            if (!object) continue;
            float x = CLIP(object->bbox.x);
            float y = CLIP(object->bbox.y);
            float w = CLIP(object->bbox.w);
            float h = CLIP(object->bbox.h);
            w = (x + w > 1) ? (1 - x) : w;
            h = (y + h > 1) ? (1 - y) : h;
            float centerX = x + w / 2;
            float centerY = y + h / 2;
            std::pair<float, float> centerPoint = std::make_pair(centerX, centerY);

            float bboxTemp = getTempByPoint(frameTempMatrix, centerPoint);
            // // if temperature higher than 24 or lower than 20, generate random
            // if (24. < bboxTemp) {
            //     bboxTemp = 24. + dis(gen);
            // } else if (20. > bboxTemp) {
            //     bboxTemp = 20. - dis(gen);
            // }
            if (std::abs(bboxTemp - ABZOLUTEZERO) > 0.1) {
                object->collection.Add<float>("temperature", bboxTemp);
            }
        }

        return 0;
    }

   private:
    std::queue<sensorData> thermalDatas;
    std::string strUrl, strUserPwd;
    std::mutex mtx;
    std::thread curlThread;
    std::atomic<bool> running_;
    std::chrono::system_clock::time_point timeStart;
    int maxQueueSize;
    int thermalSensorHeight, thermalSensorWidth, thermalSensorHeightNew, thermalSensorWidthNew;
    int frameDelayMilliseconds;
    int roiThermalLeft = 0, roiThermalRight = 0, roiThermalTop = 0,
        roiThermalBottom = 0; /* Thermal matrix ROI according to rgb frame, percentage by rgb,
                                negative for less than rgb region, positive for more */
    int POSTIntervalMilliseconds = 2000;
    // FIXME(yangtuo250): C++11 random cannot properly run on AARCH64
    // // std::tr1::random_device() rd;
    // std::tr1::mt19937 gen;
    // std::tr1::uniform_real<float> dis{std::tr1::uniform_real<float>(0, 1)};

    static size_t onWriteData(void* buffer, size_t size, size_t nmemb, void* lpVoid)
    {
        std::string* str = dynamic_cast<std::string*>((std::string*)lpVoid);
        if (NULL == str || NULL == buffer) {
            return -1;
        }

        char* pData = (char*)buffer;
        str->append(pData, size * nmemb);

        return nmemb;
    }

    /**
     * @brief thermal data main function
     *
     */
    void fetchThermalData()
    {
        std::string strResponseData, tempStr;
        CURL* pCurlHandle = curl_easy_init();
        // by Hikvision ISAPI doc
        curl_easy_setopt(pCurlHandle, CURLOPT_CUSTOMREQUEST, "GET");
        curl_easy_setopt(pCurlHandle, CURLOPT_URL, strUrl.c_str());
        curl_easy_setopt(pCurlHandle, CURLOPT_USERPWD, strUserPwd.c_str());
        curl_easy_setopt(pCurlHandle, CURLOPT_HTTPAUTH, CURLAUTH_DIGEST);
        curl_easy_setopt(pCurlHandle, CURLOPT_WRITEFUNCTION, onWriteData);
        curl_easy_setopt(pCurlHandle, CURLOPT_WRITEDATA, strResponseData);
        curl_easy_setopt(pCurlHandle, CURLOPT_TIMEOUT, 5L);
        curl_easy_setopt(pCurlHandle, CURLOPT_MAXREDIRS, 1L);
        curl_easy_setopt(pCurlHandle, CURLOPT_CONNECTTIMEOUT, 5L);
        // by Postman
        curl_easy_setopt(pCurlHandle, CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(pCurlHandle, CURLOPT_DEFAULT_PROTOCOL, "http");
        struct curl_slist* headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(pCurlHandle, CURLOPT_HTTPHEADER, headers);

        timeStart = std::chrono::system_clock::now();
        while (running_.load()) {
            CURLcode nRet = curl_easy_perform(pCurlHandle);
            int64_t currentTime = getCurrentMillisecond();
            if (0 == nRet) {
                if (!getThermalValueStr(strResponseData, tempStr)) {
                    std::cerr << "[fetchThermalData ERROR] Get Thermal data failed!" << std::endl;
                    strResponseData.clear();
                    continue;
                }
                if (thermalSensorHeight * thermalSensorWidth * 4 != static_cast<int>(tempStr.length())) {
                    std::cerr << "[fetchThermalData ERROR] Thermal string length! Expecting "
                              << thermalSensorHeight * thermalSensorWidth * 4 << ", got " << tempStr.length() << std::endl;
                    strResponseData.clear();
                    continue;
                }
                // std::cout << "[fetchThermalData DEBUG] Thermal string length: " << tempStr.length() << " match 4x"
                //           << thermalSensorHeight << "x" << thermalSensorWidth << std::endl;

                sensorDataMatrix dataMatrix(thermalSensorHeightNew, std::vector<float>(thermalSensorWidthNew, ABZOLUTEZERO));
                str2FloatMatrix(tempStr, dataMatrix);
                // std::cout << "[fetchThermalData DEBUG] dataMatrix: height " << dataMatrix.size() << " width "
                //           << dataMatrix[thermalSensorHeight - 1].size() << " first element " << dataMatrix[0][0]
                //           << " last element " << dataMatrix[thermalSensorHeight - 1][thermalSensorWidth - 1] << std::endl;
                sensorData dataWithTime = std::make_pair(currentTime, dataMatrix);
                // std::cout << "[fetchThermalData DEBUG] current temperature time millisecond: " << currentTime << std::endl;

                strResponseData.clear();
                mtx.lock();
                enqueue(dataWithTime);
                mtx.unlock();

                std::this_thread::sleep_for(std::chrono::milliseconds(POSTIntervalMilliseconds));
            }
        }
        curl_easy_cleanup(pCurlHandle);

        return;
    }

    /**
     * @brief abstract thermal data from all response data
     *
     * @param data full response data
     * @param result thermal data
     * @return true
     * @return false
     */
    bool getThermalValueStr(std::string& data, std::string& result)
    {
        // std::cout << "[getThermalValueStr DEBUG] START --------------------------------------------------------" <<
        // std::endl;
        std::vector<std::string> tokenArea, tokenTemp;
        split(data, tokenArea, "--boundary");
        // std::cout << "[getThermalValueStr DEBUG] data: " << data.length() << " tokenArea: " << tokenArea.size() <<
        // std::endl;
        if (5 != static_cast<int>(tokenArea.size())) {
            return false;
        }
        split(tokenArea[tokenArea.size() - 2], tokenTemp, "\r\n");
        // std::cout << "[getThermalValueStr DEBUG] tokenArea[tokenArea.size()-2]: " << tokenArea[tokenArea.size() -
        // 2].length()
        //           << " tokenTemp: " << tokenTemp.size() << std::endl;
        if (6 > static_cast<int>(tokenTemp.size())) {
            return false;
        }
        tokenTemp.assign(tokenTemp.begin() + 5, tokenTemp.end());
        result = join(tokenTemp, "\r\n");
        // std::cout << "[getThermalValueStr DEBUG] result: " << result.length() << std::endl;
        // std::cout << "[getThermalValueStr DEBUG] END ----------------------------------------------------------" <<
        // std::endl;

        return true;
    }

    void str2FloatMatrix(std::string& strData, sensorDataMatrix& floatData)
    {
        const float* floatArray = reinterpret_cast<const float*>(strData.c_str());
        // reshape original thermal data matrix to new ROI
        int heightStart = roiThermalTop > 0 ? roiThermalTop : 0;
        int heightEnd = roiThermalBottom > 0 ? (thermalSensorHeight + roiThermalTop) : thermalSensorHeightNew;
        int widthStart = roiThermalLeft > 0 ? roiThermalLeft : 0;
        int widthEnd = roiThermalRight > 0 ? (thermalSensorWidth + roiThermalLeft) : thermalSensorWidthNew;
        for (int i = heightStart; i < heightEnd; i++) {
            for (int j = widthStart; j < widthEnd; j++) {
                floatData[i][j] = floatArray[(i - roiThermalTop) * thermalSensorHeight + j - roiThermalLeft];
            }
        }
    }

    int64_t getCurrentMillisecond()
    {
        auto timeCurrent = std::chrono::system_clock::now();
        auto timestamp = timeCurrent - timeStart;
        int64_t duration = std::chrono::duration_cast<std::chrono::milliseconds>(timestamp).count();

        return duration;
    }

    void enqueue(sensorData& data)
    {
        if (static_cast<int>(thermalDatas.size()) == maxQueueSize) {
            thermalDatas.pop();
        }
        thermalDatas.push(data);
        // std::cout << "[enqueue DEBUG] thermalDatas queue size: " << thermalDatas.size() << std::endl;
    }

    /**
     * @brief find nearest temperature matrix by time
     *
     * @param timeMillisecond frame time
     * @param tempMatrix temperature matrix
     * @return true succeed
     * @return false earlier or later from queue time range
     */
    bool getTempMatrixByTime(int64_t timeMillisecond, sensorDataMatrix& tempMatrix)
    {
        mtx.lock();
        std::queue<sensorData> tmp{thermalDatas};
        mtx.unlock();

        sensorData dataPair;
        dataPair = tmp.front();
        if (timeMillisecond < dataPair.first - POSTIntervalMilliseconds) {
            std::cerr
                << "[getTempMatrixByTime ERROR] frame time elder than time of queue front! Using oldest temperatue instead"
                << std::endl;
            tempMatrix = dataPair.second;
            return false;
        }
        while (!tmp.empty()) {
            dataPair = tmp.front();
            if (POSTIntervalMilliseconds < std::abs(dataPair.first - timeMillisecond)) {
                tempMatrix = dataPair.second;
                return true;
            }
            if (1 == tmp.size()) {
                std::cerr
                    << "[getTempMatrixByTime ERROR] frame time newer than time of queue end! Using newest temperatue instead"
                    << std::endl;
                tempMatrix = dataPair.second;
                return false;
            }
            tmp.pop();
        }

        return false;
    }

    float getTempByPoint(sensorDataMatrix& dataMatrix, std::pair<float, float>& point)
    {
        int x = std::round(point.first * thermalSensorWidthNew);
        int y = std::round(point.second * thermalSensorHeightNew);
        x = (x > 0) ? x : 1;
        y = (y > 0) ? y : 1;

        return dataMatrix[y - 1][x - 1];
    }
};  // class ThermalAlign
}  // namespace cnstream
