#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cnstream_frame_va.hpp"
#include "cnstream_module.hpp"
#include "curl/curl.h"
#include "string_operations/join.h"
#include "string_operations/split.h"

#define CLIP(x) x < 0 ? 0 : (x > 1 ? 1 : x)

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
            LOGE(THERMAL_ALIGN) << "get_thermal_data_url user_and_passwd, max_queue_size, thermal_sensor_height, "
                                   "thermal_sensor_width, frame_delay_milliseconds are required for thermal align module";
            return false;
        }
        try {
            strUrl = param_set["get_thermal_data_url"];
        } catch (const char* msg) {
            LOGE(THERMAL_ALIGN) << "Parse get_thermal_data_url failed: " << msg;
            return false;
        }
        try {
            strUserPwd = param_set["user_and_passwd"];
        } catch (const char* msg) {
            LOGE(THERMAL_ALIGN) << "Parse user_and_passwd failed: " << msg;
            return false;
        }
        try {
            maxQueueSize = std::stoi(param_set["max_queue_size"]);
        } catch (const char* msg) {
            LOGE(THERMAL_ALIGN) << "Parse max_queue_size failed: " << msg;
            return false;
        }
        try {
            thermalSensorHeight = std::stoi(param_set["thermal_sensor_height"]);
        } catch (const char* msg) {
            LOGE(THERMAL_ALIGN) << "Parse thermal_sensor_height failed: " << msg;
            return false;
        }
        try {
            thermalSensorWidth = std::stoi(param_set["thermal_sensor_width"]);
        } catch (const char* msg) {
            LOGE(THERMAL_ALIGN) << "Parse thermal_sensor_width failed: " << msg;
            return false;
        }
        try {
            frameDelayMilliseconds = std::stoi(param_set["frame_delay_milliseconds"]);
        } catch (const char* msg) {
            LOGE(THERMAL_ALIGN) << "Parse frame_delay_milliseconds failed: " << msg;
            return false;
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
        sensorDataMatrix frameTempMatrix(thermalSensorHeight, std::vector<float>(thermalSensorWidth, 0.));
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
            object->collection.Add<float>("temperature", bboxTemp);
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
    int thermalSensorHeight, thermalSensorWidth;
    int frameDelayMilliseconds;

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

                sensorDataMatrix dataMatrix(thermalSensorHeight, std::vector<float>(thermalSensorWidth, 0.));
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

                std::this_thread::sleep_for(std::chrono::milliseconds(500));
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
        for (int i = 0; i < thermalSensorHeight; i++) {
            for (int j = 0; j < thermalSensorWidth; j++) {
                floatData[i][j] = floatArray[i * thermalSensorHeight + j];
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
        if (timeMillisecond < dataPair.first - 500) {
            std::cerr
                << "[getTempMatrixByTime ERROR] frame time elder than time of queue front! Using oldest temperatue instead"
                << std::endl;
            tempMatrix = dataPair.second;
            return false;
        }
        while (!tmp.empty()) {
            dataPair = tmp.front();
            if (500 < std::abs(dataPair.first - timeMillisecond)) {
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
        int x = std::round(point.first * thermalSensorWidth);
        int y = std::round(point.second * thermalSensorHeight);
        x = (x > 0) ? x : 1;
        y = (y > 0) ? y : 1;

        return dataMatrix[y - 1][x - 1];
    }
};  // class ThermalAlign
}  // namespace cnstream