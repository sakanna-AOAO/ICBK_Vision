#include "HikCamera.h"
#include "Log.h"
#include <cstring>

namespace ly {

HikCamera::HikCamera() {
    handle = nullptr;
    pData = nullptr;
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    debug_mode = true; // 设置是否开启调试
    
    if(debug_mode) {
        initDebugUDP();
    }
}

void HikCamera::initDebugUDP() {
    // 创建UDP socket
    udp_client_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if(udp_client_socket < 0) {
        LOG(ERROR) << "Create UDP socket failed: " << strerror(errno);
        debug_mode = false;  // 如果创建失败则禁用调试模式
        return;
    }
    
    // 设置服务器地址(VOFA+)
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(1347);  // VOFA+默认端口
    server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    
    LOG(INFO) << "UDP debug socket initialized on port 1347";
}

void HikCamera::sendDebugData(const cv::Mat& frame, double timestamp) {
    if(!debug_mode) return;
    
    // 构造调试数据包
    struct DebugData {
        double timestamp;     // 时间戳(ms)
        int width;           // 图像宽度
        int height;          // 图像高度
        double fps;          // 帧率
        double exposure;     // 曝光时间
        double gain;         // 增益
        char tail[4];        // VOFA+数据包尾
    } debug_data;
    
    debug_data.timestamp = timestamp;
    debug_data.width = frame.cols;
    debug_data.height = frame.rows;
    debug_data.fps = 1000.0 / timestamp;
    debug_data.exposure = CameraParam::exposure_time;
    debug_data.gain = CameraParam::gain;
    memcpy(debug_data.tail, "VOFA", 4);
    
    // 发送数据到VOFA+
    ssize_t sent_bytes = sendto(udp_client_socket, &debug_data, sizeof(debug_data), 0,
                               (struct sockaddr*)&server_addr, sizeof(server_addr));
    
    if(sent_bytes < 0) {
        LOG(ERROR) << "Failed to send debug data: " << strerror(errno);
    }
}

HikCamera::~HikCamera() {
    // 停止取流
    if (handle) {
        MV_CC_StopGrabbing(handle);
    }
    
    // 关闭设备
    if (handle) {
        MV_CC_DestroyHandle(handle);
        handle = nullptr;
    }

    // 释放缓存
    if (pData) {
        free(pData);
        pData = nullptr;
    }

    if(debug_mode) {
        close(udp_client_socket);
    }
}

void HikCamera::open() {
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
    int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Enum devices fail! nRet [" << nRet << "]";
        return;
    }

    if (stDeviceList.nDeviceNum == 0) {
        LOG(ERROR) << "No camera found!";
        return;
    }

    // 创建句柄
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Create handle fail! nRet [" << nRet << "]";
        return;
    }

    // 打开设备
    nRet = MV_CC_OpenDevice(handle);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Open device fail! nRet [" << nRet << "]";
        return;
    }

    // 设置ROI
    if (!setROI(offset_x, offset_y, width, height)) {
        return;
    }

    // 设置曝光和增益
    if (!setExposureTime(CameraParam::exposure_time) || 
        !setGain(CameraParam::gain)) {
        return;
    }

    // 设置白平衡(自动模式)
    if (!setWhiteBalance(true, 0)) {
        return;
    }

    // 设置像素格式(RGB8)
    if (!setPixelFormat(0)) {
        return;
    }

    // 设置帧率
    if (!setFrameRate(210.0f)) {
        return;
    }

    // 关闭触发模式
    if (!setTriggerMode(false)) {
        return;
    }

    // 开始取流
    if (!startGrabbing()) {
        return;
    }

    // 分配数据缓存
    pData = (unsigned char*)malloc(sizeof(unsigned char) * width * height * 3);
}

void HikCamera::startCapture(Params_ToVideo &params) {
    _video_thread_params.frame_pp = params.frame_pp;

    int id = 0;
    constexpr int size = 10;
    Image frame[size];
    for(auto& m: frame) m.mat = new cv::Mat(cv::Size(width, height), CV_8UC3);

    std::chrono::steady_clock::time_point start, end;

    do {
        std::unique_lock<std::mutex> umtx_video(Thread::mtx_image);
        while(Thread::image_is_update) {
            Thread::cond_is_process.wait(umtx_video);
        }

        start = std::chrono::steady_clock::now();

        // 获取一帧图像
        MV_FRAME_OUT stOutFrame = {0};
        int nRet = MV_CC_GetImageBuffer(handle, &stOutFrame, 1000);
        if (nRet == MV_OK) {
            memcpy(pData, stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3);
            
            // 转换为OpenCV格式并处理
            cv::Mat src(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, 
                       CV_8UC3, pData);
            processImage(src);  // 图像处理
            src.copyTo(*frame[id].mat);

            // 释放图像缓存
            nRet = MV_CC_FreeImageBuffer(handle, &stOutFrame);
            if (nRet != MV_OK) {
                LOG(ERROR) << "Free image buffer fail! nRet [" << nRet << "]";
            }
        } else {
            LOG(ERROR) << "Get image failed! nRet [" << nRet << "]";
            continue;
        }

        end = std::chrono::steady_clock::now();
        
        frame[id].time_stamp = start + (end-start) / 2;
        frame[id].imu_data = SerialParam::recv_data;
        *_video_thread_params.frame_pp = &frame[id];
        
        Thread::image_is_update = true;
        Thread::cond_is_update.notify_one();
        umtx_video.unlock();
        
        id = (id+1) % size;
        
        // 调试数据
        if(debug_mode) {
            double delta_t = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()/1000.0;
            sendDebugData(*frame[id].mat, delta_t);
        }

        LOG_IF(ERROR, (*_video_thread_params.frame_pp)->mat->empty()) << "get empty picture mat!";
    } while(!(*_video_thread_params.frame_pp)->mat->empty());
}

void HikCamera::processImage(cv::Mat& src) {
    // 图像预处理
    cv::medianBlur(src, src, 3);  // 中值滤波
    cv::equalizeHist(src, src);   // 直方图均衡化
}

bool HikCamera::setTriggerMode(bool enable) {
    int nRet = MV_CC_SetEnumValue(handle, "TriggerMode", enable ? 1 : 0);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Set trigger mode fail! nRet [" << nRet << "]";
        return false;
    }
    return true;
}

bool HikCamera::setExposureTime(float exposure_time) {
    int nRet = MV_CC_SetFloatValue(handle, "ExposureTime", exposure_time);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Set exposure time fail! nRet [" << nRet << "]";
        return false;
    }
    return true;
}

bool HikCamera::setGain(float gain) {
    int nRet = MV_CC_SetFloatValue(handle, "Gain", gain);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Set gain fail! nRet [" << nRet << "]";
        return false;
    }
    return true;
}

bool HikCamera::setFrameRate(float frame_rate) {
    int nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", frame_rate);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Set frame rate fail! nRet [" << nRet << "]";
        return false;
    }
    return true;
}

bool HikCamera::setROI(int offsetX, int offsetY, int width, int height) {
    int nRet;
    nRet = MV_CC_SetIntValue(handle, "Width", width);
    nRet |= MV_CC_SetIntValue(handle, "Height", height);
    nRet |= MV_CC_SetIntValue(handle, "OffsetX", offsetX);
    nRet |= MV_CC_SetIntValue(handle, "OffsetY", offsetY);
    
    if (nRet != MV_OK) {
        LOG(ERROR) << "Set ROI fail! nRet [" << nRet << "]";
        return false;
    }
    return true;
}

bool HikCamera::setWhiteBalance(bool enable, int mode) {
    int nRet = MV_CC_SetEnumValue(handle, "WhiteBalanceMode", mode);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Set white balance mode fail! nRet [" << nRet << "]";
        return false;
    }
    nRet = MV_CC_SetEnumValue(handle, "WhiteBalanceEnable", enable ? 1 : 0);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Set white balance enable fail! nRet [" << nRet << "]";
        return false;
    }
    return true;
}

bool HikCamera::setPixelFormat(int format) {
    int nRet = MV_CC_SetEnumValue(handle, "PixelFormat", format);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Set pixel format fail! nRet [" << nRet << "]";
        return false;
    }
    return true;
}

bool HikCamera::startGrabbing() {
    int nRet = MV_CC_StartGrabbing(handle);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Start grabbing fail! nRet [" << nRet << "]";
        return false;
    }
    return true;
}

bool HikCamera::stopGrabbing() {
    int nRet = MV_CC_StopGrabbing(handle);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Stop grabbing fail! nRet [" << nRet << "]";
        return false;
    }
    return true;
}

} 