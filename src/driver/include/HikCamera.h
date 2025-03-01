#ifndef AUTOAIM_HIKCAMERA_H
#define AUTOAIM_HIKCAMERA_H

#include "MvCameraControl.h"
#include "VideoCapture.h"
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace ly {

class HikCamera : public VideoCapture {
public:
    explicit HikCamera();
    ~HikCamera() override;
    void open() override;
    void startCapture(Params_ToVideo &params) override;

private:
    void* handle;                      // 相机句柄
    unsigned char* pData;              // 图像数据缓存
    MV_FRAME_OUT_INFO_EX stImageInfo;  // 图像信息
    
    // UDP调试相关
    int udp_client_socket;
    struct sockaddr_in server_addr;
    bool debug_mode;
    
    // 相机参数配置
    bool setTriggerMode(bool enable);
    bool setExposureTime(float exposure_time);
    bool setGain(float gain);
    bool setFrameRate(float frame_rate);
    bool setROI(int offsetX, int offsetY, int width, int height);
    bool setWhiteBalance(bool enable, int mode = 0);
    bool setPixelFormat(int format = 0);
    bool startGrabbing();
    bool stopGrabbing();
    
    // 调试功能
    void initDebugUDP();
    void sendDebugData(const cv::Mat& frame, double timestamp);
    
    // 图像处理
    void processImage(cv::Mat& src);
};

}

#endif //AUTOAIM_HIKCAMERA_H 