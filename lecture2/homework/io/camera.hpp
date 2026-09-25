/*
因为不太熟悉oop相关的东西，然后也缺乏这种多文件管理的代码意识，所以想借此学习一下
so会尝试不用ai纯手搓这一期的作业，写的也许会有点烂
因为这个linux上没有装中文输入法，所以这些注释都是在qq上打完然后复制的XD（感觉有点呆比
*/

#include <opencv2/opencv.hpp>

#include "hikrobot/include/MvCameraControl.h"

class Camera
{
private:
  /*
  翻阅了一下各种变量函数的定义，以我对example代码里这个粗略的理解
  MV_CC_DEVICE_INFO_LIST device_list是一个类似全局变量，记录了在线设备的数量，因此在封装时我采用静态变量的形式（不知道能不能这么干）,变量名是device_list_
  对每台相机，由void * handle唯一决定了这台相机的“权柄”(?)，疑似相机由handle唯一确定，不是很理解这个词，但是它是源码的注释用词，我把它封装成了private变量handle_
  还有一个int ret，貌似是状态返回值，正常运行时返回MV_OK，就放在构造函数的局部变量里了
  */
  inline static MV_CC_DEVICE_INFO_LIST device_list_;
  void * handle_;

public:
  Camera(
    unsigned int nTLayerType = MV_USB_DEVICE,
    unsigned int nBalanceWhiteAuto = MV_BALANCEWHITE_AUTO_CONTINUOUS,
    unsigned int nExposureAuto = MV_EXPOSURE_AUTO_MODE_OFF,
    unsigned int nGainAuto = MV_GAIN_MODE_OFF, float fExposureTime = 10000, float fGain = 20,
    float fFrameRate = 60);

  // Camera():nTLayerType(MV-USB),Ffra(60){}
  ~Camera();
  cv::Mat read(unsigned int nMsec = 100);
};