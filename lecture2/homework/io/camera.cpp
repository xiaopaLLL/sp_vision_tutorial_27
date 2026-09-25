#include "camera.hpp"

#include <iostream>

//example里的transfer函数我直接偷过来了
cv::Mat transfer(MV_FRAME_OUT & raw);

/*
对于这个构造函数
在example中进行了一些传入参数的设定，涉及到的输入有：
MV_CC_EnumDevices的nTLayerType参数，3个MV_CC_SetEnumValue的参数，2个MV_CC_SetFloatValue的参数，1个MV_CC_SetFrameRate的参数
我的想法将这些全都作为构造函数的参数（不确定应不应该这么干），并将example中的取值作为默认值（在camera.hpp里有写默认值）
example中的使用情况是：
nTLayerType参数: unsigned int MV_USB_DEVICE
MV_CC_SetEnumValue:
"BalanceWhiteAuto"的值: unsigned int MV_BALANCE_AUTO_CONTINUOUS
"ExposureAuto": unsigned int MV_EXPOSURE_AUTO_MODE_OFF
"GainAuto"的值: unsigned int MV_GAIN_MODE_OFF
MV_CC_SetFloatValue:
"ExposureTime"的值: float 10000
"Gain"的值: float 20
MV_CC_SetFrameRate的参数: float 60
*/

Camera::Camera(
  unsigned int nTLayerType, unsigned int nBalanceWhiteAuto, unsigned int nExposureAuto,
  unsigned int nGainAuto, float fExposureTime, float fGain, float fFrameRate)
{
  int ret;
  ret = MV_CC_EnumDevices(nTLayerType, &device_list_);
  if (ret != MV_OK) {
    throw std::runtime_error("EnumDevices Error");
    //在example中是直接return -1了，这个我去查了一下，说用类的时候报错可以这么抛
  }
  if (device_list_.nDeviceNum == 0) {
    throw std::runtime_error("DeviceListEmpty Error");
  }
  ret = MV_CC_CreateHandle(&handle_, device_list_.pDeviceInfo[0]);
  if (ret != MV_OK) {
    throw std::runtime_error("CreateHandle Error");
  }
  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    throw std::runtime_error("OpenDevice Error");
  }
  MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", nBalanceWhiteAuto);
  MV_CC_SetEnumValue(handle_, "ExposureAuto", nExposureAuto);
  MV_CC_SetEnumValue(handle_, "GainAuto", nGainAuto);
  MV_CC_SetFloatValue(handle_, "ExposureTime", fExposureTime);
  MV_CC_SetFloatValue(handle_, "Gain", fGain);
  MV_CC_SetFrameRate(handle_, fFrameRate);

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    std::cerr << "StartGrabbing Error" << std::endl;
  }
}

//析构函数
Camera::~Camera()
{
  int ret;
  ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK) {
    std::cerr << "StopGrabbing Error" << std::endl;
    //本来写的throw结果编译的时候被编译器警告了，于是又去查了一下，说是析构函数的报错不应该throw
  }

  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK) {
    std::cerr << "CloseDevice Error" << std::endl;
  }

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) {
    std::cerr << "DestroyHandle Error" << std::endl;
  }
}

//read函数
/*
在example中经过了如下几步：
1通过MV_CC_StartGrabbing函数开始采集图像（图像数据应该进入了这个叫buffer的地方）
2通过MV_CC_GetImageBuffer函数，将buffer的内容存入raw变量，raw是MV_FRAME_OUT类型的，因此要在第3步中进行转换，这里有一个nMsec参数，查了一下是超时时长的意思
  我在此处选择将nMsec作为形参传入，默认值取100
3通过transfer函数，将MV_FRAME_OUT类型的raw转化为cv::Mat类型的img，img是所需的图片
4用MV_CC_FreeImageBuffer函数释放buffer的内存

后面发现在调用的时候如果直接把StartGrabbing放在read函数里会一直弹报错，发现这个东西应该只在构造实例时只调用一遍
遂把StartGrabbing移至构造函数内
*/
cv::Mat Camera::read(unsigned int nMsec)
{
  int ret;

  //step 1（已被移至构造函数内）
  // ret = MV_CC_StartGrabbing(handle_);
  // if (ret != MV_OK) {
  //   std::cerr << "StartGrabbing Error" << std::endl;
  // }

  //step 2
  MV_FRAME_OUT raw;
  ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
  if (ret != MV_OK) {
    std::cerr << "GetImageBuffer Error" << std::endl;
  }

  //step 3
  cv::Mat img = transfer(raw);

  //step 4
  ret = MV_CC_FreeImageBuffer(handle_, &raw);
  if (ret != MV_OK) {
    std::cerr << "FreeImageBuffor Error" << std::endl;
  }

  return img;
}

//transfer from example.cpp
cv::Mat transfer(MV_FRAME_OUT & raw)
{
  MV_CC_PIXEL_CONVERT_PARAM cvt_param;
  cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

  cvt_param.nWidth = raw.stFrameInfo.nWidth;
  cvt_param.nHeight = raw.stFrameInfo.nHeight;

  cvt_param.pSrcData = raw.pBufAddr;
  cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
  cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;

  cvt_param.pDstBuffer = img.data;
  cvt_param.nDstBufferSize = img.total() * img.elemSize();
  cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

  auto pixel_type = raw.stFrameInfo.enPixelType;
  const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
    {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
    {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
    {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
    {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
  cv::cvtColor(img, img, type_map.at(pixel_type));

  return img;
}