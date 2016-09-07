#include "Arducam2640.h"

#define LOG_INFO 1
const uint8_t OV2640_CHIPID_HIGH = 0x0A;
const uint8_t OV2640_CHIPID_LOW =  0x0B;


// LOAD ALL OF THE PROGRAM DEFINITIONS FOR THE OV2640

#include "sensor_programs/registerDefinitions.h"
#include "sensor_programs/OV2640_1024x768_JPEG.regpgm.h"
#include "sensor_programs/OV2640_1280x1024_JPEG.regpgm.h"
#include "sensor_programs/OV2640_1600x1200_JPEG.regpgm.h"
#include "sensor_programs/OV2640_160x120_JPEG.regpgm.h"
#include "sensor_programs/OV2640_176x144_JPEG.regpgm.h"
#include "sensor_programs/OV2640_320x240_JPEG.regpgm.h"
#include "sensor_programs/OV2640_352x288_JPEG.regpgm.h"
#include "sensor_programs/OV2640_640x480_JPEG.regpgm.h"
#include "sensor_programs/OV2640_800x600_JPEG.regpgm.h"

#include "sensor_programs/OV2640_JPEG.regpgm.h"
#include "sensor_programs/OV2640_JPEG_INIT.regpgm.h"
#include "sensor_programs/OV2640_QVGA.regpgm.h"
#include "sensor_programs/OV2640_YUV422.regpgm.h"



Arducam2640::Arducam2640(int cameraNumber,CameraBank* cameraBank,int SPIFD,int I2CFD) : Arducam(cameraNumber,cameraBank,SPIFD,I2CFD)
{
  m_sensor_addr = SENSOR_ADDRESS;
  setResolution(Arducam::getDefaultResolution());
}
// implemented by each camera to get the high byte of the camera ID word and test it against the word for this camera

bool Arducam2640::testHighDeviceIdByte()
{
  return getHighDeviceIdByte() == 0x26;
}
uint8_t Arducam2640::getHighDeviceIdByte()
{
   uint8_t regVal;
  rdSensorReg8_8(OV2640_CHIPID_HIGH, &regVal);
  return regVal;
}
// implemented by each camera to get the low byte of the camera ID word and test it against the word for this camera
 bool Arducam2640::testLowDeviceIdByte()
{
  return getLowDeviceIdByte() == 0x42;
}
uint8_t Arducam2640::getLowDeviceIdByte()
{
  uint8_t regVal;
  rdSensorReg8_8(OV2640_CHIPID_LOW, &regVal);
  return regVal;
}
 void Arducam2640::initializeSensor()
 {
   if(sm_isJPEG) {
     // sensor system reset
     wrSensorReg8_8(0xff, 0x01);
     wrSensorReg8_8(0x12, 0x80);   // system reset
     delayms(1000);
     
#if LOG_INFO
     fprintf(stderr,"info:: load jpeg init\n");
#endif
     wrSensorRegs8_8(OV2640_JPEG_INIT);
     
#if LOG_INFO
     fprintf(stderr,"info:: load yuv422 init\n");
#endif
     wrSensorRegs8_8(OV2640_YUV422);
     
#if LOG_INFO
     fprintf(stderr,"info:: load jpeg\n");
#endif
     wrSensorRegs8_8(OV2640_JPEG);
     // write COM10 - CHSYNC AND HREF NOT SWAPPED,PCLK OUT,ON FALLING CLOCK,POSITIVE HREF,POSITIVE VSYNC,POSITIVE HSYNC
     wrSensorReg8_8(0xff, 0x01);
     wrSensorReg8_8(0x15, 0x00);
#if LOG_INFO
     fprintf(stderr,"info:: load resolution specific init\n");
#endif
     wrSensorRegs8_8(m_resolutionProgram);
   } else
     {
       wrSensorRegs8_8(OV2640_QVGA);
     }
#if LOG_INFO
     fprintf(stderr,"info:: completed program load\n");
#endif
}
void Arducam2640::setQuantization(uint8_t value)
{
     wrSensorReg8_8(0xff, 0x00);
     wrSensorReg8_8(0x44, value);
}
void Arducam2640::changeResolution(EResolution newResolution)
{
  setResolution(newResolution);
  wrSensorRegs8_8(m_resolutionProgram);
}
bool Arducam2640::setResolution(EResolution resolution) {
  const sensor_reg* resolutionProgram = NULL;
  switch (resolution) {
  case R160x120:
    resolutionProgram = OV2640_160x120_JPEG;
    break;
  case R176x144:
    resolutionProgram = OV2640_176x144_JPEG;
    break;
  case R320x240:
    resolutionProgram = OV2640_320x240_JPEG;
    break;
  case R352x288:
    resolutionProgram = OV2640_352x288_JPEG;
    break;
  case R640x480:
    resolutionProgram = OV2640_640x480_JPEG;
    break;
  case R800x600:
    resolutionProgram = OV2640_800x600_JPEG;
    break;
  case R1024x768:
    resolutionProgram = OV2640_1024x768_JPEG;
    break;
  case R1280x1024:
    resolutionProgram = OV2640_1280x1024_JPEG;
    break;
  case R1600x1200:
    resolutionProgram = OV2640_1600x1200_JPEG;
    break;
  default:
    return false;
  }
  m_resolutionProgram = resolutionProgram;
  m_resolution = resolution;
  return true;
}
void Arducam2640::programSensor(const struct sensor_reg regList[])
{
  wrSensorRegs8_8(regList);
}
