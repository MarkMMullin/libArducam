/*
  This file is part of Abaddon.

    Abaddon is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Abaddon is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

    This code is an extreme fork of original code at https://github.com/ArduCAM
    and all changes are Copyright 2016 Mark M. Mullin (mark.m.mullin@gmail.com)
 */
#include "Arducam2640.h"

const uint8_t OV2640_CHIPID_HIGH = 0x0A;
const uint8_t OV2640_CHIPID_LOW =  0x0B;

// bring in the register sets
#include "CameraProgram2640.h"

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
     wrSensorReg8_8(0xff, 0x01);
     wrSensorReg8_8(0x12, 0x80);   // system reset
     delayms(100);
     
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
     wrSensorReg8_8(0xff, 0x01);
     wrSensorReg8_8(0x15, 0x00);
#if LOG_INFO
     fprintf(stderr,"info:: load resolution specific init\n");
#endif
     wrSensorRegs8_8(m_resolutionProgram);
     //wrSensorReg8_8(0xff, 0x00);
     //wrSensorReg8_8(0x44, 0x32);
   } else
     {
       wrSensorRegs8_8(OV2640_QVGA);
     }
#if LOG_INFO
     fprintf(stderr,"info:: completed program load\n");
#endif
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
