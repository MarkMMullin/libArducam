#include "Arducam5642.h"
#include "OV5642_regs.h"
const uint16_t OV5642_CHIPID_HIGH = 0x300A;
const uint16_t OV5642_CHIPID_LOW =  0x300B;


Arducam::ECameraType Arducam5642::GetCameraType() const
{
  return ECameraType::OV5642;
}


Arducam5642::Arducam5642(int cameraNumber,CameraBank* cameraBank,int SPIFD,int I2CFD) : Arducam(cameraNumber,cameraBank,SPIFD,I2CFD)
{
  m_sensor_addr = 0x3C;
  setResolution(Arducam::getDefaultResolution());
}
// implemented by each camera to get the high byte of the camera ID word and test it against the word for this camera

bool Arducam5642::testHighDeviceIdByte()
{
  return getHighDeviceIdByte() == 0x56;
}
uint8_t Arducam5642::getHighDeviceIdByte()
{
   uint8_t regVal;
  rdSensorReg16_8(OV5642_CHIPID_HIGH, &regVal);
  return regVal;
}
// implemented by each camera to get the low byte of the camera ID word and test it against the word for this camera
 bool Arducam5642::testLowDeviceIdByte()
{
  return getLowDeviceIdByte() == 0x42;
}
uint8_t Arducam5642::getLowDeviceIdByte()
{
  uint8_t regVal;
  rdSensorReg16_8(OV5642_CHIPID_LOW, &regVal);
  return regVal;
}
 void Arducam5642::initializeSensor()
 {
   uint8_t reg_val;
#if LOG_INFO
     fprintf(stderr,"info:: initial programming for 5642\n");
#endif
     // software reset
   wrSensorReg16_8(0x3008, 0x80);
   wrSensorRegs16_8(OV5642_QVGA_Preview);
   delayms(100);
   if (sm_isJPEG)
     {
       
       wrSensorRegs16_8(OV5642_JPEG_Capture_QSXGA);
       wrSensorRegs16_8(OV5642_320x240_JPEG);
       wrSensorReg16_8(0x3818, 0xa8);
       wrSensorReg16_8(0x3621, 0x10);
       wrSensorReg16_8(0x3801, 0xb0);
       //#if (defined(OV5642_MINI_5MP_PLUS) || (defined ARDUCAM_SHIELD_V2))
       wrSensorReg16_8(0x4407, 0x04);
       //#else
       //       wrSensorReg16_8(0x4407, 0x0C);
       //#endif
     }
   else
     {
       wrSensorReg16_8(0x4740, 0x21);
       wrSensorReg16_8(0x501e, 0x2a);
       wrSensorReg16_8(0x5002, 0xf8);
       wrSensorReg16_8(0x501f, 0x01);
       wrSensorReg16_8(0x4300, 0x61);
       rdSensorReg16_8(0x3818, &reg_val);
       wrSensorReg16_8(0x3818, (reg_val | 0x60) & 0xff);
       rdSensorReg16_8(0x3621, &reg_val);
       wrSensorReg16_8(0x3621, reg_val & 0xdf);
     }

#if LOG_INFO
     fprintf(stderr,"info:: completed program load\n");
#endif
}
void Arducam5642::setQuantization(uint8_t value)
{
  // wrSensorReg16_8(0xff, 0x00);
  //wrSensorReg16_8(0x44, value);
}
void Arducam5642::changeResolution(EResolution newResolution)
{
  setResolution(newResolution);
  wrSensorRegs16_8(m_resolutionProgram);
}
bool Arducam5642::setResolution(EResolution resolution) {
  const sensor_reg* resolutionProgram = NULL;
  switch (resolution) {
  case R160x120:
    break;
  case R176x144:
    break;
  case R320x240:
    resolutionProgram = OV5642_320x240_JPEG;
    break;
  case R352x288:
    break;
  case R640x480:
    resolutionProgram = OV5642_640x480_JPEG;
    break;
  case R800x600:
    break;
  case R1024x768:
    resolutionProgram = OV5642_1024x768_JPEG;
    break;
  case R1280x1024:
    break;
  case R1600x1200:
    resolutionProgram = OV5642_1600x1200_JPEG;
    break;
  case R1280x960:
    resolutionProgram = OV5642_1280x960_JPEG;
    break;
  case R2048x1536:
    resolutionProgram = OV5642_2048x1536_JPEG;
    break;
  case R2592x1944:
    resolutionProgram = OV5642_2592x1944_JPEG;
    break;
  default:
    return false;
  }
  m_resolutionProgram = resolutionProgram;
  m_resolution = resolution;
  //setQuantization(40);
  return true;
}
void Arducam5642::programSensor(const struct sensor_reg regList[])
{
  wrSensorRegs16_8(regList);
}
