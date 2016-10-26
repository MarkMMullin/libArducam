#include "Arducam5640.h"
#include "OV5640_regs.h"
const uint16_t OV5640_CHIPID_HIGH = 0x300A;
const uint16_t OV5640_CHIPID_LOW =  0x300B;


Arducam::ECameraType Arducam5640::GetCameraType() const
{
  return ECameraType::OV5640;
}


Arducam5640::Arducam5640(int cameraNumber,CameraBank* cameraBank,int SPIFD,int I2CFD) : Arducam(cameraNumber,cameraBank,SPIFD,I2CFD)
{
  m_sensor_addr = SENSOR_ADDRESS;
  setResolution(Arducam::getDefaultResolution());
}
// implemented by each camera to get the high byte of the camera ID word and test it against the word for this camera

bool Arducam5640::testHighDeviceIdByte()
{
  return getHighDeviceIdByte() == 0x56;
}
uint8_t Arducam5640::getHighDeviceIdByte()
{
   uint8_t regVal;
  rdSensorReg16_8(OV5640_CHIPID_HIGH, &regVal);
  return regVal;
}
// implemented by each camera to get the low byte of the camera ID word and test it against the word for this camera
 bool Arducam5640::testLowDeviceIdByte()
{
  return getLowDeviceIdByte() == 0x42;
}
uint8_t Arducam5640::getLowDeviceIdByte()
{
  uint8_t regVal;
  rdSensorReg16_8(OV5640_CHIPID_LOW, &regVal);
  return regVal;
}
 void Arducam5640::initializeSensor()
 {
   delayms(100);
   if (sm_isJPEG)
     {
       wrSensorReg16_8(0x3103, 0x11);
       wrSensorReg16_8(0x3008, 0x82);
       delayms(100);
       wrSensorRegs16_8(OV5640YUV_Sensor_Dvp_Init);
       delayms(500);
       wrSensorRegs16_8(OV5640_JPEG_QSXGA);
       wrSensorRegs16_8(OV5640_QSXGA2QVGA);
#if (defined(OV5640_MINI_5MP_PLUS) || (defined ARDUCAM_SHIELD_V2))
       wrSensorReg16_8(0x4407, 0x04);
#else
       wrSensorReg16_8(0x4407, 0x0C);
#endif

     }
   else
     {
          wrSensorReg16_8(0x3103, 0x11);
          wrSensorReg16_8(0x3008, 0x82);
          delayms(500);
          wrSensorRegs16_8(OV5640YUV_Sensor_Dvp_Init);
          wrSensorRegs16_8(OV5640_RGB_QVGA);
     }

#if LOG_INFO
     fprintf(stderr,"info:: completed program load\n");
#endif
}
void Arducam5640::setQuantization(uint8_t value)
{
     wrSensorReg16_8(0xff, 0x00);
     wrSensorReg16_8(0x44, value);
}
void Arducam5640::changeResolution(EResolution newResolution)
{
  setResolution(newResolution);
  wrSensorRegs16_8(m_resolutionProgram);
}
bool Arducam5640::setResolution(EResolution resolution) {
  const sensor_reg* resolutionProgram = NULL;
  switch (resolution) {
  case R160x120:
    break;
  case R176x144:
    break;
  case R320x240:
    resolutionProgram = OV5640_QSXGA2QVGA;
    break;
  case R352x288:
    resolutionProgram = OV5640_QSXGA2CIF;
    break;
  case R640x480:
    resolutionProgram = OV5640_QSXGA2VGA;
    break;
  case R800x600:
    break;
  case R1024x768:
    resolutionProgram = OV5640_QSXGA2XGA;
    break;
  case R1280x1024:
    break;
  case R1600x1200:
    resolutionProgram = OV5640_QSXGA2UXGA;
    break;
  case R1280x960:
    resolutionProgram = OV5640_QSXGA2SXGA;
    break;
  case R2048x1536:
    resolutionProgram = OV5640_QSXGA2QXGA;
    break;
  case R2592x1944:
    resolutionProgram = OV5640_JPEG_QSXGA;
    break;
  default:
    return false;
  }
  m_resolutionProgram = resolutionProgram;
  m_resolution = resolution;
  setQuantization(40);
  return true;
}
void Arducam5640::programSensor(const struct sensor_reg regList[])
{
  wrSensorRegs16_8(regList);
}
