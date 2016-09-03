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
  along with Abaddon.  If not, see <http://www.gnu.org/licenses/>.

  This code is an extreme fork of original code at https://github.com/ArduCAM
  and all changes are Copyright 2016 Mark M. Mullin (mark.m.mullin@gmail.com)
*/
// standard headers
#include <string>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <linux/i2c-dev.h>
// Abaddon headers
#include "Arducam.h"
#include "CameraBank.h"
// Arducam::
uint16_t Arducam::sm_fifoReadAttempts = 2;
uint16_t Arducam::sm_maxBurstBlockSize = 1024;
uint32_t Arducam::sm_imageBufferSize = 2 * 1024; //@config[imageBufferSize] - number of kb to reserve for in memory images before write
uint8_t* Arducam::sm_imageBuffer[2] = {NULL,NULL};
bool Arducam::sm_isJPEG = true;
Arducam::EResolution Arducam::sm_resolution;
Arducam::EImageAquisitionMode Arducam::sm_acquisitionMode;
Arducam::Arducam(int cameraNumber,CameraBank* theBank,int SPIFD,int I2CFD) {
  m_cameraBank = theBank;
  m_SPIFD = SPIFD;
  m_I2CFD = I2CFD;

  m_cameraNo = cameraNumber;

  m_isInitialized = false;
  m_isCapturing = false;
  m_burstReadIndex = 0;
  m_burstReadBuffer = m_burstWriteBuffer = NULL;
  m_burstReadBuffer = new uint8_t[sm_maxBurstBlockSize + 16];
  m_burstWriteBuffer = new uint8_t[sm_maxBurstBlockSize + 16];
  m_acquisitionMode = sm_acquisitionMode;
}

Arducam::~Arducam() {
  // TODO Auto-generated destructor stub
}

bool Arducam::isCorrectSensor()
{
  return testHighDeviceIdByte() && testLowDeviceIdByte();
}

bool Arducam::powerUp() {
#if LOG_INFO
  fprintf(stderr,"info:start powerup camera %d\n",m_cameraNo);
#endif
  // if you can't reset it you can't use it
  if(!reset()) {
    fprintf(stderr,"warning:powerup reset failed on camera %d\n",m_cameraNo);
    return false;
  }
#if LOG_INFO
  fprintf(stderr,"info:powerup reset complete on camera %d\n",m_cameraNo);
#endif
  // if the SPI isn't working you can't use it
  if(!testSPIBus(5))
    {
      fprintf(stderr,"warning:powerup spi check failed on camera %d\n",m_cameraNo);
      return false;
    }
#if LOG_INFO
  fprintf(stderr,"info:powerup spi check complete on camera %d\n",m_cameraNo);
  uint8_t arduchipRev = read_reg(ARDUCHIP_REV);
  uint8_t vLow = arduchipRev & VER_LOW_MASK;
  uint8_t vHigh = (arduchipRev & VER_HIGH_MASK) >>6;
  fprintf(stderr,"info:device: arduchip rev %d.%d (%02x)\n",vHigh,vLow,arduchipRev);
#endif
  
  // Check if the camera device id is correct (as realized in concrete subclass) or you can't use it
  if(!isCorrectSensor()) {
    uint8_t hb = getHighDeviceIdByte();
    uint8_t lb = getLowDeviceIdByte();
    fprintf(stderr,"warning:powerup i2c sensor version check failed on camera %d with hi = %02x,low = %02x\n",m_cameraNo,hb,lb);
    //return false;
  }
#if LOG_INFO
  fprintf(stderr,"info:powerup i2c sensor version check complete on camera %d\n",m_cameraNo);
#endif
  // Change MCU mode
  write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
  // sensor initialization
  initializeSensor();
  sleep(1); // Let auto exposure do it's thing after changing image settings
  // can be marked as successfully initialized
  m_isInitialized = true;
  return true;
}
bool Arducam::reset()
{
  uint8_t rv = read_reg(ARDUCHIP_GPIO);
  if(rv == 0xFF)
    return false;

#if LOG_INFO
  fprintf(stderr,"info: GPIO = 0x%02x\n",rv);
#endif
#if DO_GPIO_RESET	  
  write_reg(ARDUCHIP_GPIO,0x0);
  delayms(500);
  write_reg(ARDUCHIP_GPIO,0x15);
  int ctr = 10;
  while(true) {
    rv = read_reg(ARDUCHIP_GPIO);
    if(rv == 0x15) {
      delayms(50);
      break;
    }
    write_reg(ARDUCHIP_GPIO,0x15);
    delayms(500);
    if(ctr-- <= 0) {
      return false;
    }
  }
#endif
  return true;
}
void Arducam::capture() {
  // Flush the FIFO
  flush_fifo();
  // Clear the capture done flag
  clear_fifo_flag();
  start_capture();
}

bool Arducam::serialRead(FILE *fp) {
  uint8_t temp, temp_last;
  uint32_t deposit = 0;

  uint32_t length=0;
  int attempts = sm_fifoReadAttempts;
  // test ordering is deliberate in order to allow fifo read attempts of zero to cause continous failure
  while(attempts-- > 0 && ((length = read_fifo_length()) == 0))
    pthread_yield();
  if(length == 0) {
    clear_fifo_flag();
    return false;
  }
#if LOG_INFO
  fprintf(stderr,"info: image size = %d\n",length);
#endif
  //uint8_t* buffer = sm_imageBuffer[imageBufferNumber()];
  uint8_t buffer[555555];
  temp = read_fifo();
  bool isImageStarted = false;
  // Read JPEG data from FIFO
  //while ((temp != 0xD9) | (temp_last != 0xFF)) {
  while (length-- > 0) {
    temp_last = temp;
    temp = read_fifo();
    if(!isImageStarted && (temp == 0xD8) & (temp_last == 0xFF))
      {
	buffer[deposit++] = temp_last;
	buffer[deposit++] = temp;
	isImageStarted = true;
      }
    else if(isImageStarted)
      {
	buffer[deposit++] = temp;
	// // Write image data to buffer if not full
	// if (deposit < sm_imageBufferSize) {
	//   buffer[deposit++] = temp;
	// } else {
	//   // Write BUF_SIZE uint8_ts image data to file
	//   fwrite(buffer, sm_imageBufferSize, 1, fp);
	//   deposit = 0;
	//   buffer[deposit++] = temp;
	// }
      }
  }
  // Write the remain uint8_ts in the buffer
  if (deposit > 0) {
    fwrite(buffer, deposit, 1, fp);
  }
  return true;
}
bool Arducam::tightSerialRead(FILE *fp) {
  uint8_t address = ARDUCHIP_READ_FIFO;
  uint8_t temp, temp_last;
  int i = 0;
  i = 0;

  uint8_t tx[2]={address,0x00};
  uint8_t rx[2]={0,0};
  struct spi_ioc_transfer tr;
  memset(&tr,0,sizeof(tr));
  tr.tx_buf = (unsigned long)tx;
  tr.rx_buf = (unsigned long)rx;
  tr.len = ARRAY_SIZE(tx);
  tr.delay_usecs = m_cameraBank->GetSPIDelay();
  tr.speed_hz = m_cameraBank->GetSPISpeed();
  tr.bits_per_word = m_cameraBank->GetSPIBits();

  ioctl(m_SPIFD, SPI_IOC_MESSAGE(1), &tr);
  temp = rx[1];
  temp_last = 0;
  m_bufferIndex = 0;
  // Write first image data to buffer
  sm_imageBuffer[imageBufferNumber()][i++] = temp;

  // Read JPEG data from FIFO
  while ((temp != 0xD9) | (temp_last != 0xFF)) {
    temp_last = temp;

    ioctl(m_SPIFD, SPI_IOC_MESSAGE(1), &tr);
    temp = rx[1];
    writeImageByte(temp,fp);
  }
  // Write the remain uint8_ts in the buffer
  if (i > 0) {
    fwrite(sm_imageBuffer[imageBufferNumber()], i, 1, fp);
  }
  return true;
}
//@config[maxBurstBlockSize] - max size of burst block transfer element - larger sizes are more efficient, but they starve the fifos for attention which
// causes them to loose their data

#define TRACEBULKOPERATIONS 0
uint8_t Arducam::burstReadByte(int remaining)
{
  // automatic reload every time read index passes end of buffer
  if(m_burstReadIndex >= sm_maxBurstBlockSize)
    {
      struct spi_ioc_transfer tr;
      uint16_t readSize = sm_maxBurstBlockSize <= remaining ? sm_maxBurstBlockSize : remaining;
      memset(&tr,0,sizeof(tr));
      tr.tx_buf = (unsigned long)m_burstWriteBuffer;
      tr.rx_buf = (unsigned long)m_burstReadBuffer;
      tr.len = readSize;
      tr.delay_usecs = 0;
      tr.speed_hz = m_cameraBank->GetSPISpeed();
      tr.bits_per_word = m_cameraBank->GetSPIBits();
      // 
      tr.cs_change = readSize != remaining ? 1 : 0;
      ioctl(m_SPIFD, SPI_IOC_MESSAGE(1), &tr);
      // re-initialize local read
      m_burstReadIndex = 0;
    }
  return m_burstReadBuffer[m_burstReadIndex++];
}


bool Arducam::burstRead(FILE *fp) {
  uint32_t length=0;
  int attempts = sm_fifoReadAttempts;
  // test ordering is deliberate in order to allow fifo read attempts of zero to cause continous failure
  while(attempts-- > 0 && ((length = read_fifo_length()) == 0))
    pthread_yield();
  if(length == 0) {
    clear_fifo_flag();
    return false;
  }
  m_fifoLength = length;
  // force a leading data capture
  m_burstReadIndex = Arducam::sm_maxBurstBlockSize;
#if TRACEBULKOPERATIONS
  printf("claimed image length = %d\n",length);
#endif
  uint8_t temp = 0,temp_last = 0;
  uint8_t tx[1]={BURST_FIFO_READ};
  uint8_t rx[1]={0};
  struct spi_ioc_transfer tr;
  memset(&tr,0,sizeof(tr));
  tr.tx_buf = (unsigned long)tx;
  tr.rx_buf = (unsigned long)rx;
  tr.len = ARRAY_SIZE(tx);
  tr.delay_usecs = 0;
  tr.speed_hz = m_cameraBank->GetSPISpeed();
  tr.bits_per_word = m_cameraBank->GetSPIBits();
  tr.cs_change = 1;
  // issue the burst fifo read command and hold chip select for the duration of the data fetch
  ioctl(m_SPIFD, SPI_IOC_MESSAGE(1), &tr);


  bool isImageData = false;

  m_bufferIndex = 0;

  while(length-- > 0)
    {
      if(isImageData)
	writeImageByte(burstReadByte(length),fp);
      else {
	temp_last = temp;

	temp = burstReadByte(length);
	if((temp == 0xD8) & (temp_last == 0xFF))
	{
	  // image has started, write the segment marker to the file
	  isImageData = true;
	  writeImageByte(temp_last,fp);
	  writeImageByte(temp,fp);
	}
      }
    }
  if (m_bufferIndex > 0) {
    fwrite(sm_imageBuffer[imageBufferNumber()], m_bufferIndex, 1, fp);
  }
  fflush(fp);
  return isImageData;
}


Arducam::EResolution Arducam::parseResolution(const std::string& resolution) {
  if (resolution.compare("R160x120") == 0)
    return R160x120;
  else if (resolution.compare("R176x144") == 0)
    return R176x144;
  else if (resolution.compare("R320x240") == 0)
    return R320x240;
  else if (resolution.compare("R352x288") == 0)
    return R352x288;
  else if (resolution.compare("R640x480") == 0)
    return R640x480;
  else if (resolution.compare("R800x600") == 0)
    return R800x600;
  else if (resolution.compare("R1024x768") == 0)
    return R1024x768;
  else if (resolution.compare("R1280x1024") == 0)
    return R1280x1024;
  else if (resolution.compare("R1600x1200") == 0)
    return R1600x1200;
  else
    return R320x240;
}


void Arducam::bus_write(uint8_t address, uint8_t value) const
{
  int ret;
  uint8_t tx[2]={address,value};
  uint8_t rx[2]={0,0};
  struct spi_ioc_transfer tr;
  memset(&tr,0,sizeof(tr));
  tr.tx_buf = (unsigned long)tx;
  tr.rx_buf = (unsigned long)rx;
  tr.len = ARRAY_SIZE(tx);
  tr.delay_usecs = m_cameraBank->GetSPIDelay();
  tr.speed_hz = m_cameraBank->GetSPISpeed();
  tr.bits_per_word = m_cameraBank->GetSPIBits();
  tr.pad = 0;
  ret = ioctl(m_SPIFD, SPI_IOC_MESSAGE(1), &tr);
  if (ret < 0)
    printf("can't send spi combined addr/value message (%d/%x)",errno,errno);
}

uint8_t Arducam::bus_write(uint8_t address)
{
  uint8_t tx[1]={address};
  uint8_t rx[1]={0};
  struct spi_ioc_transfer tr;
  memset(&tr,0,sizeof(tr));
  tr.tx_buf = (unsigned long)tx;
  tr.rx_buf = (unsigned long)rx;
  tr.len = ARRAY_SIZE(tx);
  tr.delay_usecs = m_cameraBank->GetSPIDelay();
  tr.speed_hz = m_cameraBank->GetSPISpeed();
  tr.bits_per_word = m_cameraBank->GetSPIBits();
  tr.cs_change = 1;
  tr.pad = 0;
  tr.cs_change = 1;
  int ret = ioctl(m_SPIFD, SPI_IOC_MESSAGE(1), &tr);
  if (ret < 0)
    printf("warn:can't send spi addr message (%d/%x)",errno,errno);
  return rx[0];
}

uint8_t Arducam::bus_read(uint8_t address) const
{
  int ret;
  uint8_t tx[2]={address,0x00};
  uint8_t rx[2]={0,0};
  struct spi_ioc_transfer tr;
  memset(&tr,0,sizeof(tr));
  tr.tx_buf = (unsigned long)tx;
  tr.rx_buf = (unsigned long)rx;
  tr.len = ARRAY_SIZE(tx);
  tr.delay_usecs = m_cameraBank->GetSPIDelay();
  tr.speed_hz = m_cameraBank->GetSPISpeed();
  tr.bits_per_word = m_cameraBank->GetSPIBits();
  tr.pad = 0;	
  ret = ioctl(m_SPIFD, SPI_IOC_MESSAGE(1), &tr);
  if (ret < 0)
    fprintf(stderr,"warn:can't read spi message(%d/%x/%c)",errno,errno,errno);
  return rx[1];
}

uint8_t Arducam::continue_bus_read()
{
  uint8_t tx[1]={0};
  uint8_t rx[1]={0};
  struct spi_ioc_transfer tr;
  memset(&tr,0,sizeof(tr));
  tr.tx_buf = (unsigned long)tx;
  tr.rx_buf = (unsigned long)rx;
  tr.len = ARRAY_SIZE(tx);
  tr.delay_usecs = m_cameraBank->GetSPIDelay();
  tr.speed_hz = m_cameraBank->GetSPISpeed();
  tr.bits_per_word = m_cameraBank->GetSPIBits();
  tr.cs_change = 0;
  // TODO - handle fault in ioctl operation
  ioctl(m_SPIFD, SPI_IOC_MESSAGE(1), &tr);
  return rx[0];
}



uint8_t Arducam::wrSensorReg8_16(uint8_t regID, uint16_t regDat)
{
  char wbuf[3]={regID,(uint8_t) ((regDat>>8) & 0x00ff),(uint8_t) (regDat & 0x00ff)}; //first byte is address to write. others are bytes to be written
  write(m_I2CFD, wbuf, 3);
  return 1;
}

uint8_t Arducam::rdSensorReg8_16(uint8_t regID, uint16_t* regDat)
{
  char read_start_buf[1] = {regID};
  char rbuf[2];
  write(m_I2CFD, read_start_buf, 1); //reposition file pointer to register 0x28
  read(m_I2CFD, rbuf, 2);
  *regDat =((rbuf[0]<<8) & rbuf[1]) & 0xffff;
  return 1;
}

uint8_t Arducam::wrSensorReg16_8(uint16_t regID, uint8_t regDat)
{
  uint8_t reg_H,reg_L;
  reg_H = (regID >> 8) & 0x00ff;
  reg_L = regID & 0x00ff;
  char wbuf[3]={reg_H,reg_L,regDat}; //first byte is address to write. others are bytes to be written
  write(m_I2CFD, wbuf, 3);
  return 1;
}

uint8_t Arducam::rdSensorReg16_8(uint16_t regID, uint8_t* regDat)
{
  uint8_t reg_H,reg_L;
  reg_H = (regID >> 8) & 0x00ff;
  reg_L = regID & 0x00ff;
  char read_start_buf[2] = {reg_H,reg_L};
  char rbuf[1];
  write(m_I2CFD, read_start_buf, 2); //reposition file pointer to register 0x28
  read(m_I2CFD, rbuf, 1);
  *regDat =rbuf[0];
  return 1;
}

int Arducam::wrSensorRegs8_8(const struct sensor_reg reglist[])
{
#if 1
  const struct sensor_reg *next = reglist;
  while((next->reg != 0xff) || (next->val != 0xff)) {
    wrSensorReg8_8(next->reg, next->val);
    next++;
  } 
  return 1;
#else
   uint16_t reg_addr = 0;
  uint16_t reg_val = 0;
  const struct sensor_reg *next = reglist;
  do {
    reg_addr = pgm_read_word(&next->reg);
    reg_val = pgm_read_word(&next->val);
    wrSensorReg8_8(reg_addr, reg_val);
    next++;
  } while((reg_addr != 0xff) || (reg_val != 0xff));
  return 1;
#endif
}


int Arducam::wrSensorRegs8_16(const struct sensor_reg reglist[])
{
  unsigned int reg_addr,reg_val;
  const struct sensor_reg *next = reglist;

  while ((reg_addr != 0xff) | (reg_val != 0xffff))
    {
      reg_addr = pgm_read_word(&next->reg);
      reg_val = pgm_read_word(&next->val);
      wrSensorReg8_16(reg_addr, reg_val);
      next++;
    }
  return 1;
}

int Arducam::wrSensorRegs16_8(const struct sensor_reg reglist[])
{
  unsigned int reg_addr,reg_val;
  const struct sensor_reg *next = reglist;

  while ((reg_addr != 0xffff) | (reg_val != 0xff))
    {
      reg_addr = pgm_read_word(&next->reg);
      reg_val = pgm_read_word(&next->val);
      wrSensorReg16_8(reg_addr, reg_val);
      next++;
    }

  return 1;
}


