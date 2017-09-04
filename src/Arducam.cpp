/*
  This file is part of libArduino.

  libArduino is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  libArduino is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with libArduino.  If not, see <http://www.gnu.org/licenses/>.

  This code is an extreme fork of original code at https://github.com/ArduCAM
  and all changes are Copyright 2016 Mark M. Mullin (mark.m.mullin@gmail.com)
*/
// standard headers
#include <string>
#include <vector>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <linux/i2c-dev.h>

#include "MMMDEFINES.h"
#include "Arducam.h"
#include "CameraBank.h"
#include "CameraManager.h"

#define DO_GPIO_RESET 0
#define LOG_INFO 1
// Arducam::
uint16_t Arducam::sm_fifoReadAttempts = 2;
uint16_t Arducam::sm_maxBurstBlockSize = 1024;
uint32_t Arducam::sm_imageBufferSize = 2 * 1024; //@config[imageBufferSize] - number of kb to reserve for in memory images before write
uint8_t* Arducam::sm_imageBuffer[2] = {NULL,NULL};
bool Arducam::sm_isJPEG = true;
Arducam::EResolution Arducam::sm_resolution;
int Arducam::sm_quantization;

Arducam::EImageAquisitionMode Arducam::sm_acquisitionMode;
uint32_t Arducam::sm_imagebuffer_size;
Arducam::Arducam(int cameraNumber,CameraBank* theBank,int SPIFD,int I2CFD) {
  m_cameraBank = theBank;
  m_SPIFD = SPIFD;
  m_I2CFD = I2CFD;

  m_cameraNo = cameraNumber;
  m_shotCounter = 0;

  m_isInitialized = false;
  m_isCapturing = false;
  m_burstReadIndex = 0;
  m_burstReadBuffer = m_burstWriteBuffer = NULL;
  m_burstReadBuffer = new uint8_t[sm_maxBurstBlockSize + 16];
  m_burstWriteBuffer = new uint8_t[sm_maxBurstBlockSize + 16];
  m_acquisitionMode = sm_acquisitionMode;
  m_imageBuffer = new uint8_t[sm_imagebuffer_size];
  m_imageBufferDeposit = 0;
  m_lastImageBuffer = new uint8_t[sm_imagebuffer_size];
  m_lastImageBufferSize = 0;
  pthread_mutex_init(&m_lastImageBufferMutex, NULL);
  pthread_cond_init(&m_imageUpdateConditionVariable,NULL);
}

Arducam::~Arducam() {
  SAFE_DELETE(m_imageBuffer);
  SAFE_DELETE(m_lastImageBuffer);
}

bool Arducam::isCorrectSensor()
{
  return testHighDeviceIdByte() && testLowDeviceIdByte();
}

bool Arducam::powerUp() {
#if LOG_INFO
  fprintf(stderr,"info,camera,%d,start powerup\n",m_cameraNo);
#endif
  // if you can't reset it you can't use it
  if(!reset()) {
    fprintf(stderr,"warning,camera,%d,powerup reset failed\n",m_cameraNo);
    return false;
  }
#if LOG_INFO
  fprintf(stderr,"info,camera,%d,powerup reset complete\n",m_cameraNo);
#endif
  // if the SPI isn't working you can't use it
  if(!testSPIBus(5))
    {
      fprintf(stderr,"warning,camera,%dpowerup spi check failed\n",m_cameraNo);
      return false;
    }
  
  // Change MCU mode
  write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
#if LOG_INFO
  fprintf(stderr,"info,camera,%d,powerup spi check complete\n",m_cameraNo);
  uint8_t arduchipRev = read_reg(ARDUCHIP_REV);
  uint8_t vLow = arduchipRev & VER_LOW_MASK;
  uint8_t vHigh = (arduchipRev & VER_HIGH_MASK) >>6;
  fprintf(stderr,"info,device, arduchip, rev %d.%d (%02x)\n",vHigh,vLow,arduchipRev);
#endif
  
  // Check if the camera device id is correct (as realized in concrete subclass) or you can't use it
  if(!isCorrectSensor()) {
    uint8_t hb = getHighDeviceIdByte();
    uint8_t lb = getLowDeviceIdByte();
    fprintf(stderr,"warning,:camera,%d,overlooked,powerup i2c sensor version check failed  with hi = %02x,low = %02x\n",m_cameraNo,hb,lb);
    //return false;
  }
#if LOG_INFO
  fprintf(stderr,"info,camera,%d,powerup i2c sensor version check complete\n",m_cameraNo);
#endif
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
  int ctr = 10000;
  if(CameraNumber() != 47)
  while(rv == 0xFF && --ctr > 0)
    {
      rv = read_reg(ARDUCHIP_GPIO);
    }
  if(ctr == 0)
    {
      fprintf(stderr,"info, GPIO,value 0xFF, assumed unreadable\n");
      return false;
    }
#if LOG_INFO
  fprintf(stderr,"info, GPIO,read 0x%02x\n",rv);
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
    delayms(100);
    if(ctr-- <= 0) {
      return false;
    }
  }
#endif
  return true;
}
void Arducam::capture() {
  // Flush the FIFO
  //flush_fifo();
  flush_fifo();
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
    flush_fifo();
    return false;
  }
#if LOG_INFO
  fprintf(stderr,"info,image, size,%d\n",length);
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
  SAFE_DELETE(m_imageBuffer);
  // Write the remain uint8_ts in the buffer
  if (deposit > 0) {
    fwrite(buffer, deposit, 1, fp);
    m_imageBuffer = new uint8_t[deposit];
    memcpy(m_imageBuffer,buffer,deposit);
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
  flushImageBuffer(fp);
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
  bool isUnknownLength = false;
  uint32_t length=0;
  int attempts = sm_fifoReadAttempts;
  // test ordering is deliberate in order to allow fifo read attempts of zero to cause continous failure
  while(attempts-- > 0 && ((length = read_fifo_length()) == 0))
    delayms(100);

  if(length == 0) {
#if LOG_INFO
    fprintf(stderr,"warn,camera, %d, returned 0 length image\n",m_cameraNo);
#endif
    return false;
    //length = 99999999;
    //isUnknownLength = true;
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
      
	temp_last = temp;

	temp = burstReadByte(length);
	
	if(isImageData) {
	  writeImageByte(temp,fp);
	  if(isUnknownLength && temp == 0xD9 && temp_last == 0xFF)
	    length = 0;
      } else {
	if((temp == 0xD8) & (temp_last == 0xFF))
	  {
	    // image has started, write the segment marker to the file
	    isImageData = true;
	    writeImageByte(temp_last,fp);
	    writeImageByte(temp,fp);
	  }
      }
    }
  flushImageBuffer(fp);
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
    fprintf(stderr,"warn,SPI,can't send spi combined addr/value message (%d/%x)",errno,errno);
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
    fprintf(stderr,"warn,SPI,can't send spi addr message (%d/%x)",errno,errno);
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
    fprintf(stderr,"warn,SPI,can't read spi message(%d/%x/%c)",errno,errno,errno);
  // if(GetCameraType()  == ECameraType::OV5642)
  //   rx[1] = (uint8_t)(rx[1] >> 1) | (rx[1] << 7);
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
  reg_H = (regID >> 8) & 0xff;
  reg_L = regID & 0xff;
  uint8_t wbuf[3]={reg_H,reg_L,regDat}; 
  write(m_I2CFD, wbuf, 3);
  return 1;
}

uint8_t Arducam::rdSensorReg16_8(uint16_t regID, uint8_t* regDat)
{
  uint8_t reg_H,reg_L;
  reg_H = (regID >> 8) & 0xff;
  reg_L = regID & 0xff;
  uint8_t read_start_buf[2] = {reg_H,reg_L};
  uint8_t rbuf[2];
  write(m_I2CFD, read_start_buf, 2); 
  read(m_I2CFD, rbuf, 1);
  *regDat =rbuf[0];
  //fprintf(stderr,"%04x WHERE MSB=%02x,LSB=%02x = %02x\n",regID,reg_H,reg_L,rbuf[0]);
  return 1;
}

uint8_t Arducam::wrSensorReg8_8(uint8_t regID, uint8_t regDat)
{
  uint8_t wbuf[2]={regID,regDat}; //first byte is address to write. others are bytes to be written
  write(m_I2CFD, wbuf, 2);
  //fprintf(stderr,"WRITE %02x to %02x\n",regDat,regID);
  if(regID == 0xFF)
    delayms(200);
  return 1;
}

int Arducam::wrSensorRegs8_8(const struct sensor_reg reglist[])
{
  int baseIndex = 0;
  int testIndex = 1;
  bool isEOF = false;
  while(!isEOF)
    {
      isEOF = reglist[testIndex].reg == 0xFF && reglist[testIndex].val == 0xFF;
      bool isMatch = reglist[testIndex].reg == reglist[baseIndex].reg;
      isMatch = false;
      if(!isMatch || isEOF)
	{
	  if(baseIndex + 1 == testIndex)
	    {
	      uint8_t reg = (uint8_t) (reglist[baseIndex].reg & 0xFF);
	      uint8_t val =  (uint8_t) (reglist[baseIndex].val & 0xFF);
	      uint8_t wbuf[2]={reg,val}; //first byte is address to write. others are bytes to be written
	      write(m_I2CFD, wbuf, 2);
	      //fprintf(stderr,"0x%02x,0x%02x\n",reg , val);
	      if(reg == 0xFF)
		delayms(200);
	    }
	  else
	    {
	      //fprintf(stderr,"nope");
	      std::vector<uint8_t> regseq;
	      uint16_t numWriteBytes = 1 + (testIndex - baseIndex);
	      regseq.reserve(numWriteBytes);
	      regseq.push_back(reglist[baseIndex].reg);
	      for(int ri = baseIndex;ri < testIndex;ri++)
		regseq.push_back(reglist[ri].val);
	      uint8_t* regwritedata = &regseq[0];
	      write(m_I2CFD, regwritedata,numWriteBytes );
	      //for(int i = 0;i < numWriteBytes;i++)
	      //	fprintf(stderr,"0x%02x,0x%02x,\tseq = %d\n",reglist[baseIndex].reg,regseq[i],i);
	      delayms(200);
			
	    }
	  baseIndex = testIndex;
	}
     
      testIndex++;
    }

  return 1;
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
  const struct sensor_reg *next = reglist;
  // excess parens per request of the compiler
  while ((next->reg != 0xffff) && (next->val != 0xff))
    {
      //fprintf(stderr,"WRITE:  %04x = %02x\n",next->reg,next->val);
      wrSensorReg16_8(next->reg, next->val);
      next++;
    }

  return 1;
}

void Arducam::writeInterimImageBytes(uint8_t writeByte,FILE* fp)
{	

  // Write BUF_SIZE uint8_ts image data to file
  if(CameraManager::sm_recordingOn)
    fwrite(sm_imageBuffer[imageBufferNumber()], sm_imageBufferSize, 1, fp);
  if(m_imageBufferDeposit + m_bufferIndex < sm_imagebuffer_size)
    {
      memcpy(&m_imageBuffer[m_imageBufferDeposit],sm_imageBuffer[imageBufferNumber()],sm_imageBufferSize);
      m_imageBufferDeposit += sm_imageBufferSize;
    }
	m_bufferIndex = 0;
	sm_imageBuffer[imageBufferNumber()][m_bufferIndex++] = writeByte;
}
  void Arducam::flushImageBuffer(FILE* fp)
  {
    if(m_bufferIndex > 0)
      {
	if(CameraManager::sm_recordingOn)
	  fwrite(sm_imageBuffer[imageBufferNumber()], m_bufferIndex, 1, fp);
	if(m_imageBufferDeposit + m_bufferIndex < sm_imagebuffer_size)
	  {
	    memcpy(&m_imageBuffer[m_imageBufferDeposit],sm_imageBuffer[imageBufferNumber()],m_bufferIndex);
	    m_imageBufferDeposit += m_bufferIndex;
	  }
	m_bufferIndex = 0;
      }
    // don't bother updating if it would overflow the buffer, this is just advisory information anyway
    if(m_imageBufferDeposit <= sm_imagebuffer_size)
      {
	pthread_mutex_lock (&m_lastImageBufferMutex);
	memcpy(m_lastImageBuffer,m_imageBuffer,m_imageBufferDeposit);
	m_lastImageBufferSize = m_imageBufferDeposit;
	pthread_mutex_unlock (&m_lastImageBufferMutex);
#if LOG_INFO
	fprintf(stderr,"info,Camera,%d,copy buffer store size, %d\n",m_cameraNo,m_lastImageBufferSize);
#endif
	// let everyone whos waiting know to grab the new image
	pthread_cond_broadcast(&m_imageUpdateConditionVariable);
      }
    m_imageBufferDeposit = 0;
  }

  uint8_t* Arducam::getLastImageBuffer(uint8_t* copyBuffer,uint32_t& bufferSize)
  {
    pthread_mutex_lock (&m_lastImageBufferMutex);
    bufferSize = m_lastImageBufferSize;
    memcpy(copyBuffer,m_lastImageBuffer,bufferSize);
    pthread_mutex_unlock (&m_lastImageBufferMutex);
#if LOG_INFO
    fprintf(stderr,"info,camera,%d,copy buffer FETCH size,%d\n",m_cameraNo,m_lastImageBufferSize);
#endif
    return copyBuffer;
  }

short Arducam::isCapturing()
{
    if(!m_isCapturing) return false;
    uint8_t rdval = read_reg(ARDUCHIP_TRIG);
    //fprintf(stderr,"trigger = %02x\n",rdval);
    if((rdval & 0xF0) != 0 || rdval == 0)
      {
	m_isCapturing = false;
	return -1;
      }
    m_isCapturing = !(rdval & CAP_DONE_MASK);
    return m_isCapturing ? 0 : 1;  
}
