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
#ifndef ARDUCAM_H_
#define ARDUCAM_H_

#define LOG_INFO 1

// standard headers
#include <string>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>

#include "SensorRegister.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define pgm_read_byte(x)        (*((uint8_t *)x))
//  #define pgm_read_word(x)        (*((short *)(x & 0xfffffffe)))
#define pgm_read_word(x)        ( ((*((uint8_t *)x + 1)) << 8) + (*((uint8_t *)x)))
#define pgm_read_byte_near(x)   (*((uint8_t *)x))
#define pgm_read_byte_far(x)    (*((uint8_t *)x))
//  #define pgm_read_word_near(x)   (*((short *)(x & 0xfffffffe))
//  #define pgm_read_word_far(x)    (*((short *)(x & 0xfffffffe)))
#define pgm_read_word_near(x)   ( ((*((uint8_t *)x + 1)) << 8) + (*((uint8_t *)x)))
#define pgm_read_word_far(x)    ( ((*((uint8_t *)x + 1)) << 8) + (*((uint8_t *)x))))


class CameraBank;



class Arducam {
  friend class CameraConfig;
 public:
  enum ECameraType {
    NONE = 0,
    OV2640 = 1,
    OV5642 = 2,
    OV5640 = 3
  };
  enum EImageAquisitionMode {
    Serial=0, TightSerial=1, Burst=2
  };
  enum EResolution {
    R160x120 = 1,
    R176x144 = 2,
    R320x240 = 3,
    R352x288 = 4,
    R640x480 = 5,
    R800x600 = 6,
    R1024x768 = 7,
    R1280x1024 = 8,
    R1600x1200 = 9,
    R1280x960 = 10,
    R2048x1536 = 11,
    R2592x1944 = 12
  };
  /****************************************************/
  /* Sensor related definition                                                                                            */
  /****************************************************/

  Arducam(int cameraNumber,CameraBank* cameraBank,int SPIFD,int I2CFD);
  // run powerup checks and sequencing for SPI bus, and check version of sensor over I2C bus
  bool powerUp();
  // sensor specific initialization realized in subclass
  virtual void initializeSensor() = 0;
  // change the camera resolution WITHOUT reprogramming the sensor
  virtual void changeResolution(EResolution newResolution) = 0;
  // load programming into the sensor registers
  virtual void programSensor(const struct sensor_reg regList[]) = 0;
  // set image quantization (quality)
  virtual void setQuantization(uint8_t value) = 0;
  // sensor type info
  virtual ECameraType GetCameraType() const = 0;
  // true if the sensors reported type matches the class type (it better!)
  bool isCorrectSensor();
  virtual ~Arducam();
  // return the global index for this camera
  inline int CameraNumber() const { return m_cameraNo; }
  // return the camerabank instamnce
  inline CameraBank* GetCameraBank() {return m_cameraBank; }
  inline void waitOnVSync() { while(read_reg(Arducam::ARDUCHIP_TRIG) & 1);}
  
  // make an image capture request
  void capture();
  // true as long as image is actively being captured
  // external agents must increment the shot counter because the cameras opinion doesn't matter
  void incrementShotCounter() { m_shotCounter++;}
  uint32_t getShotCounter() const { return m_shotCounter;}
  short isCapturing();
  // record the name of the last saved file from this camera for monitoring purposes
  inline void setLastSave(const char* fname)
  {
    m_lastFilename = fname;
  }
  inline std::string getLastSave() {
    return m_lastFilename;
  }
        
  // transfer contents of image capture buffer to file
  inline bool transferImageBufferToFile(FILE* fp) {
    bool result;
		
    switch (m_acquisitionMode) {
    case Serial:
      result = serialRead(fp);
      break;
    case TightSerial:
      result = tightSerialRead(fp);
      break;
    case Burst:
      result = burstRead(fp);
      break;
    }
    flush_fifo();
    return result;
  }
  // return the last image the camera obtained - used to ameliorate races
  uint8_t* getLastImageBuffer(uint8_t* copyBuffer,uint32_t& bufferSize);
  //  current camera resolution code
  EResolution getResolution() const { return m_resolution; }
  
  
  //todo defaults  need to be per sensor type
  // set default global resolution for camera initialization
  static void setDefaultResolution(EResolution resolution) {sm_resolution = resolution; }
  // get default global resolution for camera initialization
  static const EResolution getDefaultResolution() { return sm_resolution; }
  // global default resolution constant
  static EResolution sm_resolution;
  // set the global quantization default
  static void setDefaultQuantization(int quantization) { sm_quantization = quantization;}
  // get the global quantization default
  static const int getDefaultQuantization() {return sm_quantization;}
  // global default quantization constant
  static int sm_quantization;
  
  // set the resolution of this camera and invoke any necessary sensor program changes
  virtual bool setResolution(EResolution resolution) = 0;
  // parse a resolution string and return the resolution
  static EResolution parseResolution(const std::string& resolution);
  // true iff the camera has successfully completed an initialization sequence
  inline bool isInitialized() const {
    return m_isInitialized;
  }
  // gets the sensor address associated with this camera - technically constant at the individual
  // sensor level but easier this way
  inline uint8_t getSensorAddress() {
    return m_sensor_addr;
  }
	
  static inline uint32_t getImageBufferSize() { return sm_imagebuffer_size;}
  // utility millisecond delay
  inline void delayms(uint32_t delay) const {
    usleep(1000 * delay);
  }
  inline void delayus(uint32_t delay) const {
    usleep( delay);
  }
  
  inline pthread_mutex_t* getLastImageBufferMutex()  { return &m_lastImageBufferMutex; }
  inline pthread_cond_t* getImageUpdateConditionVariable()  { return &m_imageUpdateConditionVariable; }
 protected:
  // reset the camera
  bool reset();
  // fetch image using byte by byte serial read (very slow, but easy to follow)
  bool serialRead(FILE *fp);
  // fetch image using optimized serial read (still slow)
  bool tightSerialRead(FILE *fp);
  // fetch image using burst read (fastest, but hardest to follow)
  bool burstRead(FILE *fp);
  // handles buffered reading of individual bytes
  uint8_t burstReadByte(int remaining);

  uint8_t wrSensorReg8_8(uint8_t regID, uint8_t regDat);

  inline uint8_t rdSensorReg8_8(uint8_t regID, uint8_t* regDat) {
    char read_start_buf[1] = {regID};
    char rbuf[1];
    write(m_I2CFD, read_start_buf, 1); //reposition file pointer to register 0x28
    read(m_I2CFD, rbuf, 1);
    *regDat = rbuf[0];
    return 1;
  }
  uint8_t wrSensorReg8_16(uint8_t regID, uint16_t regDat);
  uint8_t rdSensorReg8_16(uint8_t regID, uint16_t* regDat);
  uint8_t wrSensorReg16_8(uint16_t regID, uint8_t regDat);
  uint8_t rdSensorReg16_8(uint16_t regID, uint8_t* regDat);
  int wrSensorRegs8_8(const struct sensor_reg reglist[]);
  int wrSensorRegs8_16(const struct sensor_reg reglist[]);
  int wrSensorRegs16_8(const struct sensor_reg reglist[]);

  inline uint8_t read_reg(uint8_t addr) const
  {
    return bus_read(addr & 0x7F);

  }

  inline void write_reg(uint8_t addr, uint8_t data) const
  {
    bus_write(addr | 0x80, data);
  }
  // write command only, no data - e.g. burst read command
  uint8_t bus_write(uint8_t address);
  void bus_write(uint8_t address, uint8_t value) const;
  uint8_t bus_read(uint8_t address) const;
  uint8_t continue_bus_read();


  inline uint32_t read_fifo_length() {
    uint32_t len1,len2,len3,length=0;
    len1 = read_reg(FIFO_SIZE1);
    len2 = read_reg(FIFO_SIZE2);
    len3 = read_reg(FIFO_SIZE3) & 0x07;
    length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
    return length;
  }
  inline uint8_t read_fifo(void) {
    return bus_read(ARDUCHIP_READ_FIFO);
  }
  inline void flush_fifo(void) {
    write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK | FIFO_RDPTR_RST_MASK | FIFO_WRPTR_RST_MASK );
  }

  inline void start_capture(void) {
    write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
    m_isCapturing = true;
  }


  inline void writeImageByte(uint8_t writeByte,FILE* fp)
  {	
    // Write image data to buffer if not full
    if (m_bufferIndex < sm_imageBufferSize) {
      sm_imageBuffer[imageBufferNumber()][m_bufferIndex++] = writeByte;
    } else
      writeInterimImageBytes(writeByte,fp);
  }
  void writeInterimImageBytes(uint8_t writeByte,FILE* fp);
  void flushImageBuffer(FILE* fp);

  inline int imageBufferNumber() { return  m_cameraNo > 3 ? 1 : 0; }
  // return true if the SPI bus is operational (0 tries forces fail)
  inline bool testSPIBus(int tries) const {
	  
    while(tries-- > 0) {
      write_reg(ARDUCHIP_TEST1, 0x55);
      uint8_t temp = read_reg(ARDUCHIP_TEST1);
      if(temp == 0x55)
	return true;
      delayms(100);
    }
    return false;
  }

  // implemented by each camera to get the high byte of the camera ID word and test it against the word for this camera
  virtual bool testHighDeviceIdByte() = 0;
  // get the high device id byte
  virtual uint8_t getHighDeviceIdByte() = 0;
  // implemented by each camera to get the low byte of the camera ID word and test it against the word for this camera
  virtual bool testLowDeviceIdByte() = 0;
  virtual uint8_t getLowDeviceIdByte() = 0;
  // initialize the bank image buffers
  static inline void initializeBankImageBuffers() {
    sm_imageBuffer[0] = new uint8_t[sm_imageBufferSize];	  
    sm_imageBuffer[1] = new uint8_t[sm_imageBufferSize];
  }

  // a value between 0 and 3 ( 4 cameras per bank)
  int m_cameraNo;
  // reference to the camera bank this camera is a member of
  CameraBank* m_cameraBank;
  //todo - what to do about 16 bit sensors
  // initialized with sensor address by derivative class
  uint8_t m_sensor_addr;
  // defines how data is downloaded from Arducam
  EImageAquisitionMode m_acquisitionMode;
  // true iff the camera has successfully completed an initialization sequence
  bool m_isInitialized;
  // counts the shots accepted from this camera
  uint32_t m_shotCounter;
  // true while an active capture operation is ongoing
  bool m_isCapturing;
  // length of inbound FIFO from Arducam
  uint32_t m_fifoLength;
  // current byte read offset in burst read buffer
  uint32_t m_burstReadIndex;
  // current write index in image buffer
  uint32_t m_bufferIndex;
  // buffer for incoming bytes from camera during burst read
  uint8_t* m_burstReadBuffer;
  // buffer for outgoing bytes to camera during burst read
  uint8_t* m_burstWriteBuffer;
  // I2C  I bus file descriptor
  int m_I2CFD;
  //  SPI bus file descriptor
  int m_SPIFD;
  // camera resolution
  EResolution m_resolution;
  // name of the last written image file
  std::string m_lastFilename;
  // buffer of the last written image
  uint8_t* m_imageBuffer;
  uint32_t m_imageBufferDeposit;
  uint8_t* m_lastImageBuffer;
  uint32_t m_lastImageBufferSize;
  // sensor program realizing camera resolution
  const sensor_reg* m_resolutionProgram;
  // gate for updating the lastImageBuffer data
  pthread_mutex_t m_lastImageBufferMutex;
  // condition variable for signalling an update to the image
  pthread_cond_t m_imageUpdateConditionVariable;
  void updateImageBuffer();
  static uint8_t* sm_imageBuffer[2];
  static uint32_t sm_imageBufferSize;
  static uint16_t sm_maxBurstBlockSize;
  static uint16_t sm_fifoReadAttempts;
  static bool sm_isJPEG;
  static EImageAquisitionMode sm_acquisitionMode;
  static uint32_t sm_imagebuffer_size;


  // instructions
  static const int ARDUCHIP_READ_FIFO  = 0x3D;
  static const int ARDUCHIP_TEST1 = 0x00;  //TEST register
  static const int ARDUCHIP_TEST2 = 0x01;  //TEST register

  static const int ARDUCHIP_FRAMES = 0x01;  //Bit[2:0]Number of frames to be captured

  static const int ARDUCHIP_MODE = 0x02;  //Mode register
  static const int MCU2LCD_MODE = 0x00;
  static const int CAM2LCD_MODE = 0x01;
  static const int LCD2MCU_MODE = 0x02;

  static const int ARDUCHIP_TIM = 0x03;  //Timming control
  static const int HREF_LEVEL_MASK = 0x01; //0 = High active , 		1 = Low active
  static const int VSYNC_LEVEL_MASK = 0x02; //0 = High active , 		1 = Low active
  static const int LCD_BKEN_MASK = 0x04;  //0 = Enable, 			1 = Disable
  static const int DELAY_MASK = 0x08; //0 = no delay, 			1 = delay one clock
  static const int MODE_MASK = 0x10; //0 = LCD mode, 			1 = FIFO mode
  static const int FIFO_PWRDN_MASK = 0x20; //0 = Normal operation, 	1 = FIFO power down

  static const int ARDUCHIP_FIFO = 0x04;  //FIFO and I2C control
  static const int FIFO_CLEAR_MASK = 0x01;
  static const int FIFO_START_MASK = 0x02;
  static const int FIFO_RDPTR_RST_MASK = 0x10;
  static const int FIFO_WRPTR_RST_MASK = 0x20;

  static const int ARDUCHIP_GPIO = 0x06;  //GPIO Write Register
  static const int GPIO_RESET_MASK = 0x01;  //0 = default state,		1 =  Sensor reset IO value
  static const int GPIO_PWDN_MASK	= 0x02;  //0 = Sensor power down IO value, 1 = Sensor power enable IO value

  static const int BURST_FIFO_READ = 0x3C;  //Burst FIFO read operation
  static const int SINGLE_FIFO_READ = 0x3D;  //Single FIFO read operation

  static const int ARDUCHIP_REV = 0x40;  //ArduCHIP revision
  static const int VER_LOW_MASK = 0x3F;
  static const int VER_HIGH_MASK = 0xC0;

  static const int ARDUCHIP_TRIG = 0x41;  //Trigger source
  static const int VSYNC_MASK = 0x01;
  static const int SHUTTER_MASK = 0x02;
  static const int CAP_DONE_MASK = 0x08;

  static const int FIFO_SIZE1 = 0x42;  //Camera write FIFO size[7:0] for burst to read
  static const int FIFO_SIZE2 = 0x43;  //Camera write FIFO size[15:8]
  static const int FIFO_SIZE3 = 0x44;  //Camera write FIFO size[18:16]

	
};

#endif

