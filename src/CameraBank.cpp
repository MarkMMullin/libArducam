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
#include <string>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <sys/time.h>

#include "CameraBank.h"

#define LOG_GPIO_INFO 0
int CameraBank::sm_busSpeed = 0;
Arducam::ECameraType CameraBank::sm_cameraType[8];
CameraBank*CameraBank::sm_banks[2];
uint8_t CameraBank::sm_gpioPins[4] = {60,49,117,115};
uint8_t CameraBank::sm_bank0SPI = 0;
uint8_t CameraBank::sm_bank0I2C = 0;
uint8_t CameraBank::sm_bank1SPI = 0;
uint8_t CameraBank::sm_bank1I2C = 0;
uint8_t CameraBank::sm_common_device_address = 0;
bool CameraBank::InitializeBanks()
{
  for(int i = 0; i < 2;i++)
      CameraBank::sm_banks[i] = new CameraBank(i);
  return true;
}
// This should be called at outermost scope of camera access operation
void CameraBank::Activate(Arducam* camera)
{
  int bankIndex = camera->GetCameraBank()->BankNumber();
  int deviceIndex = camera->CameraNumber() % 4;

#if LOG_INFO
  fprintf(stderr,"info,camera,%d,activating,device,%d,bank,%d\n",
	  camera->CameraNumber(),deviceIndex,bankIndex);
#endif

  // set the gpio pins
  int pinval[4];
  pinval[0] = pinval[1] = pinval[2] = pinval[3] = -1;

  if(bankIndex == 1)
    {
      pinval[2] = (deviceIndex & 1) > 0 ? 1 : 0;
      pinval[3] = (deviceIndex & 2) > 0 ? 1 : 0;
    }
  else
    {
      pinval[0] = (deviceIndex & 1) > 0 ? 1 : 0;
      pinval[1] = (deviceIndex & 2) > 0 ? 1 : 0;
    }

  // now that everything is initialized, set the pin states
  for(int i = 0;i < 4;i++)
    if(pinval[i] != -1)
      gpioSet(sm_gpioPins[i],pinval[i]);

  
}
CameraBank::CameraBank(int bankNumber)
{
  m_bankNumber = bankNumber;
  m_SPICSId = 0;
  if(bankNumber == 1)
    {
      m_SPIbusId = CameraBank::sm_bank1SPI;
      m_I2CbusId = CameraBank::sm_bank1I2C;
    }
  else
    {
      m_SPIbusId = CameraBank::sm_bank0SPI;
      m_I2CbusId = CameraBank::sm_bank0I2C;
    }
  m_SPIBits = WORD_SIZE;
  m_SPIMode = 0;
  m_SPISpeed = sm_busSpeed;
  m_SPIDelay = 0;

  openSPI();
  openI2C();
	
}
void CameraBank::openI2C()
{
  char i2cName[32];
  sprintf(i2cName,"/dev/i2c-%d",m_I2CbusId);

  // initialize i2c:
  if((m_I2CFD = open(i2cName, O_RDWR)) < 0)
    {
      perror("Failed to open i2c device.\n");
      exit(1);
    }
 
  if(ioctl(m_I2CFD, I2C_SLAVE_FORCE,sm_common_device_address) < 0) // original value = 0x30
   {
     printf("Failed to access i2c bus.\n");
     exit(1);
   }

}

void CameraBank::openSPI()
{
  char spiName[32];
  int ret;
  sprintf(spiName,"/dev/spidev%d.%d",m_SPIbusId,m_SPICSId);
  m_SPIFD = open(spiName, O_RDWR);
  if (m_SPIFD < 0)
    throw("can't open device");
  ret = ioctl(m_SPIFD, SPI_IOC_WR_MODE, &m_SPIMode);
  if (ret == -1)
    throw("can't set spi mode");
  ret = ioctl(m_SPIFD, SPI_IOC_RD_MODE, &m_SPIMode);
  if (ret == -1)
    throw("can't get spi mode");
  ret = ioctl(m_SPIFD, SPI_IOC_WR_BITS_PER_WORD, &m_SPIBits);
  if (ret == -1)
    throw("can't set bits per word");
  ret = ioctl(m_SPIFD, SPI_IOC_RD_BITS_PER_WORD, &m_SPIBits);
  if (ret == -1)
    throw("can't get bits per word");
  ret = ioctl(m_SPIFD, SPI_IOC_WR_MAX_SPEED_HZ, &m_SPISpeed);
  if (ret == -1)
    throw("can't set max speed hz");
  ret = ioctl(m_SPIFD, SPI_IOC_RD_MAX_SPEED_HZ, &m_SPISpeed);
  if (ret == -1)
    throw("can't get max speed hz");

}
// Simple gpio write - file lifetime contained within this function
void CameraBank::gpioSet(int gpio, int value)
{
  char buf[32];
  int fd;
  sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio);
  fd = open(buf, O_WRONLY);
  sprintf(buf, "%d", value);
  write(fd, buf, 1);
  close(fd);
#if LOG_GPIO_INFO
  fprintf(stderr,"info,GPIO,pin, %d,value,%d\n",gpio,value);
#endif
}
