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
#include "Arducam.h"
class CameraBank
{
  
  friend class CameraConfig;
 public:
  static bool InitializeBanks();
  static inline CameraBank* GetBank(int bankIndex) { return sm_banks[bankIndex]; }
  void Activate(Arducam* camera);
  inline int BankNumber() const { return m_bankNumber; }
  inline int GetSPIFD() { return m_SPIFD; }
  inline int GetI2CFD() { return m_I2CFD; }

  inline uint8_t GetSPIMode() { return m_SPIMode; }
  inline void SetSPIMode(uint8_t mode) { m_SPIMode = mode; }
  inline uint8_t GetSPIBits() { return m_SPIBits; }
  inline void SetSPIBits(uint8_t mode) { m_SPIBits = mode; }
  inline uint32_t GetSPISpeed() { return m_SPISpeed; }
   inline void SetSPISpeed(uint32_t mode) { m_SPISpeed = mode; }
  inline uint16_t GetSPIDelay() { return m_SPIDelay; }
   inline void SetSPIDelay(uint8_t mode) { m_SPIDelay = mode; }
   inline static Arducam::ECameraType GetCameraType(int bnk,int idx)  { return CameraBank::sm_cameraType[(bnk*4)+ idx]; }
   inline static void SetCameraType(int bnk,int idx,Arducam::ECameraType tp) { CameraBank::sm_cameraType[(bnk*4)+ idx] = tp; }
 private:
   CameraBank(int bankNumber);
   // open the SPI file stream for access
  void openSPI();
  // open the I2C file stream for access
  void openI2C();
  // utility function to set the value of a gpio output
  void gpioSet(int gpio, int value);
  // index of this camera bank
  int m_bankNumber;
  // The SPI bus number
  int m_SPIbusId;
  // The chip select line number
  int m_SPICSId;
  // SPI operating mode
   uint8_t m_SPIMode;
  // SPI bits per machine word
   uint8_t m_SPIBits;
  // SPI bus speed
   uint32_t m_SPISpeed;
  // SPI bus delay
   uint16_t m_SPIDelay;
  // I2C bus number
  int m_I2CbusId;
  // I2C  I bus file descriptor
  int m_I2CFD;
  //  SPI bus file descriptor
  int m_SPIFD;
  static Arducam::ECameraType sm_cameraType[8];
  static CameraBank* sm_banks[2];
  static uint8_t sm_gpioPins[4];
  static int sm_busSpeed;
  static uint8_t sm_bank0SPI;
  static uint8_t sm_bank0I2C;
  static uint8_t sm_bank1SPI;
  static uint8_t sm_bank1I2C;
  static uint8_t sm_common_device_address;
  static const int WORD_SIZE = 8;
};
