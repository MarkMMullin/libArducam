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
#include "Arducam.h"
// the purpose of this class is to specialize operations that are specific to this
// camera model
class Arducam2640 : public Arducam
{
 public:
  Arducam2640(int cameraNumber,CameraBank* cameraBank,int SPIFD,int I2CFD);
  virtual void changeResolution(EResolution newResolution) ;
  virtual bool setResolution(EResolution resolution);
  
  // set image quantization (quality)
  virtual void setQuantization(uint8_t value);
  virtual void programSensor(const struct sensor_reg regList[]);
  virtual Arducam::ECameraType GetCameraType() const;
 protected:
  virtual  bool testHighDeviceIdByte();
  virtual uint8_t getHighDeviceIdByte();
  virtual bool testLowDeviceIdByte();
  virtual uint8_t getLowDeviceIdByte();
  virtual  void initializeSensor();
  static const int SENSOR_ADDRESS = 0x30;


};
