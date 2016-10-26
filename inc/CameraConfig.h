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
#ifndef CameraConfig_h
#define CameraConfig_h
#include "configuru.hpp"
#include "CameraBank.h"
 class CameraConfig {
 public:
   static const char CC_OV2640 = 'A';
    static const char CC_OV5642 = 'B';
    static const char CC_OV5640 = 'C';
    static const char CC_NONE = 'X';
    static const char CC_BANKEND = ':';
   
  static void Load(configuru::Config& cfg);
 private:
  static void processCameraConfig(const std::string& cfg);
 };
#endif
