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
#include <string>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <sstream>
#include <vector>


#include "CameraConfig.h"
#include "Arducam.h"
#include "CameraManager.h"
using namespace std;

//********************************************************
//  Utility to split strings, needs a better home
//******************************************************
vector<string> &split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}

/**
		R160x120 = 1,
		R176x144 = 2,
		R320x240 = 3,
		R352x288 = 4,
		R640x480 = 5,
		R800x600 = 6,
		R1024x768 = 7,
		R1280x1024 = 8,
		R1600x1200 = 9
*/

void CameraConfig::Load(configuru::Config& cfg)
{
  
  int isJPEG = (int) cfg["isJPEG"];
  Arducam::sm_isJPEG = isJPEG != 0;
  int acquisitionMode = (int) cfg["acquisitionMode"];
  Arducam::sm_acquisitionMode = (Arducam::EImageAquisitionMode) acquisitionMode;
  int imageBufferSize = (int) cfg["imageBufferSizeInKb"];

  Arducam::sm_imageBufferSize = imageBufferSize * 1024;

  int busSpeed = (int) cfg["busSpeed"];
  CameraBank::sm_busSpeed = busSpeed;
  CameraBank::sm_bank0SPI = (uint8_t) ((int) cfg["bank0-spi"]);
  CameraBank::sm_bank0I2C = (uint8_t) ((int) cfg["bank0-i2c"]);
  CameraBank::sm_bank1SPI = (uint8_t) ((int) cfg["bank1-spi"]);
  CameraBank::sm_bank1I2C = (uint8_t) ((int) cfg["bank1-i2c"]);
  string gpioPins = (string) cfg["camSelectGPIO"];
  vector<string> pins = vector<string>();
  split(gpioPins,',',pins);
  for(unsigned int i = 0;i < pins.size();i++)
    CameraBank::sm_gpioPins[i] = atoi(pins.at(i).c_str());

  int maxBurstBlockSize = (int) cfg["maxBurstBlockSize"];
  Arducam::sm_maxBurstBlockSize = maxBurstBlockSize;

  int fifoReadAttempts = (int) cfg["maxFifoReadAttempts"];
  Arducam::sm_fifoReadAttempts = fifoReadAttempts;

  
  string resolution = (string) cfg["resolution"];
  Arducam::EResolution res = Arducam::parseResolution(resolution);
  Arducam::setDefaultResolution(res);
  Arducam::initializeBankImageBuffers();

  int cacheImageBufferSize = (int) cfg["cacheImageBufferSizeInKb"];
  cacheImageBufferSize *= 1024;
  Arducam::sm_imagebuffer_size = cacheImageBufferSize;
  CameraManager::sm_maxPasses = (int) cfg["maxpasses"];
  CameraManager::sm_timeoutmSecs = (int) cfg["cameraTimeoutmSec"];
  CameraManager::sm_imageDir = (string) cfg["imageDir"];
  CameraManager::sm_recordingOn = (int) cfg["recordingOn"] != 0;
  string ac = (string) cfg["cameramask"];
  int x = strtol(ac.c_str(), NULL, 16);
  CameraManager::sm_activeCameras = x;
}
