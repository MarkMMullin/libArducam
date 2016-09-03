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
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

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

#include "Arducam.h"
#include "CameraConfig.h"
#include "CameraManager.h"
using namespace std;

int main(int argc, char *argv[]) {
  configuru::Config cfg = configuru::parse_file("/home/mark/libArducam/cameraconfig.json", configuru::JSON);
  CameraConfig::Load(cfg);

  // start the CameraManager
  CameraManager* cm = CameraManager::GetSingleton();
  fprintf(stderr,"Running\n");
  cm->StartCapture();
}
