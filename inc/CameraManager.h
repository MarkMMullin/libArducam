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
#ifndef CameraManager_h
#define CameraManager_h
#include "Arducam.h"
#include <thread>
#include <string>
class CameraManager {
  friend class Arducam;
  friend class CameraConfig;
  friend void bankImageCaptureDriver(int highBankFlag);
 public:
  static CameraManager* GetSingleton();
  static inline std::string GetVersion() {
    char vb[32];
    sprintf(vb,"%s-%s:%s:%s.%s",BUILDTYPE,MAJORVER,MINORVER,PATCHVER,BUILDVER);
    return std::string(vb);
  }
  static inline std::string GetImageDirectory() { return sm_imageDir; }
  inline Arducam* getCamera(int cameraIndex) { return m_cameras[cameraIndex]; }

  void StartCapture();
  inline void StopCapture() { m_kill = true; }

  inline void getPerformanceStats(int cameraId,int& completed,int& attempted,int& timeouts,double& avgTime,double& completionRatio) const
        {
          double sr = (((double) m_reads[cameraId] - m_fails[cameraId]) / (double) m_reads[cameraId]) * 100.0;
          double st = m_sampleTimeSum[cameraId] / (double) m_reads[cameraId];
          completed = m_reads[cameraId] - m_fails[cameraId];
          attempted = m_reads[cameraId];
          timeouts = m_timeouts[cameraId];
          avgTime = st;
          completionRatio = sr;
        }
 private:
  CameraManager();
  int countOutputDirectories();
  void allocateCameras();
  inline int getCameraBank(int cameraNumber) { return cameraNumber / 4; }
  inline int getCameraIndex(int cameraNumber) { return cameraNumber % 4;}
   bool saveCamera(int cameraNumber);
   std::string createFilename(int cameraNumber);
   void createBundle();
  Arducam* m_cameras[8];
  long m_timeouts[8];
  long m_reads[8];
  long m_fails[8];
  timeval m_lastSampleTime[8];
  double m_sampleTimeSum[8];
  std::thread m_threads[2];
  int m_frameCount[8];
  bool m_kill = false;
  int m_bundleNumber;
  int m_bundleCapacity;

  static CameraManager* m_singleton;
  static int sm_timeoutmSecs;
  static int sm_activeCameras;
  static int sm_maxPasses;
  static bool sm_recordingOn;
  static std::string sm_imageDir;
};
#endif
