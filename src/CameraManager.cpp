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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <thread>
#include <iostream>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

#include "CameraBank.h"
#include "CameraManager.h"
// include specific arducam classes being instantiated
#include "Arducam2640.h"


CameraManager* CameraManager::m_singleton = NULL;
int CameraManager::sm_timeoutmSecs = 0;
int CameraManager::sm_activeCameras = 0;
int CameraManager::sm_maxPasses = 0;
bool CameraManager::sm_recordingOn = 0;
std::string CameraManager::sm_imageDir;
timeval sLastReportTime;

const int BUNDLECAPACITY = 5000;
CameraManager::CameraManager()
{

  m_bundleNumber = countOutputDirectories();
  createBundle();
  for(int i = 0;i < 8;i++)
    {
      m_fails[i] = 0;
      m_reads[i] = 0;
      m_timeouts[i] = 0;
      m_sampleTimeSum[i] = 0;
      
      gettimeofday(&m_lastSampleTime[i],NULL);
    }
  gettimeofday(&sLastReportTime,NULL);
}

CameraManager* CameraManager::GetSingleton()
{
  if(m_singleton == NULL) {
    m_singleton = new CameraManager();
    if(!CameraBank::InitializeBanks())
      return NULL;
    m_singleton->allocateCameras();
  }
  return m_singleton;
}

void bankImageCaptureDriver(int highBankFlag)
{
  bool highBank = highBankFlag != 0;
  CameraManager* cm = CameraManager::GetSingleton();
  const int bankOffset = highBank ? 4 : 0;
  const int bankIndex = highBank ?  1 : 0;
  int camIndex = -1;
  CameraBank* camBank = CameraBank::GetBank(highBank ? 1 : 0);
  int passCount = 0;
  while(!cm->m_kill) {
    try {
      camIndex = (camIndex + 1) % 4;
      int bankAddress = bankOffset + camIndex;

      Arducam* theCam = cm->getCamera(bankAddress);
      if(theCam == NULL) {
	struct timespec tim, tim2;
	tim.tv_sec = 0;
	tim.tv_nsec = 250000000L;
	nanosleep(&tim , &tim2);
	continue;
      }

  
      camBank->Activate(theCam);
      timeval endTime;
      long seconds, useconds;

      gettimeofday(&cm->m_lastSampleTime[bankAddress],NULL);
      theCam->capture();
      theCam->delayms(100);
      bool failed = false;
      while(theCam->isCapturing()) {
	theCam->delayms(100);
	gettimeofday(&endTime, NULL);
	seconds  = endTime.tv_sec  - cm->m_lastSampleTime[bankAddress].tv_sec;
	useconds = endTime.tv_usec - cm->m_lastSampleTime[bankAddress].tv_usec;
	long mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
	if(mtime > CameraManager::sm_timeoutmSecs)
	  {
	    cm->m_timeouts[bankAddress]++;
	    failed = true;
	    break;
	  }
	  
	pthread_yield();
      }
      if(!failed) 
	failed = !cm->saveCamera(bankAddress);

      cm->m_reads[bankAddress]++;
      if(failed)
	cm->m_fails[bankAddress]++;
	

      pthread_yield();
      // update sample time stats
      timeval currentTime;
      gettimeofday(&currentTime, NULL);
      double dseconds  = currentTime.tv_sec  - cm->m_lastSampleTime[bankAddress].tv_sec;
      double duseconds  = currentTime.tv_usec  - cm->m_lastSampleTime[bankAddress].tv_usec;
      double mtime = (dseconds * 1000 + duseconds/1000.0) + 0.5;
      cm->m_sampleTimeSum[bankAddress] += mtime;

      if(CameraManager::sm_maxPasses > 0 && CameraManager::sm_maxPasses <=  ++passCount)
	break;
    }
    catch(...)
      {
	fprintf(stderr,"error:exception quashed\n");
      }
  }
  fprintf(stderr,"Camera %d in bank %d EXITED\n",camIndex,bankIndex);
  pthread_exit(NULL);
}

void CameraManager::StartCapture()
{
  sleep(1);    // one last chance for all the cameras to catch up
  // ensure only threads needed to actually run cameras get started and waited on
  bool thread1Running = false;
  for(int i = 0;i < 4;i++)
    if(m_cameras[i] != NULL)
      {
	m_threads[0] = std::thread(bankImageCaptureDriver,0);
	thread1Running = true;
	break;
      }
  bool thread2Running = false;
  for(int i = 4;i < 8;i++)
    if(m_cameras[i] != NULL)
      {
	m_threads[1] = std::thread(bankImageCaptureDriver,1);
	thread2Running = true;
	break;
      }
  if(thread1Running)
    m_threads[0].join();
  if(thread2Running)
    m_threads[1].join();
}

void CameraManager::allocateCameras()
{
  int mask = 1;
  for(int i = 0;i < 8;i++)
    {
      if((mask & sm_activeCameras) == 0) {
	mask = mask * 2;
	m_cameras[i] = NULL;
	continue;
      }
      mask = mask * 2;
#if LOG_INFO
      fprintf(stderr,"info:testing camera %d\n",i);
#endif
      int cameraBank = i / 4;
      // Allocating camera i,cameraBank,cameraIndex
      CameraBank* camBank = CameraBank::GetBank(cameraBank);
      
      int spifd = camBank->GetSPIFD();
      int i2cfd = camBank->GetI2CFD();
#if LOG_INFO
      fprintf(stderr,"info:camera bank %d spi %d i2c %d\n",cameraBank,spifd,i2cfd);
#endif
      Arducam* newCam = new Arducam2640(i,camBank,spifd,i2cfd);
      try {
	// need the correct CS lines illuminated to talk to the camera
	camBank->Activate(newCam);
#if LOG_INFO
	fprintf(stderr,"info:activated camera %d\n",i);
#endif
	// verify correct operation of camera and bring it online
        
#if LOG_INFO
        if(newCam->powerUp())
	  fprintf(stderr,"info:created camera %d\n",i);
#else
        newCam->powerUp();
#endif
      }
      catch(const char*x)
	{
	  fprintf(stderr,x);
	}
      if(newCam != NULL && newCam->isInitialized()) {
	m_cameras[i] = newCam;
      } else {
	if(newCam != NULL)
	  delete newCam;
	m_cameras[i] = NULL;
      }
    }
}

std::string CameraManager::createFilename(int cameraNumber)
{
  timeval curTime;
  char fname[256];
  gettimeofday(&curTime, NULL);
    sprintf(fname, "%s/bundle_%d//C%d-%lu.%lu.jpg",
	  sm_imageDir.c_str(),
	    m_bundleNumber,
	  cameraNumber,
	  curTime.tv_sec,
	  curTime.tv_usec);
    return std::string(fname);
}

bool CameraManager::saveCamera(int cameraNumber) {
  std::string filename = createFilename(cameraNumber);

  FILE *fp = NULL;
  int retries = 1000;
  if(CameraManager::sm_recordingOn)
    {
      while(fp == NULL && retries > 0)
	{
      
	  fp = fopen(filename.c_str(), "w");
	  if(!fp)
	    {
	      fprintf(stderr,"filesystem stuttering\n");
	      --retries;
	      filename = createFilename(cameraNumber);
	      
	    }
	}
  
      if (!fp) {
	printf("Error: could not open %s\n", filename.c_str());
	return false;
      }
    }
  //fprintf(stderr,"transfer %d\n",cameraNumber);
  bool result = false;
  result = m_cameras[cameraNumber]->transferImageBufferToFile(fp);
  if(CameraManager::sm_recordingOn)
    {
      fclose(fp);
      if(result != false)
	{
	  if(--m_bundleCapacity < 0)
	    {
	      ++m_bundleNumber;
	      createBundle();
	    }
	}
      else
	remove(filename.c_str());
    }
  if(result)
      m_cameras[cameraNumber]->setLastSave(filename.c_str());
  return result;
}
void CameraManager::createBundle()
{
  char dirName[256];
  sprintf(dirName,"%s/bundle_%d",
	  CameraManager::sm_imageDir.c_str(),
	  m_bundleNumber);
  
    mkdir(dirName, 0777);
    
    m_bundleCapacity = BUNDLECAPACITY;
}
int CameraManager::countOutputDirectories()
{

  struct dirent *dp;
  DIR *fd;

  if ((fd = opendir(CameraManager::sm_imageDir.c_str())) == NULL) {
    fprintf(stderr, "listdir: can't open %s\n", CameraManager::sm_imageDir.c_str());
    return -1;
  }
  int numBundles = 0;
  while ((dp = readdir(fd)) != NULL) {
  if (!strcmp(dp->d_name, ".") || !strcmp(dp->d_name, ".."))
    continue;    /* skip self and parent */
  if(strncmp("bundle",dp->d_name,6) == 0)
    ++numBundles;
  }
  closedir(fd);
  return numBundles;
}
