#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

<<<<<<< HEAD
#define THISFIRMWARE "AntennaTracker V4.6.1-beta1"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,6,1,FIRMWARE_VERSION_TYPE_BETA

#define FW_MAJOR 4
#define FW_MINOR 6
#define FW_PATCH 1
#define FW_TYPE FIRMWARE_VERSION_TYPE_BETA
=======
#define THISFIRMWARE "AntennaTracker V4.5.0-beta4"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,5,0,FIRMWARE_VERSION_TYPE_BETA+3

#define FW_MAJOR 4
#define FW_MINOR 5
#define FW_PATCH 0
#define FW_TYPE FIRMWARE_VERSION_TYPE_BETA+3
>>>>>>> AntennaTracker: prepare for 4.5.0-beta4

#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
