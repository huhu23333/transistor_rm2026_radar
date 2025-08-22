//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <stdio.h>
#include <stdlib.h>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <string.h>
#include "livox_sdk.h"
#include <math.h>



__declspec(dllexport) int pyif_Init();
__declspec(dllexport) void pyif_Uninit();
__declspec(dllexport) void pyif_draw2dImageF(double* yaws, double* pitchs, double* values, uint64_t point_number, uint16_t imageSize, uint16_t values_number, double* result, uint8_t* mask);
#define PYIF_PonitDataArrayLen 4096
typedef struct {
  int32_t x;            /**< X axis, Unit:mm */
  int32_t y;            /**< Y axis, Unit:mm */
  int32_t z;            /**< Z axis, Unit:mm */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;          /**< Tag */
} PYIF_PonitData;
__declspec(dllexport) PYIF_PonitData pyif_ponitDataArray[2][PYIF_PonitDataArrayLen];
__declspec(dllexport) bool pyif_usedFlag_0 = true;
__declspec(dllexport) bool pyif_usedFlag_1 = true;
uint8_t pyif_writeTo = 0;
uint32_t pyif_writeIndex = 0;
__declspec(dllexport) uint64_t pyif_writeCount = 0;



typedef enum {
  kDeviceStateDisconnect = 0,
  kDeviceStateConnect = 1,
  kDeviceStateSampling = 2,
} DeviceState;

typedef struct {
  uint8_t handle;
  DeviceState device_state;
  DeviceInfo info;
} DeviceItem;

DeviceItem devices[kMaxLidarCount];
uint32_t data_recveive_count[kMaxLidarCount];

/** Connect all the broadcast device. */
int lidar_count = 0;
char broadcast_code_list[kMaxLidarCount][kBroadcastCodeSize];

/** Connect the broadcast device in list, please input the broadcast code and modify the BROADCAST_CODE_LIST_SIZE. */
/*#define BROADCAST_CODE_LIST_SIZE  3
int lidar_count = BROADCAST_CODE_LIST_SIZE;
char broadcast_code_list[kMaxLidarCount][kBroadcastCodeSize] = {
  "000000000000002",
  "000000000000003",
  "000000000000004"
};*/

/** Receiving error message from Livox Lidar. */
void OnLidarErrorStatusCallback(livox_status status, uint8_t handle, ErrorMessage *message) {
  static uint32_t error_message_count = 0;
  if (message != NULL) {
    ++error_message_count;
    if (0 == (error_message_count % 100)) {
      printf("handle: %u\n", handle);
      printf("temp_status : %u\n", message->lidar_error_code.temp_status);
      printf("volt_status : %u\n", message->lidar_error_code.volt_status);
      printf("motor_status : %u\n", message->lidar_error_code.motor_status);
      printf("dirty_warn : %u\n", message->lidar_error_code.dirty_warn);
      printf("firmware_err : %u\n", message->lidar_error_code.firmware_err);
      printf("pps_status : %u\n", message->lidar_error_code.device_status);
      printf("fan_status : %u\n", message->lidar_error_code.fan_status);
      printf("self_heating : %u\n", message->lidar_error_code.self_heating);
      printf("ptp_status : %u\n", message->lidar_error_code.ptp_status);
      printf("time_sync_status : %u\n", message->lidar_error_code.time_sync_status);
      printf("system_status : %u\n", message->lidar_error_code.system_status);
    }
  }
}

/** Receiving point cloud data from Livox LiDAR. */
void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data) {
  if (data) {
    data_recveive_count[handle] ++ ;
    /** Parsing the timestamp and the point cloud data. */
    if ( data ->data_type == kDualExtendCartesian) {
      for(uint32_t data_index = 0; data_index < data_num; data_index += 1) {
        
        LivoxDualExtendRawPoint *p_point_data = (LivoxDualExtendRawPoint *)(data->data + data_index*sizeof(LivoxDualExtendRawPoint));

        PYIF_PonitData data_to_check;
        data_to_check.x = p_point_data -> x1;
        data_to_check.y = p_point_data -> y1;
        data_to_check.z = p_point_data -> z1;
        data_to_check.reflectivity = p_point_data -> reflectivity1;
        data_to_check.tag = p_point_data -> tag1;
        if (data_to_check.x != 0 || data_to_check.y != 0 || data_to_check.z != 0) {
          pyif_writeCount += 1;
          pyif_ponitDataArray[pyif_writeTo][pyif_writeIndex].x = data_to_check.x;
          pyif_ponitDataArray[pyif_writeTo][pyif_writeIndex].y = data_to_check.y;
          pyif_ponitDataArray[pyif_writeTo][pyif_writeIndex].z = data_to_check.z;
          pyif_ponitDataArray[pyif_writeTo][pyif_writeIndex].reflectivity = data_to_check.reflectivity;
          pyif_ponitDataArray[pyif_writeTo][pyif_writeIndex].tag = data_to_check.tag;
          pyif_writeIndex += 1;
          if (pyif_writeIndex >= PYIF_PonitDataArrayLen) {
            if (pyif_writeTo == 0) {
              pyif_usedFlag_0 = false;
              pyif_writeTo = 1;
            } else if (pyif_writeTo == 1) {
              pyif_usedFlag_1 = false;
              pyif_writeTo = 0;
            }
            pyif_writeIndex = 0;
          }
        }
        data_to_check.x = p_point_data -> x2;
        data_to_check.y = p_point_data -> y2;
        data_to_check.z = p_point_data -> z2;
        data_to_check.reflectivity = p_point_data -> reflectivity2;
        data_to_check.tag = p_point_data -> tag2;
        if (data_to_check.x != 0 || data_to_check.y != 0 || data_to_check.z != 0) {
          pyif_writeCount += 1;
          pyif_ponitDataArray[pyif_writeTo][pyif_writeIndex].x = data_to_check.x;
          pyif_ponitDataArray[pyif_writeTo][pyif_writeIndex].y = data_to_check.y;
          pyif_ponitDataArray[pyif_writeTo][pyif_writeIndex].z = data_to_check.z;
          pyif_ponitDataArray[pyif_writeTo][pyif_writeIndex].reflectivity = data_to_check.reflectivity;
          pyif_ponitDataArray[pyif_writeTo][pyif_writeIndex].tag = data_to_check.tag;
          pyif_writeIndex += 1;
          if (pyif_writeIndex >= PYIF_PonitDataArrayLen) {
            if (pyif_writeTo == 0) {
              pyif_usedFlag_0 = false;
              pyif_writeTo = 1;
            } else if (pyif_writeTo == 1) {
              pyif_usedFlag_1 = false;
              pyif_writeTo = 0;
            }
            pyif_writeIndex = 0;
          }
        }
      }
      //printf("data_type %d packet num %d\n", data->data_type, data_recveive_count[handle]);
    }
    else {
      printf("Not LivoxDualExtendRawPoint!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    }
  }
}

/** Callback function of starting sampling. */
void OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {
  printf("OnSampleCallback statue %d handle %d response %d \n", status, handle, response);
  if (status == kStatusSuccess) {
    if (response != 0) {
      devices[handle].device_state = kDeviceStateConnect;
    }
  } else if (status == kStatusTimeout) {
    devices[handle].device_state = kDeviceStateConnect;
  }
}

/** Callback function of stopping sampling. */
void OnStopSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {
}

/** Query the firmware version of Livox LiDAR. */
void OnDeviceInformation(livox_status status, uint8_t handle, DeviceInformationResponse *ack, void *data) {
  if (status != kStatusSuccess) {
    printf("Device Query Informations Failed %d\n", status);
  }
  if (ack) {
    printf("firm ver: %d.%d.%d.%d\n",
           ack->firmware_version[0],
           ack->firmware_version[1],
           ack->firmware_version[2],
           ack->firmware_version[3]);
  }
}

void LidarConnect(const DeviceInfo *info) {
  uint8_t handle = info->handle;
  QueryDeviceInformation(handle, OnDeviceInformation, NULL);
  if (devices[handle].device_state == kDeviceStateDisconnect) {
    devices[handle].device_state = kDeviceStateConnect;
    devices[handle].info = *info;
  }
}

void LidarDisConnect(const DeviceInfo *info) {
  uint8_t handle = info->handle;
  devices[handle].device_state = kDeviceStateDisconnect;
}

void LidarStateChange(const DeviceInfo *info) {
  uint8_t handle = info->handle;
  devices[handle].info = *info;
}

/** Callback function of changing of device state. */
void OnDeviceInfoChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == NULL) {
    return;
  }

  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }
  if (type == kEventConnect) {
    LidarConnect(info);
    printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
  } else if (type == kEventDisconnect) {
    LidarDisConnect(info);
    printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
  } else if (type == kEventStateChange) {
    LidarStateChange(info);
    printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
  }

  if (devices[handle].device_state == kDeviceStateConnect) {
    printf("Device Working State %d\n", devices[handle].info.state);
    if (devices[handle].info.state == kLidarStateInit) {
      printf("Device State Change Progress %u\n", devices[handle].info.status.progress);
    } else {
      printf("Device State Error Code 0X%08x\n", devices[handle].info.status.status_code.error_code);
    }
    printf("Device feature %d\n", devices[handle].info.feature);
    SetErrorMessageCallback(handle, OnLidarErrorStatusCallback);
    if (devices[handle].info.state == kLidarStateNormal) {
      LidarStartSampling(handle, OnSampleCallback, NULL);
      devices[handle].device_state = kDeviceStateSampling;
    }
  }
}

/** Callback function when broadcast message received.
 * You need to add listening device broadcast code and set the point cloud data callback in this function.
 */
void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == NULL || info->dev_type == kDeviceTypeHub) {
    return;
  }

  printf("Receive Broadcast Code %s\n", info->broadcast_code);

  if (lidar_count > 0) {
    bool found = false;
    int i = 0;
    for (i = 0; i < lidar_count; ++i) {
      if (strncmp(info->broadcast_code, broadcast_code_list[i], kBroadcastCodeSize) == 0) {
        found = true;
        break;
      }
    }
    if (!found) {
      return;
    }
  }

  bool result = false;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess) {
    /** Set the point cloud data for a specific Livox LiDAR. */
    SetDataCallback(handle, GetLidarData, NULL);
    devices[handle].handle = handle;
    devices[handle].device_state = kDeviceStateDisconnect;
  }
}

int pyif_Init() {

  printf("Livox SDK initializing.\n");
/** Initialize Livox-SDK. */
  if (!Init()) {
    printf("Initialize Failed :( \n");
    return -1;
  }
  printf("Livox SDK has been initialized.\n");

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  printf("Livox SDK version %d.%d.%d .\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

  memset(devices, 0, sizeof(devices));
  memset(data_recveive_count, 0, sizeof(data_recveive_count));

/** Set the callback function receiving broadcast message from Livox LiDAR. */
  SetBroadcastCallback(OnDeviceBroadcast);

/** Set the callback function called when device state change,
 * which means connection/disconnection and changing of LiDAR state.
 */
  SetDeviceStateUpdateCallback(OnDeviceInfoChange);

/** Start the device discovering routine. */
  if (!Start()) {
    Uninit();
    printf("Start Failed :( \n");
    return -1;
  }
  printf("Start discovering device.\n");

  return 0;
}

void pyif_Uninit() {

  int i = 0;
  for (i = 0; i < kMaxLidarCount; ++i) {
    if (devices[i].device_state == kDeviceStateSampling) {
/** Stop the sampling of Livox LiDAR. */
      LidarStopSampling(devices[i].handle, OnStopSampleCallback, NULL);
    }
  }

/** Uninitialize Livox-SDK. */
  Uninit();

}

void pyif_draw2dImageF(double* yaws, double* pitchs, double* values, uint64_t point_number, uint16_t imageSize, uint16_t values_number, double* result, uint8_t* mask) {
  for (uint64_t point_index = 0; point_index < point_number; point_index += 1) {
    double value = values[point_index];
    for (uint16_t values_index = 0; values_index < values_number; values_index += 1) {
      if (!isnan(value)) {
        int x_index = max(min((int)((yaws[point_index]/1.2287117934040082+0.5) * imageSize), imageSize-1), 0);
        int y_index = max(min((int)((-pitchs[point_index]/1.2287117934040082+0.5) * imageSize), imageSize-1), 0);
        result[x_index + imageSize * y_index + imageSize * imageSize * values_index] = values[point_index + point_number * values_index];
        mask[x_index + imageSize * y_index] = 255;
      }
    }
  }
}

/* int main(int argc, const char *argv[]) {
  
  pyif_Init();

#ifdef WIN32
  Sleep(30000);
#else
  sleep(30);
#endif

  pyif_Uninit();
}
 */