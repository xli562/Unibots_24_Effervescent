/**
 *******************************************************************
 * @file    ord_lidar_sdk.h
 * @author  weiwuxian
 * @version V0.0.1
 * @date    2022-04-24
 * @brief   This header file contains oradar lidar interface.
 *******************************************************************
 */

#ifndef ORD_LIDAR_SDK_H
#define ORD_LIDAR_SDK_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <core/common/ordlidar_protocol.h>
  /// lidar instance
  typedef struct
  {
    void *lidar;
  } ORDLidar;

  /// lidar version
  typedef struct
  {
    char version[16];
  } LidarVersion;

  ORDLidar *oradar_lidar_create(uint8_t type, int model);
  void oradar_lidar_destroy(ORDLidar **lidar);
  bool oradar_set_serial_port(ORDLidar *lidar, char *port, int baudrate);
  bool oradar_connect(ORDLidar *lidar);
  bool oradar_disconnect(ORDLidar *lidar);
  bool oradar_get_timestamp(ORDLidar *lidar, uint16_t *timestamp);
  bool oradar_get_rotation_speed(ORDLidar *lidarm, double *rotation_speed);
  bool oradar_get_firmware_version(ORDLidar *lidar, LidarVersion *version);
  bool oradar_set_rotation_speed(ORDLidar *lidar, uint16_t speed);
  bool oradar_activate(ORDLidar *lidar);
  bool oradar_deactive(ORDLidar *lidar);

  bool oradar_get_grabonescan_blocking(ORDLidar *lidar, one_scan_data_st *data, int timeout_ms);
  bool oradar_get_grabfullscan_blocking(ORDLidar *lidar, full_scan_data_st *data, int timeout_ms);
  bool oradar_get_grabonescan(ORDLidar *lidar, one_scan_data_st *data);
  bool oradar_get_grabfullscan(ORDLidar *lidar, full_scan_data_st *data);

#ifdef __cplusplus
}
#endif

#endif
