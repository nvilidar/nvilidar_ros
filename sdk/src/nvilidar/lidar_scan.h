#ifndef _LIDAR_SCAN_H_
#define _LIDAR_SCAN_H_

#include <stdint.h>
#include <vector>


#if defined(_WIN32)
#pragma pack()
#endif


/**
 * @brief The Laser Point struct
 * @note angle unit: rad.\n
 * range unit: meter.\n
 */
typedef struct {
  /// lidar angle. unit(rad)
  float angle;
  /// lidar range. unit(m)
  float range;
  /// lidar intensity
  float intensity;
} LidarPoint;

/**
 * @brief A struct for returning configuration from the NVILIDAR
 * @note angle unit: rad.\n
 * time unit: second.\n
 * range unit: meter.\n
 */
typedef struct {
  /// Start angle for the laser scan [rad].  0 is forward and angles are measured clockwise when viewing NVILIDAR from the top.
  float min_angle;
  /// Stop angle for the laser scan [rad].   0 is forward and angles are measured clockwise when viewing NVILIDAR from the top.
  float max_angle;
  /// angle resoltuion [rad]
  float angle_increment;
  /// Scan resoltuion [s]
  float time_increment;
  /// Time between scans
  float scan_time;
  /// Minimum range [m]
  float min_range;
  /// Maximum range [m]
  float max_range;
} LidarConfig;


typedef struct {
  /// System time when first range was measured in nanoseconds
  uint64_t stamp;
  /// Array of lidar points
  std::vector<LidarPoint> points;
  /// Configuration of scan
  LidarConfig config;
} LidarScan;

#endif
