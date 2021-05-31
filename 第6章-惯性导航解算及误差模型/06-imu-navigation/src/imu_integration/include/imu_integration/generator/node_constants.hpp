#ifndef IMU_INTEGRATION_NODE_CONSTANTS_HPP_
#define IMU_INTEGRATION_NODE_CONSTANTS_HPP_

#include <cmath>
#include <ros/ros.h>

namespace imu_integration {

namespace generator {

// 原始数据
/*constexpr double kRhoX = 3.0;
constexpr double kRhoY = 4.0;
constexpr double kRhoZ = 1.0;

constexpr double kOmegaXY = M_PI / 10.0;
constexpr double kOmegaZ = 10.0 * kOmegaXY;

constexpr double kYaw = M_PI / 10.0;
constexpr double kPitch = 0.20;
constexpr double kRoll = 0.10;*/

// 静止状态
/*constexpr double kRhoX = 0.0;
constexpr double kRhoY = 0.0;
constexpr double kRhoZ = 0.0;

constexpr double kOmegaXY = 0.0;
constexpr double kOmegaZ = 0.0;

constexpr double kYaw = 0.0;
constexpr double kPitch = 0.0;
constexpr double kRoll = 0.0;*/  

// 匀速运动
/*constexpr double kRhoX = 3.0;
constexpr double kRhoY = 4.0;
constexpr double kRhoZ = 0.0;

constexpr double kOmegaXY = M_PI / 10.0;
constexpr double kOmegaZ = 0;

constexpr double kYaw = M_PI / 10.0;
constexpr double kPitch = 0.20;
constexpr double kRoll = 0.10;*/


// 快速转弯
constexpr double kRhoX = 3.0;
constexpr double kRhoY = 4.0;
constexpr double kRhoZ = 1.0;

constexpr double kOmegaXY = M_PI / 3.0;
constexpr double kOmegaZ = 0;

constexpr double kYaw = M_PI / 10.0;
constexpr double kPitch = 0.20;
constexpr double kRoll = 0.10;

}  // namespace generator

}  // namespace imu_integration

#endif  // IMU_INTEGRATION_NODE_CONSTANTS_HPP_