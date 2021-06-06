/*
 * @Description: Error-State Kalman Filter for IMU-Lidar-GNSS-Odo fusion
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#include <cstdlib>
#include <limits>

#include <cmath>
#include <fstream>
#include <iostream>
#include <ostream>

// use sophus to handle so3 hat & SO3 log operations:
#include <sophus/so3.hpp>

#include "lidar_localization/models/kalman_filter/error_state_kalman_filter.hpp"

#include "lidar_localization/global_defination/global_defination.h"

#include "glog/logging.h"

namespace lidar_localization {

// 误差卡尔曼滤波器
ErrorStateKalmanFilter::ErrorStateKalmanFilter(const YAML::Node &node) {
  //
  // 提取配置:
  //
  // a. 地球常量:
  EARTH.GRAVITY_MAGNITUDE = node["earth"]["gravity_magnitude"].as<double>();
  EARTH.LATITUDE = node["earth"]["latitude"].as<double>();
  EARTH.LATITUDE *= M_PI / 180.0;

  // b. 先验状态的协方差:
  COV.PRIOR.POSI = node["covariance"]["prior"]["pos"].as<double>();
  COV.PRIOR.VEL = node["covariance"]["prior"]["vel"].as<double>();
  COV.PRIOR.ORI = node["covariance"]["prior"]["ori"].as<double>();
  COV.PRIOR.EPSILON = node["covariance"]["prior"]["epsilon"].as<double>();
  COV.PRIOR.DELTA = node["covariance"]["prior"]["delta"].as<double>();

  // c. 过程噪声的协方差矩阵:
  COV.PROCESS.ACCEL = node["covariance"]["process"]["accel"].as<double>();
  COV.PROCESS.GYRO = node["covariance"]["process"]["gyro"].as<double>();
  COV.PROCESS.BIAS_ACCEL =
      node["covariance"]["process"]["bias_accel"].as<double>();
  COV.PROCESS.BIAS_GYRO =
      node["covariance"]["process"]["bias_gyro"].as<double>();

  // d. 测量噪声的协方差矩阵:
  COV.MEASUREMENT.POSE.POSI =
      node["covariance"]["measurement"]["pose"]["pos"].as<double>();
  COV.MEASUREMENT.POSE.ORI =
      node["covariance"]["measurement"]["pose"]["ori"].as<double>();

  // 输出对应的卡尔曼滤波器参数:
  LOG(INFO) << std::endl
            << "Error-State Kalman Filter params:" << std::endl
            << "\tgravity magnitude: " << EARTH.GRAVITY_MAGNITUDE << std::endl
            << "\tlatitude: " << EARTH.LATITUDE << std::endl
            << std::endl
            << "\tprior cov. pos.: " << COV.PRIOR.POSI << std::endl
            << "\tprior cov. vel.: " << COV.PRIOR.VEL << std::endl
            << "\tprior cov. ori: " << COV.PRIOR.ORI << std::endl
            << "\tprior cov. epsilon.: " << COV.PRIOR.EPSILON << std::endl
            << "\tprior cov. delta.: " << COV.PRIOR.DELTA << std::endl
            << std::endl
            << "\tprocess noise gyro.: " << COV.PROCESS.GYRO << std::endl
            << "\tprocess noise accel.: " << COV.PROCESS.ACCEL << std::endl
            << std::endl
            << "\tmeasurement noise pose.: " << std::endl
            << "\t\tpos: " << COV.MEASUREMENT.POSE.POSI
            << ", ori.: " << COV.MEASUREMENT.POSE.ORI << std::endl
            << std::endl
            << std::endl;

  //
  // 初始化滤波器:
  //
  // a. 地球常量:
  g_ = Eigen::Vector3d(0.0, 0.0, EARTH.GRAVITY_MAGNITUDE);
  // b. 重置状态和先验协方差矩阵:
  ResetState();
  ResetCovariance();

  // c. 过程噪声:
  Q_.block<3, 3>(kIndexNoiseAccel, kIndexNoiseAccel) = COV.PROCESS.ACCEL * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseGyro, kIndexNoiseGyro) = COV.PROCESS.GYRO * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseBiasAccel, kIndexNoiseBiasAccel) = COV.PROCESS.BIAS_ACCEL * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseBiasGyro, kIndexNoiseBiasGyro) = COV.PROCESS.BIAS_GYRO * Eigen::Matrix3d::Identity();

  // d. 测量噪声:
  RPose_.block<3, 3>(0, 0) = COV.MEASUREMENT.POSE.POSI * Eigen::Matrix3d::Identity();
  RPose_.block<3, 3>(3, 3) = COV.MEASUREMENT.POSE.ORI * Eigen::Matrix3d::Identity();

  // e. 过程方程:
  F_.block<3, 3>(kIndexErrorPos, kIndexErrorVel) = Eigen::Matrix3d::Identity();
  F_.block<3, 3>(kIndexErrorOri, kIndexErrorGyro) = -Eigen::Matrix3d::Identity();

  B_.block<3, 3>(kIndexErrorOri, kIndexNoiseGyro) = Eigen::Matrix3d::Identity();
  B_.block<3, 3>(kIndexErrorAccel, kIndexNoiseBiasAccel) = Eigen::Matrix3d::Identity();
  B_.block<3, 3>(kIndexErrorGyro, kIndexNoiseBiasGyro) = Eigen::Matrix3d::Identity();

  // f. 测量方程:
  GPose_.block<3, 3>(0, kIndexErrorPos) = Eigen::Matrix3d::Identity();
  GPose_.block<3, 3>(3, kIndexErrorOri) = Eigen::Matrix3d::Identity();
  CPose_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  CPose_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

  // init soms:
  QPose_.block<kDimMeasurementPose, kDimState>(0, 0) = GPose_;
}

/**
 * @brief  初始化滤波器
 * @param  pose, 初始位置
 * @param  vel, 初始速度
 * @param  imu_data, 初始的IMU测量值
 * @return true if success false otherwise
 */
void ErrorStateKalmanFilter::Init(const Eigen::Vector3d &vel,
                                  const IMUData &imu_data) {
  // 初始化历程计:
  Eigen::Matrix3d C_nb = imu_data.GetOrientationMatrix().cast<double>();  // 这个为车体坐标系到导航坐标系的旋转矩阵
  // a. 使用IMU估计的姿态作为初始姿态
  pose_.block<3, 3>(0, 0) = C_nb;
  // b. 将速度转换到导航坐标系下:
  vel_ = C_nb * vel;

  // 设置初始位置:
  init_pose_ = pose_;

  // 初始化imu储存序列:
  imu_data_buff_.clear();
  imu_data_buff_.push_back(imu_data);

  // 初始化上一帧数据的时间戳:
  time_ = imu_data.time;

  // 设置过程方程:
  Eigen::Vector3d linear_acc_init(imu_data.linear_acceleration.x,
                                  imu_data.linear_acceleration.y,
                                  imu_data.linear_acceleration.z);
  Eigen::Vector3d angular_vel_init(imu_data.angular_velocity.x,
                                   imu_data.angular_velocity.y,
                                   imu_data.angular_velocity.z);
  // covert to navigation frame:
  linear_acc_init = GetUnbiasedLinearAcc(linear_acc_init, C_nb);    // 导航坐标系下
  angular_vel_init = GetUnbiasedAngularVel(angular_vel_init, C_nb); // 机体坐标系下

  // 初始化过程方程(主要是计算F和B):
  UpdateProcessEquation(linear_acc_init, angular_vel_init);

  LOG(INFO) << std::endl
            << "Kalman Filter Inited at " << static_cast<int>(time_)
            << std::endl
            << "Init Position: " << pose_(0, 3) << ", " << pose_(1, 3) << ", "
            << pose_(2, 3) << std::endl
            << "Init Velocity: " << vel_.x() << ", " << vel_.y() << ", "
            << vel_.z() << std::endl;
}

// 状态更新和误差更新的核心函数
/**
 * @brief  卡尔曼滤波器更新状态值
 * @param  imu_data, 输入的IMU测量值
 * @return true if success false otherwise
 */
bool ErrorStateKalmanFilter::Update(const IMUData &imu_data) {
  //
  // ESKF框架
  //
  // 更新IMU缓存区:
  if (time_ < imu_data.time) {
    // 更新IMU里程计:
    Eigen::Vector3d linear_acc_mid;       //加速度均值
    Eigen::Vector3d angular_vel_mid;      //角速度均值
    imu_data_buff_.push_back(imu_data);   //保存当前的Imu数据

    //更新里程计的估计值
    UpdateOdomEstimation(linear_acc_mid, angular_vel_mid); 
    imu_data_buff_.pop_front();           //将前一个imu数据弹出

    // 更新误差的先验估计:
    double T = imu_data.time - time_;
    UpdateErrorEstimation(T, linear_acc_mid, angular_vel_mid);

    // 更新时间:
    time_ = imu_data.time;

    return true;
  }

  return false;
}

/**
 * @brief  卡尔曼误差矫正
 * frame
 * @param  measurement_type, input measurement type
 * @param  measurement, input measurement
 * @return void
 */
bool ErrorStateKalmanFilter::Correct(const IMUData &imu_data,
                                     const MeasurementType &measurement_type,
                                     const Measurement &measurement) {
  static Measurement measurement_;

  // 获取时间:
  double time_delta = measurement.time - time_;

  if (time_delta > -0.05) {
    // 如果时间差距太大，则再更新一次预测:
    if (time_ < measurement.time) {
      Update(imu_data);
    }

    // 获取观测在导航坐标系中:
    measurement_ = measurement;
    measurement_.T_nb = init_pose_ * measurement_.T_nb;

    // 矫正误差的估计值:
    CorrectErrorEstimation(measurement_type, measurement_);

    // 去除误差:
    EliminateError();

    // reset error state:
    ResetState();

    return true;
  }

  LOG(INFO) << "ESKF Correct: Observation is not synced with filter. Skip, "
            << (int)measurement.time << " <-- " << (int)time_ << " @ "
            << time_delta << std::endl;

  return false;
}

/**
 * @brief  get odometry estimation
 * @param  pose, init pose
 * @param  vel, init vel
 * @return void
 */
void ErrorStateKalmanFilter::GetOdometry(Eigen::Matrix4f &pose,
                                         Eigen::Vector3f &vel) {
  // init:
  Eigen::Matrix4d pose_double = pose_;
  Eigen::Vector3d vel_double = vel_;

  // eliminate error:
  // a. position:
  pose_double.block<3, 1>(0, 3) =
      pose_double.block<3, 1>(0, 3) - X_.block<3, 1>(kIndexErrorPos, 0);
  // b. velocity:
  vel_double = vel_double - X_.block<3, 1>(kIndexErrorVel, 0);
  // c. orientation:
  Eigen::Matrix3d C_nn =
      Sophus::SO3d::exp(X_.block<3, 1>(kIndexErrorOri, 0)).matrix();
  pose_double.block<3, 3>(0, 0) = C_nn * pose_double.block<3, 3>(0, 0);

  // finally:
  pose_double = init_pose_.inverse() * pose_double;
  vel_double = init_pose_.block<3, 3>(0, 0).transpose() * vel_double;

  pose = pose_double.cast<float>();
  vel = vel_double.cast<float>();
}

/**
 * @brief  get covariance estimation
 * @param  cov, covariance output
 * @return void
 */
void ErrorStateKalmanFilter::GetCovariance(Cov &cov) {
  static int OFFSET_X = 0;
  static int OFFSET_Y = 1;
  static int OFFSET_Z = 2;

  // a. delta position:
  cov.pos.x = P_(kIndexErrorPos + OFFSET_X, kIndexErrorPos + OFFSET_X);
  cov.pos.y = P_(kIndexErrorPos + OFFSET_Y, kIndexErrorPos + OFFSET_Y);
  cov.pos.z = P_(kIndexErrorPos + OFFSET_Z, kIndexErrorPos + OFFSET_Z);

  // b. delta velocity:
  cov.vel.x = P_(kIndexErrorVel + OFFSET_X, kIndexErrorVel + OFFSET_X);
  cov.vel.y = P_(kIndexErrorVel + OFFSET_Y, kIndexErrorVel + OFFSET_Y);
  cov.vel.z = P_(kIndexErrorVel + OFFSET_Z, kIndexErrorVel + OFFSET_Z);

  // c. delta orientation:
  cov.ori.x = P_(kIndexErrorOri + OFFSET_X, kIndexErrorOri + OFFSET_X);
  cov.ori.y = P_(kIndexErrorOri + OFFSET_Y, kIndexErrorOri + OFFSET_Y);
  cov.ori.z = P_(kIndexErrorOri + OFFSET_Z, kIndexErrorOri + OFFSET_Z);

  // d. gyro. bias:
  cov.gyro_bias.x =
      P_(kIndexErrorGyro + OFFSET_X, kIndexErrorGyro + OFFSET_X);
  cov.gyro_bias.y =
      P_(kIndexErrorGyro + OFFSET_Y, kIndexErrorGyro + OFFSET_Y);
  cov.gyro_bias.z =
      P_(kIndexErrorGyro + OFFSET_Z, kIndexErrorGyro + OFFSET_Z);

  // e. accel bias:
  cov.accel_bias.x =
      P_(kIndexErrorAccel + OFFSET_X, kIndexErrorAccel + OFFSET_X);
  cov.accel_bias.y =
      P_(kIndexErrorAccel + OFFSET_Y, kIndexErrorAccel + OFFSET_Y);
  cov.accel_bias.z =
      P_(kIndexErrorAccel + OFFSET_Z, kIndexErrorAccel + OFFSET_Z);
}

/**
 * @brief  get unbiased angular velocity in body frame
 * @param  angular_vel, angular velocity measurement
 * @param  R, corresponding orientation of measurement
 * @return unbiased angular velocity in body frame
 */
inline Eigen::Vector3d ErrorStateKalmanFilter::GetUnbiasedAngularVel(
    const Eigen::Vector3d &angular_vel, const Eigen::Matrix3d &R) {
  return angular_vel - gyro_bias_;
}

/**
 * @brief  获取无偏的线加速度在导航坐标系下
 * @param  linear_acc, linear acceleration measurement
 * @param  R, corresponding orientation of measurement
 * @return unbiased linear acceleration in navigation frame
 */
inline Eigen::Vector3d
ErrorStateKalmanFilter::GetUnbiasedLinearAcc(const Eigen::Vector3d &linear_acc,
                                             const Eigen::Matrix3d &R) {
  return R * (linear_acc - accl_bias_) - g_;
}

/**
 * @brief  获取角度增量（旋转向量）
 * @param  index_curr, 当前的imu数据索引
 * @param  index_prev, 上一次的imu数据索引
 * @param  angular_delta, 角度增量输出
 * @return true if success false otherwise
 */
bool ErrorStateKalmanFilter::GetAngularDelta(const size_t index_curr,
                                             const size_t index_prev,
                                             Eigen::Vector3d &angular_delta,
                                             Eigen::Vector3d &angular_vel_mid) {
  // 判断索引有效性
  if (index_curr <= index_prev || imu_data_buff_.size() <= index_curr) {
    return false;
  }

  // 获取当前帧和前一帧的IMU数据
  const IMUData &imu_data_curr = imu_data_buff_.at(index_curr);
  const IMUData &imu_data_prev = imu_data_buff_.at(index_prev);

  // 获取时间间隔
  double delta_t = imu_data_curr.time - imu_data_prev.time;

  // 当前的角速度
  Eigen::Vector3d angular_vel_curr = Eigen::Vector3d(
      imu_data_curr.angular_velocity.x, imu_data_curr.angular_velocity.y,
      imu_data_curr.angular_velocity.z);
  Eigen::Matrix3d R_curr = imu_data_curr.GetOrientationMatrix().cast<double>();
  // 无偏的当前的角速度
  angular_vel_curr = GetUnbiasedAngularVel(angular_vel_curr, R_curr);

  // 前一帧的角速度
  Eigen::Vector3d angular_vel_prev = Eigen::Vector3d(
      imu_data_prev.angular_velocity.x, imu_data_prev.angular_velocity.y,
      imu_data_prev.angular_velocity.z);
  Eigen::Matrix3d R_prev = imu_data_prev.GetOrientationMatrix().cast<double>();
  // 无偏的前一帧的角速度
  angular_vel_prev = GetUnbiasedAngularVel(angular_vel_prev, R_prev);

  // 旋转向量
  angular_delta = 0.5 * delta_t * (angular_vel_curr + angular_vel_prev);

  // 当前帧和前一帧的角速度均值
  angular_vel_mid = 0.5 * (angular_vel_curr + angular_vel_prev);
  return true;
}

/**
 * @brief  更新旋转姿态
 * @param  angular_delta, 旋转矢量
 * @param  R_curr, current orientation
 * @param  R_prev, previous orientation
 * @return void
 */
void ErrorStateKalmanFilter::UpdateOrientation(
    const Eigen::Vector3d &angular_delta, Eigen::Matrix3d &R_curr,
    Eigen::Matrix3d &R_prev) {
  // 幅值（旋转角）:
  double angular_delta_mag = angular_delta.norm();
  // 旋转方向:
  Eigen::Vector3d angular_delta_dir = angular_delta.normalized();

  // 构造deltaq:
  double angular_delta_cos = cos(angular_delta_mag / 2.0);          // 实部(cos(theta/2))
  double angular_delta_sin = sin(angular_delta_mag / 2.0);
  Eigen::Quaterniond dq(angular_delta_cos,
                        angular_delta_sin * angular_delta_dir.x(),  // 虚部(sin(theta/2) * u)
                        angular_delta_sin * angular_delta_dir.y(),
                        angular_delta_sin * angular_delta_dir.z());
  Eigen::Quaterniond q(pose_.block<3, 3>(0, 0));

  // 更新姿态:
  q = q * dq;

  // 更新前一帧的姿态和当前帧的姿态:
  R_prev = pose_.block<3, 3>(0, 0);
  pose_.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  R_curr = pose_.block<3, 3>(0, 0);
}

/**
 * @brief  获取速度
 * @param  index_curr, current imu measurement buffer index
 * @param  index_prev, previous imu measurement buffer index
 * @param  R_curr, corresponding orientation of current imu measurement
 * @param  R_prev, corresponding orientation of previous imu measurement
 * @param  velocity_delta, velocity delta output
 * @param  linear_acc_mid, mid-value unbiased linear acc
 * @return true if success false otherwise
 */
bool ErrorStateKalmanFilter::GetVelocityDelta(
    const size_t index_curr, const size_t index_prev,
    const Eigen::Matrix3d &R_curr, const Eigen::Matrix3d &R_prev, double &T,
    Eigen::Vector3d &velocity_delta, Eigen::Vector3d &linear_acc_mid) {
  
  // 判断索引有效性
  if (index_curr <= index_prev || imu_data_buff_.size() <= index_curr) {
    return false;
  }

  // 提取前一帧和当前帧的imu数据
  const IMUData &imu_data_curr = imu_data_buff_.at(index_curr);
  const IMUData &imu_data_prev = imu_data_buff_.at(index_prev);

  // deltat
  T = imu_data_curr.time - imu_data_prev.time;

  // 当前帧和前一帧的无偏线加速度
  Eigen::Vector3d linear_acc_curr = Eigen::Vector3d(
      imu_data_curr.linear_acceleration.x, imu_data_curr.linear_acceleration.y,
      imu_data_curr.linear_acceleration.z);
  linear_acc_curr = GetUnbiasedLinearAcc(linear_acc_curr, R_curr);
  Eigen::Vector3d linear_acc_prev = Eigen::Vector3d(
      imu_data_prev.linear_acceleration.x, imu_data_prev.linear_acceleration.y,
      imu_data_prev.linear_acceleration.z);
  linear_acc_prev = GetUnbiasedLinearAcc(linear_acc_prev, R_prev);

  // 获取当前帧和前一帧的线加速度均值:
  linear_acc_mid = 0.5 * (linear_acc_curr + linear_acc_prev);

  // 速度增量
  velocity_delta = T * linear_acc_mid;

  return true;
}

/**
 * @brief  更新位置
 * @param  T, timestamp delta
 * @param  velocity_delta, effective velocity change
 * @return void
 */
void ErrorStateKalmanFilter::UpdatePosition(
    const double &T, const Eigen::Vector3d &velocity_delta) {
  pose_.block<3, 1>(0, 3) += T * vel_ + 0.5 * T * velocity_delta; // vot + 1/2 at^2
  vel_ += velocity_delta;
}

/**
 * @brief  更新IMU里程计的估计值（欧拉积分过程）
 * @param  linear_acc_mid, output mid-value unbiased linear acc
 * @return void
 */
void ErrorStateKalmanFilter::UpdateOdomEstimation(
    Eigen::Vector3d &linear_acc_mid, Eigen::Vector3d &angular_vel_mid) {

  // step1 获取旋转向量:
  Eigen::Vector3d angular_delta; 
  GetAngularDelta(1, 0, angular_delta, angular_vel_mid);

  // step2 更新旋转姿态:
  Eigen::Matrix3d R_curr, R_prev;
  UpdateOrientation(angular_delta, R_curr, R_prev);

  // step3 更新速度:
  double T;
  Eigen::Vector3d velocity_delta;
  // 获取速度增量
  GetVelocityDelta(
      1, 0, 
      R_curr, R_prev, 
      T, velocity_delta, linear_acc_mid
  );

  // 更新位置:
  UpdatePosition(T, velocity_delta);
}

/**
 * @brief  设置噪声方程（也就是F和B）
 * @param  C_nb, rotation matrix, body frame -> navigation frame
 * @param  f_n, accel measurement in navigation frame
 * @return void
 */
void ErrorStateKalmanFilter::SetProcessEquation(const Eigen::Matrix3d &C_nb,
                                                const Eigen::Vector3d &f_n,
                                                const Eigen::Vector3d &w_b) {
  // 设置噪声方程
  // a. set process equation for delta vel:
  F_.block<3, 3>(kIndexErrorVel, kIndexErrorOri) = -C_nb*Sophus::SO3d::hat(f_n).matrix();     // -R_t[a_t]x

  F_.block<3, 3>(kIndexErrorVel, kIndexErrorAccel) = -C_nb;                                   // -R_t

  F_.block<3, 3>(kIndexErrorOri, kIndexErrorOri) = -Sophus::SO3d::hat(w_b).matrix();          // -[omega_t]x

  // b. set process equation for delta ori:
  B_.block<3, 3>(kIndexErrorVel, 0) = C_nb;                                                   // R_t
}

/**
 * @brief  更新过程方程
 * @param  imu_data, input IMU measurement
 * @param  T, output time delta
 * @return void
 */
void ErrorStateKalmanFilter::UpdateProcessEquation(
    const Eigen::Vector3d &linear_acc_mid,
    const Eigen::Vector3d &angular_vel_mid) {
  // 设置线性化点:
  Eigen::Matrix3d C_nb = pose_.block<3, 3>(0, 0);   // R_t
  Eigen::Vector3d f_n = linear_acc_mid + g_;        // a_t
  Eigen::Vector3d w_b = angular_vel_mid;            // omega_t

  // set process equation:
  SetProcessEquation(C_nb, f_n, w_b);
}

/**
 * @brief  update error estimation
 * @param  linear_acc_mid, input mid-value unbiased linear acc
 * @return void
 */
void ErrorStateKalmanFilter::UpdateErrorEstimation(
    const double &T, const Eigen::Vector3d &linear_acc_mid,
    const Eigen::Vector3d &angular_vel_mid) {
  static MatrixF F_1st;
  static MatrixF F_2nd;
  // 更新过程方程，也就是设置F和B:
  UpdateProcessEquation(linear_acc_mid, angular_vel_mid);

  // 对F和B进行离散化:
  F_1st = T*F_;
  F_2nd = 0.5*T*F_*F_1st;
  // 二阶泰勒近似:
  MatrixF F = MatrixF::Identity() + F_1st + F_2nd;

  MatrixB B;
  B.block<9, 6>(0, 0) = T*B_.block<9, 6>(0, 0);       // 左上角

  B.block<6, 6>(9, 6) = sqrt(T)*B_.block<6, 6>(9, 6); // 右下角

  // 更新误差的预测值（这里其实永远是0）
  X_ = F*X_; // fix this

  // 更新协方差矩阵的预测值
  P_ = F*P_*F.transpose() + B*Q_*B.transpose(); // fix this
}

/**
 * @brief  correct error estimation using pose measurement
 * @param  T_nb, input pose measurement
 * @return void
 */
void ErrorStateKalmanFilter::CorrectErrorEstimationPose(
    const Eigen::Matrix4d &T_nb, Eigen::VectorXd &Y, Eigen::MatrixXd &G,
    Eigen::MatrixXd &K) {
  
  // 设置观测值:
  Eigen::Vector3d P_nn_obs = pose_.block<3,1>(0, 3) - T_nb.block<3,1>(0,3);            // 位置
  Eigen::Matrix3d C_nn_obs = T_nb.block<3,3>(0, 0).transpose()*pose_.block<3,3>(0, 0); // 姿态

  YPose_.block<3, 1>(0, 0) = P_nn_obs;
  YPose_.block<3, 1>(3, 0) = Sophus::SO3d::vee(C_nn_obs - Eigen::Matrix3d::Identity());

  // 测量值
  Y = YPose_;
  
  // 测量矩阵
  G = GPose_;

  // 计算卡尔曼增益:
  MatrixRPose R = RPose_; // fix this
  MatrixCPose C = CPose_;
  K = P_*G.transpose()*(G*P_*G.transpose()+C*R*C.transpose()).inverse();            
}

/**
 * @brief  矫正误差的估计值
 * @param  measurement_type, measurement type
 * @param  measurement, input measurement
 * @return void
 */
void ErrorStateKalmanFilter::CorrectErrorEstimation(
    const MeasurementType &measurement_type, const Measurement &measurement) {
  //
  // TODO: understand ESKF correct workflow
  //
  Eigen::VectorXd Y;
  Eigen::MatrixXd G, K;
  switch (measurement_type) {
  case MeasurementType::POSE:
    CorrectErrorEstimationPose(measurement.T_nb, Y, G, K);
    break;
  default:
    break;
  }

  // 计算后验误差估计和协方差矩阵
  P_ = (MatrixP::Identity() - K*G)*P_; 
  X_ = X_+K*(Y-G*X_); 
  
}

/**
 * @brief  去除误差
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::EliminateError(void) {
  //
  // TODO: correct state estimation using the state of ESKF
  //
  // a. 位置:
  pose_.block<3, 1>(0, 3) -= X_.block<3, 1>(kIndexErrorPos, 0); 
  // b. 速度:
  vel_ -= X_.block<3, 1>(kIndexErrorVel, 0);  
  // c. 姿态（这个公式与ppt稍有不同）:
  Eigen::Matrix3d C_nn = Sophus::SO3d::exp(X_.block<3, 1>(kIndexErrorOri, 0)).matrix();
  pose_.block<3, 3>(0, 0) = pose_.block<3, 3>(0, 0)*C_nn.transpose(); 

  // d. gyro bias:
  if (IsCovStable(kIndexErrorGyro)) {
    gyro_bias_ -= X_.block<3, 1>(kIndexErrorGyro, 0);
  }

  // e. accel bias:
  if (IsCovStable(kIndexErrorAccel)) {
    accl_bias_ -= X_.block<3, 1>(kIndexErrorAccel, 0);
  }
}

/**
 * @brief  is covariance stable
 * @param  INDEX_OFSET, state index offset
 * @param  THRESH, covariance threshold, defaults to 1.0e-5
 * @return void
 */
bool ErrorStateKalmanFilter::IsCovStable(const int INDEX_OFSET,
                                         const double THRESH) {
  for (int i = 0; i < 3; ++i) {
    if (P_(INDEX_OFSET + i, INDEX_OFSET + i) > THRESH) {
      return false;
    }
  }

  return true;
}

/**
 * @brief  reset filter state
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::ResetState(void) {
  // reset current state:
  X_ = VectorX::Zero();
}

/**
 * @brief  reset filter covariance
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::ResetCovariance(void) {
  P_ = MatrixP::Zero();

  P_.block<3, 3>(kIndexErrorPos, kIndexErrorPos) =
      COV.PRIOR.POSI * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorVel, kIndexErrorVel) =
      COV.PRIOR.VEL * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorOri, kIndexErrorOri) =
      COV.PRIOR.ORI * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorGyro, kIndexErrorGyro) =
      COV.PRIOR.EPSILON * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorAccel, kIndexErrorAccel) =
      COV.PRIOR.DELTA * Eigen::Matrix3d::Identity();
}

/**
 * @brief  get Q analysis for pose measurement
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::GetQPose(Eigen::MatrixXd &Q, Eigen::VectorXd &Y) {
  // build observability matrix for position measurement:
  Y = Eigen::VectorXd::Zero(kDimState * kDimMeasurementPose);
  Y.block<kDimMeasurementPose, 1>(0, 0) = YPose_;
  for (int i = 1; i < kDimState; ++i) {
    QPose_.block<kDimMeasurementPose, kDimState>(i * kDimMeasurementPose, 0) =
        (QPose_.block<kDimMeasurementPose, kDimState>(
             (i - 1) * kDimMeasurementPose, 0) *
         F_);

    Y.block<kDimMeasurementPose, 1>(i * kDimMeasurementPose, 0) = YPose_;
  }

  Q = QPose_;
}

/**
 * @brief  update observability analysis
 * @param  measurement_type, measurement type
 * @return void
 */
void ErrorStateKalmanFilter::UpdateObservabilityAnalysis(
    const double &time, const MeasurementType &measurement_type) {
  // get Q:
  Eigen::MatrixXd Q;
  Eigen::VectorXd Y;
  switch (measurement_type) {
  case MeasurementType::POSE:
    GetQPose(Q, Y);
    break;
  default:
    break;
  }

  observability.time_.push_back(time);
  observability.Q_.push_back(Q);
  observability.Y_.push_back(Y);
}

/**
 * @brief  save observability analysis to persistent storage
 * @param  measurement_type, measurement type
 * @return void
 */
bool ErrorStateKalmanFilter::SaveObservabilityAnalysis(
    const MeasurementType &measurement_type) {
  // get fusion strategy:
  std::string type;
  switch (measurement_type) {
  case MeasurementType::POSE:
    type = std::string("pose");
    break;
  case MeasurementType::POSE_VEL:
    type = std::string("pose_velocity");
    break;
  case MeasurementType::POSI:
    type = std::string("position");
    break;
  case MeasurementType::POSI_VEL:
    type = std::string("position_velocity");
    break;
  default:
    return false;
    break;
  }

  // build Q_so:
  const int N = observability.Q_.at(0).rows();

  std::vector<std::vector<double>> q_data, q_so_data;

  Eigen::MatrixXd Qso(observability.Q_.size() * N, kDimState);
  Eigen::VectorXd Yso(observability.Y_.size() * N);

  for (size_t i = 0; i < observability.Q_.size(); ++i) {
    const double &time = observability.time_.at(i);

    const Eigen::MatrixXd &Q = observability.Q_.at(i);
    const Eigen::VectorXd &Y = observability.Y_.at(i);

    Qso.block(i * N, 0, N, kDimState) = Q;
    Yso.block(i * N, 0, N, 1) = Y;

    KalmanFilter::AnalyzeQ(kDimState, time, Q, Y, q_data);

    if (0 < i && (0 == i % 10)) {
      KalmanFilter::AnalyzeQ(kDimState, observability.time_.at(i - 5),
                             Qso.block((i - 10), 0, 10 * N, kDimState),
                             Yso.block((i - 10), 0, 10 * N, 1), q_so_data);
    }
  }

  std::string q_data_csv =
      WORK_SPACE_PATH + "/slam_data/observability/" + type + ".csv";
  std::string q_so_data_csv =
      WORK_SPACE_PATH + "/slam_data/observability/" + type + "_som.csv";

  KalmanFilter::WriteAsCSV(kDimState, q_data, q_data_csv);
  KalmanFilter::WriteAsCSV(kDimState, q_so_data, q_so_data_csv);

  return true;
}

} // namespace lidar_localization