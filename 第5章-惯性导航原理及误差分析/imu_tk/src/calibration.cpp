/* 
 * imu_tk - Inertial Measurement Unit Toolkit
 * 
 *  Copyright (c) 2014, Alberto Pretto <pretto@diag.uniroma1.it>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "imu_tk/calibration.h"
#include "imu_tk/filters.h"
#include "imu_tk/integration.h"
#include "imu_tk/visualization.h"

#include <limits>
#include <iostream>
#include "ceres/ceres.h"

using namespace imu_tk;
using namespace Eigen;
using namespace std;

// 加速度计的残差函数
template <typename _T1> struct MultiPosAccResidual
{
  MultiPosAccResidual( const _T1 &g_mag, const Eigen::Matrix< _T1, 3 , 1> &sample ) :
  g_mag_(g_mag),
  sample_(sample){}
  
  template <typename _T2>
    bool operator() ( const _T2* const params, _T2* residuals ) const
  {
    Eigen::Matrix< _T2, 3 , 1> raw_samp( _T2(sample_(0)), _T2(sample_(1)), _T2(sample_(2)) );
    /* Assume body frame same as accelerometer frame, 
     * so bottom left params in the misalignment matris are set to zero */
    CalibratedTriad_<_T2> calib_triad(_T2(0), _T2(0), _T2(0),
                                     params[0], params[1], params[2], 
                                     params[3], params[4], params[5], 
                                     params[6], params[7], params[8] );
    
    Eigen::Matrix< _T2, 3 , 1> calib_samp = calib_triad.unbiasNormalize( raw_samp );
    residuals[0] = _T2 ( g_mag_ ) - calib_samp.norm();
    return true;
  }
  
  static ceres::CostFunction* Create ( const _T1 &g_mag, const Eigen::Matrix< _T1, 3 , 1> &sample )
  {
    return ( new ceres::AutoDiffCostFunction< MultiPosAccResidual, 1, 9 > (
               new MultiPosAccResidual<_T1>( g_mag, sample ) ) );
  }
  
  const _T1 g_mag_;
  const Eigen::Matrix< _T1, 3 , 1> sample_;
};


template <typename _T1>
class AnalyticMultiPosAccResidual : public ceres::SizedCostFunction<1, 9> {
  public:
    AnalyticMultiPosAccResidual(const _T1 &g_mag, const Eigen::Matrix< _T1, 3 , 1> &sample)
      : g_mag_(g_mag), sample_(sample) {}
    
    virtual ~AnalyticMultiPosAccResidual() {}

    virtual bool Evaluate(double const* const* params, double *residuals, double **jacobians) const {
        const double Txz = params[0][0];
        const double Txy = params[0][1];
        const double Tyx = params[0][2];
        const double Kx = params[0][3];
        const double Ky = params[0][4];
        const double Kz = params[0][5];
        const double bx = params[0][6];
        const double by = params[0][7];
        const double bz = params[0][8];

        // 下三角模型
        // 安装误差矩阵
        Eigen::Matrix<double, 3, 3> T;
        T << 1   , 0   , 0, 
             Txz , 1   , 0, 
             -Txy, Tyx , 1;
        
        // 刻度误差系数矩阵
        Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
        K(0, 0) = Kx;
        K(1, 1) = Ky;
        K(2, 2) = Kz;

        // 偏差向量
        Eigen::Vector3d bias(bx, by, bz);
        
        Eigen::Matrix<double,3,1 > sample(double(sample_(0)),double(sample_(1)),double(sample_(2)));

        Eigen::Vector3d calib_samp = T * K * (sample.col(0) - bias);

        residuals[0] = g_mag_ - calib_samp.norm();

        if (jacobians != nullptr) {
            if (jacobians[0] != nullptr) {
                Eigen::Vector3d x_xnorm = calib_samp / (calib_samp.norm());
                // 测量值减去bias
                Eigen::Vector3d v = sample.col(0) - bias;
                // calib_samp 对参数的偏导
                Eigen::Matrix<double, 3, 9> J_theta = Eigen::Matrix<double, 3, 9>::Zero();
                J_theta(0, 3) = v(0);
                J_theta(0, 6) = -Kx;
                J_theta(1, 0) = Kx * v(0);
                J_theta(1, 3) = Txz * v(0);
                J_theta(1, 4) = v(1);
                J_theta(1, 6) = -Txz * Kx;
                J_theta(1, 7) = -Ky;
                J_theta(2, 1) = -Kx * v(0);
                J_theta(2, 2) = Ky * v(1);
                J_theta(2, 3) = -Txy * v(0);
                J_theta(2, 4) = Tyx * v(1);
                J_theta(2, 5) = v(2);
                J_theta(2, 6) = Txy * Kx;
                J_theta(2, 7) = -Tyx * Ky;
                J_theta(2, 8) = -Kz;
                Eigen::Matrix<double, 1, 9> Jaco = - x_xnorm.transpose() * J_theta;
                jacobians[0][0] = Jaco(0, 0);
                jacobians[0][1] = Jaco(0, 1);
                jacobians[0][2] = Jaco(0, 2);
                jacobians[0][3] = Jaco(0, 3);
                jacobians[0][4] = Jaco(0, 4);
                jacobians[0][5] = Jaco(0, 5);
                jacobians[0][6] = Jaco(0, 6);
                jacobians[0][7] = Jaco(0, 7);
                jacobians[0][8] = Jaco(0, 8);
            }
        }
        return true;
    }

  private:
    const _T1 g_mag_;
    const Eigen::Matrix< _T1, 3 , 1> sample_;
};



template <typename _T1> struct MultiPosGyroResidual
{
  MultiPosGyroResidual( const Eigen::Matrix< _T1, 3 , 1> &g_versor_pos0, 
                        const Eigen::Matrix< _T1, 3 , 1> &g_versor_pos1,
                        const std::vector< TriadData_<_T1> > &gyro_samples, 
                        const DataInterval &gyro_interval_pos01, 
                        _T1 dt, bool optimize_bias) :

  g_versor_pos0_(g_versor_pos0), 
  g_versor_pos1_(g_versor_pos1),
  gyro_samples_(gyro_samples),
  interval_pos01_(gyro_interval_pos01),
  dt_(dt), optimize_bias_(optimize_bias){}
  
  template <typename _T2>
    bool operator() ( const _T2* const params, _T2* residuals ) const
  {
    CalibratedTriad_<_T2> calib_triad( params[0], params[1], params[2], 
                                      params[3], params[4], params[5], 
                                      params[6], params[7], params[8],
                                      optimize_bias_?params[9]:_T2(0), 
                                      optimize_bias_?params[10]:_T2(0), 
                                      optimize_bias_?params[11]:_T2(0) );

    std::vector< TriadData_<_T2> > calib_gyro_samples;
    calib_gyro_samples.reserve( interval_pos01_.end_idx - interval_pos01_.start_idx + 1 );
    
    for( int i = interval_pos01_.start_idx; i <= interval_pos01_.end_idx; i++ )
      calib_gyro_samples.push_back( TriadData_<_T2>( calib_triad.unbiasNormalize( gyro_samples_[i] ) ) );
    
    Eigen::Matrix< _T2, 3 , 3> rot_mat;
    integrateGyroInterval( calib_gyro_samples, rot_mat, _T2(dt_) );
    
    Eigen::Matrix< _T2, 3 , 1> diff = rot_mat.transpose()*g_versor_pos0_.template cast<_T2>() -
                                      g_versor_pos1_.template cast<_T2>();
    
    residuals[0] = diff(0);
    residuals[1] = diff(1);
    residuals[2] = diff(2);
    
    return true;
  }
  
  static ceres::CostFunction* Create ( const Eigen::Matrix< _T1, 3 , 1> &g_versor_pos0, 
                                       const Eigen::Matrix< _T1, 3 , 1> &g_versor_pos1,
                                       const std::vector< TriadData_<_T1> > &gyro_samples, 
                                       const DataInterval &gyro_interval_pos01, 
                                       _T1 dt, bool optimize_bias )
  {
    if( optimize_bias )
      return ( new ceres::AutoDiffCostFunction< MultiPosGyroResidual, 3, 12 > (
                new MultiPosGyroResidual( g_versor_pos0, g_versor_pos1, gyro_samples, 
                                          gyro_interval_pos01, dt, optimize_bias ) ) );
    else
      return ( new ceres::AutoDiffCostFunction< MultiPosGyroResidual, 3, 9 > (
                new MultiPosGyroResidual( g_versor_pos0, g_versor_pos1, gyro_samples, 
                                          gyro_interval_pos01, dt, optimize_bias ) ) );
  }
  
  const Eigen::Matrix< _T1, 3 , 1> g_versor_pos0_, g_versor_pos1_;
  const std::vector< TriadData_<_T1> > gyro_samples_;
  const DataInterval interval_pos01_;
  const _T1 dt_;
  const bool optimize_bias_;
};

template <typename _T>
  MultiPosCalibration_<_T>::MultiPosCalibration_() :
  g_mag_(9.81),
  min_num_intervals_(12),
  init_interval_duration_(_T(30.0)),
  interval_n_samples_(100),
  acc_use_means_(false),
  gyro_dt_(-1.0),
  optimize_gyro_bias_(false),
  verbose_output_(false){}

// 首先标定加速度计
template <typename _T>
  bool MultiPosCalibration_<_T>::calibrateAcc ( const std::vector< TriadData_<_T> >& acc_samples )
{
  cout<<"Accelerometers calibration: calibrating..."<<endl;
  
  // 保存最优的采样时间段以及相应的加速度计和陀螺仪的采样值
  min_cost_static_intervals_.clear();
  calib_acc_samples_.clear();
  calib_gyro_samples_.clear();
  
  // 采样次数
  int n_samps = acc_samples.size();
  
  // 提取前50秒内的加速度计数据
  DataInterval init_static_interval = DataInterval::initialInterval( acc_samples, init_interval_duration_ );

  // 计算前50秒内的加速度计的方差
  Eigen::Matrix<_T, 3, 1> acc_variance = dataVariance( acc_samples, init_static_interval );

  // 方差的模
  _T norm_th = acc_variance.norm();

  _T min_cost = std::numeric_limits< _T >::max();
  int min_cost_th = -1;

  // 保存最优的参数
  std::vector< double > min_cost_calib_params;
  
  // 使用不同的阈值进行多次优化求解，取cost最小的作为最终结果
  for (int th_mult = 2; th_mult <= 10; th_mult++)
  {
    // 保存静态时间间隔和相应的数据
    std::vector< imu_tk::DataInterval > static_intervals;
    std::vector< imu_tk::TriadData_<_T> > static_samples;

    // 加速度计的参数
    std::vector< double > acc_calib_params(9);
    
    // 安装误差
    // 此处采用下三角
    acc_calib_params[0] = init_acc_calib_.misXZ();
    acc_calib_params[1] = init_acc_calib_.misXY();
    acc_calib_params[2] = init_acc_calib_.misYX();
    
    // 刻度系数误差
    acc_calib_params[3] = init_acc_calib_.scaleX();
    acc_calib_params[4] = init_acc_calib_.scaleY();
    acc_calib_params[5] = init_acc_calib_.scaleZ();
    
    // 零偏
    acc_calib_params[6] = init_acc_calib_.biasX();
    acc_calib_params[7] = init_acc_calib_.biasY();
    acc_calib_params[8] = init_acc_calib_.biasZ();
    
    std::vector< DataInterval > extracted_intervals;
    // 提取加速度计稳定的时间间隔(使用不同的方差阈值来判断是否为静止段)
    staticIntervalsDetector ( acc_samples, th_mult*norm_th, static_intervals );

    extractIntervalsSamples ( acc_samples, static_intervals, 
                              static_samples, extracted_intervals,
                              interval_n_samples_, acc_use_means_ );
    
    if(verbose_output_)
      cout<<"Accelerometers calibration: extracted "<<extracted_intervals.size()
          <<" intervals using threshold multiplier "<<th_mult<<" -> ";
    
    // TODO Perform here a quality test
    // 检查提取到的稳定间隔个数是否大于求解方程所需要的个数(此处为12)
    if( extracted_intervals.size() < min_num_intervals_)
    {
      if( verbose_output_) cout<<"Not enough intervals, calibration is not possible"<<endl;
      continue;
    }
    
    // 开始矫正
    if( verbose_output_) cout<<"Trying calibrate... "<<endl;
    
    // 构建残差模型
    ceres::Problem problem;
    for( int i = 0; i < static_samples.size(); i++)
    {
      // ceres::CostFunction* cost_function =
      //   MultiPosAccResidual<_T>::Create ( g_mag_, static_samples[i].data() );
      ceres::CostFunction* cost_function = new AnalyticMultiPosAccResidual<_T>(g_mag_, static_samples[i].data());
      problem.AddResidualBlock ( cost_function, NULL /* squared loss */, acc_calib_params.data() ); 
    }
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = verbose_output_;

    ceres::Solver::Summary summary;
    ceres::Solve ( options, &problem, &summary );
    if( summary.final_cost < min_cost)
    {
      min_cost = summary.final_cost;
      min_cost_th = th_mult;
      min_cost_static_intervals_ = static_intervals;
      min_cost_calib_params = acc_calib_params;
    }
    cout<<"residual "<<summary.final_cost<<endl;
  }
  
  if( min_cost_th < 0 )
  {
    if(verbose_output_) 
      cout<<"Accelerometers calibration: Can't obtain any calibratin with the current dataset"<<endl;
    return false;
  }

  acc_calib_ = CalibratedTriad_<_T>( 0,0,0, 
                                     min_cost_calib_params[0],
                                     min_cost_calib_params[1],
                                     min_cost_calib_params[2],
                                     min_cost_calib_params[3],
                                     min_cost_calib_params[4],
                                     min_cost_calib_params[5],
                                     min_cost_calib_params[6],
                                     min_cost_calib_params[7],
                                     min_cost_calib_params[8] );
  
  calib_acc_samples_.reserve(n_samps);
  
  // Calibrate the input accelerometer data with the obtained calibration
  // 使用标定后的内参去还原数据
  for( int i = 0; i < n_samps; i++)
    calib_acc_samples_.push_back( acc_calib_.unbiasNormalize( acc_samples[i]) );
  
  if(verbose_output_) 
  {
    Plot plot;
    plot.plotIntervals( calib_acc_samples_, min_cost_static_intervals_);
    
    cout<<"Accelerometers calibration: Better calibration obtained using threshold multiplier "<<min_cost_th
        <<" with residual "<<min_cost<<endl
        <<acc_calib_<<endl
        <<"Accelerometers calibration: inverse scale factors:"<<endl
        <<1.0/acc_calib_.scaleX()<<endl
        <<1.0/acc_calib_.scaleY()<<endl
        <<1.0/acc_calib_.scaleZ()<<endl;
        
    waitForKey();
  }
  
  return true;
    
}

// 矫正加速度计和陀螺仪的主函数
template <typename _T> 
  bool MultiPosCalibration_<_T>::calibrateAccGyro ( const vector< TriadData_<_T> >& acc_samples, 
                                                   const vector< TriadData_<_T> >& gyro_samples )
{
  // 标定加速度计
  if( !calibrateAcc( acc_samples ) )
    return false;
  
  cout<<"Gyroscopes calibration: calibrating..."<<endl;
  
  std::vector< TriadData_<_T> > static_acc_means;
  std::vector< DataInterval > extracted_intervals;
  extractIntervalsSamples ( calib_acc_samples_, min_cost_static_intervals_, 
                            static_acc_means, extracted_intervals,
                            interval_n_samples_, true );
  
  int n_static_pos = static_acc_means.size(), n_samps = gyro_samples.size();
  
  // Compute the gyroscopes biases in the (static) initialization interval
  DataInterval init_static_interval = DataInterval::initialInterval( gyro_samples, init_interval_duration_ );
  Eigen::Matrix<_T, 3, 1> gyro_bias = dataMean( gyro_samples, init_static_interval );
  
  gyro_calib_ = CalibratedTriad_<_T>(0, 0, 0, 0, 0, 0, 
                                    1.0, 1.0, 1.0, 
                                    gyro_bias(0), gyro_bias(1), gyro_bias(2) );
  

  // calib_gyro_samples_ already cleared in calibrateAcc()
  calib_gyro_samples_.reserve(n_samps);
  // Remove the bias
  for( int i = 0; i < n_samps; i++ )
    calib_gyro_samples_.push_back(gyro_calib_.unbias(gyro_samples[i]));
  
  std::vector< double > gyro_calib_params(12);

  gyro_calib_params[0] = init_gyro_calib_.misYZ();
  gyro_calib_params[1] = init_gyro_calib_.misZY();
  gyro_calib_params[2] = init_gyro_calib_.misZX();
  gyro_calib_params[3] = init_gyro_calib_.misXZ();
  gyro_calib_params[4] = init_gyro_calib_.misXY();
  gyro_calib_params[5] = init_gyro_calib_.misYX();
  
  gyro_calib_params[6] = init_gyro_calib_.scaleX();
  gyro_calib_params[7] = init_gyro_calib_.scaleY();
  gyro_calib_params[8] = init_gyro_calib_.scaleZ();
  
  // Bias has been estimated and removed in the initialization period
  gyro_calib_params[9] = 0.0;
  gyro_calib_params[10] = 0.0;
  gyro_calib_params[11] = 0.0;
  
  ceres::Problem problem;
      
  for( int i = 0, t_idx = 0; i < n_static_pos - 1; i++ )
  {
    Eigen::Matrix<_T, 3, 1> g_versor_pos0 = static_acc_means[i].data(),
                            g_versor_pos1 = static_acc_means[i + 1].data();
                               
    g_versor_pos0 /= g_versor_pos0.norm();                           
    g_versor_pos1 /= g_versor_pos1.norm();
    
    int gyro_idx0 = -1, gyro_idx1 = -1;
    _T ts0 = calib_acc_samples_[extracted_intervals[i].end_idx].timestamp(), 
       ts1 = calib_acc_samples_[extracted_intervals[i + 1].start_idx].timestamp();
     
    // Assume monotone signal time
    for( ; t_idx < n_samps; t_idx++ )
    {
      if( gyro_idx0 < 0 )
      {
        if( calib_gyro_samples_[t_idx].timestamp() >= ts0 )
          gyro_idx0 = t_idx;
      }
      else
      {
        if( calib_gyro_samples_[t_idx].timestamp() >= ts1 )
        {
          gyro_idx1 = t_idx - 1;
          break;
        }
      }
    }
    
//     cout<<"from "<<calib_gyro_samples_[gyro_idx0].timestamp()<<" to "
//         <<calib_gyro_samples_[gyro_idx1].timestamp()
//         <<" v0 : "<< g_versor_pos0(0)<<" "<< g_versor_pos0(1)<<" "<< g_versor_pos0(2)
//         <<" v1 : "<< g_versor_pos1(0)<<" "<< g_versor_pos1(1)<<" "<< g_versor_pos1(2)<<endl;
    
    DataInterval gyro_interval(gyro_idx0, gyro_idx1);
    
    ceres::CostFunction* cost_function =
      MultiPosGyroResidual<_T>::Create ( g_versor_pos0, g_versor_pos1, calib_gyro_samples_,
                                         gyro_interval, gyro_dt_, optimize_gyro_bias_ );

    problem.AddResidualBlock ( cost_function, NULL /* squared loss */, gyro_calib_params.data() ); 
      
  }
  
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = verbose_output_;

  ceres::Solver::Summary summary;

  ceres::Solve ( options, &problem, &summary );
  gyro_calib_ = CalibratedTriad_<_T>( gyro_calib_params[0],
                                     gyro_calib_params[1],
                                     gyro_calib_params[2],
                                     gyro_calib_params[3],
                                     gyro_calib_params[4],
                                     gyro_calib_params[5],
                                     gyro_calib_params[6],
                                     gyro_calib_params[7],
                                     gyro_calib_params[8],
                                     gyro_bias(0) + gyro_calib_params[9],
                                     gyro_bias(1) + gyro_calib_params[10],
                                     gyro_bias(2) + gyro_calib_params[11]);                            

  // Calibrate the input gyroscopes data with the obtained calibration
  for( int i = 0; i < n_samps; i++)
    calib_gyro_samples_.push_back( gyro_calib_.unbiasNormalize( gyro_samples[i]) );
  
  if(verbose_output_) 
  {
    
    cout<<summary.FullReport()<<endl;
    cout<<"Gyroscopes calibration: residual "<<summary.final_cost<<endl
        <<gyro_calib_<<endl
        <<"Gyroscopes calibration: inverse scale factors:"<<endl
        <<1.0/gyro_calib_.scaleX()<<endl
        <<1.0/gyro_calib_.scaleY()<<endl
        <<1.0/gyro_calib_.scaleZ()<<endl;
  }
  
  return true;
}

template class MultiPosCalibration_<double>;
template class MultiPosCalibration_<float>;