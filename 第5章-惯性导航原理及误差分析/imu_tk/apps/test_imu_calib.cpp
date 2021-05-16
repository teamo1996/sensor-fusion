#include <iostream>

#include "imu_tk/io_utils.h"
#include "imu_tk/calibration.h"
#include "imu_tk/filters.h"
#include "imu_tk/integration.h"
#include "imu_tk/visualization.h"

using namespace std;
using namespace imu_tk;
using namespace Eigen;

int main(int argc, char** argv)
{
  if( argc < 3 )
    return -1;

  vector< TriadData > acc_data, gyro_data;
  
  // 读取加速度计和陀螺仪信息
  cout<<"Importing IMU data from the Matlab matrix file : "<< argv[1]<<endl;  
  importAsciiData( argv[1], acc_data, imu_tk::TIMESTAMP_UNIT_SEC );
  cout<<"Importing IMU data from the Matlab matrix file : "<< argv[2]<<endl;  
  importAsciiData( argv[2], gyro_data, imu_tk::TIMESTAMP_UNIT_SEC  );
  
  // 构造初始的内参类
  CalibratedTriad init_acc_calib, init_gyro_calib;
  init_acc_calib.setBias( Vector3d(32768, 32768, 32768) );
  init_gyro_calib.setScale( Vector3d(1.0/6258.0, 1.0/6258.0, 1.0/6258.0) );
  
  // 构造标定类
  MultiPosCalibration mp_calib;
    
  mp_calib.setInitStaticIntervalDuration(50.0);             // 设置初始的静止时间为50s

  mp_calib.setInitAccCalibration( init_acc_calib );         // 设置加速度计内参的初始猜测值
  mp_calib.setInitGyroCalibration( init_gyro_calib );       // 设置陀螺仪内参的初始猜测值
  mp_calib.setGravityMagnitude(9.81744);                    // 设置重力加速度的幅值
  mp_calib.enableVerboseOutput(true);                       // 设置输出信息为真
  mp_calib.enableAccUseMeans(false);                        // 设置为使用每一段的所有采样数据，而不是每一段的均值
  //mp_calib.setGyroDataPeriod(0.01);
  mp_calib.calibrateAccGyro(acc_data, gyro_data );          // 开始标定
  mp_calib.getAccCalib().save("test_imu_acc.calib");
  mp_calib.getGyroCalib().save("test_imu_gyro.calib");
  
//   for( int i = 0; i < acc_data.size(); i++)
//   {
//     cout<<acc_data[i].timestamp()<<" "
  //         <<acc_data[i].x()<<" "<<acc_data[i].y()<<" "<<acc_data[i].z()<<" "
  //         <<gyro_data[i].x()<<" "<<gyro_data[i].y()<<" "<<gyro_data[i].z()<<endl;
//   }
//   cout<<"Read "<<acc_data.size()<<" tuples"<<endl;
  
  return 0;
}