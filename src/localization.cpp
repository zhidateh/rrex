#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <rrex/FusedPose.h>
#include <rrex/Encoder.h>
#include <iostream>
#include <chrono>
#include <vector>
#include <numeric>
#include <math.h>
#include <Eigen/Dense>

#include "kalman.hpp"

struct ImuData{
  double ax,ay,az, hdg, pitch, roll;
};

struct EncoderData{
  double x, y, hdg, vel, vel_l, vel_r; 
};


class Localization
{
public:
  Localization()
  {
    //Topic you want to publish
    pub_ = n_.advertise<rrex::FusedPose>("/loc_node/fusedpose", 1);

    //Topic you want to subscribe
    sub_imu = n_.subscribe("/imu_node/imu", 1000, &Localization::callback_imu, this);
    sub_encoder = n_.subscribe("/encoder_node/encoder", 1000, &Localization::callback_encoder, this);
  
    initKalmanFilter();
    
  }

  void initKalmanFilter()
  {
    int n = 3; // Number of states
    int m = 1; // Number of measurements
    int c = 1; // Number of control inputs
    
    A.resize(n, n); // System dynamics matrix
    B.resize(n, c); // Input control matrix
    C.resize(m, n); // Output matrix
    Q.resize(n, n); // Process noise covariance
    R.resize(m, m); // Measurement noise covariance
    P.resize(n, n); // Estimate error covariance
  
    y_vl.resize(m);
    y_vr.resize(m);
    y_hdg.resize(m);
    u.resize(c);
    
    A << 1, fps, 0, 0, 1, fps, 0, 0, 1;
    B << 0, 0, 0;
    C << 0, 1, 0;
    Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
    R << 10;
    P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;
 
    kf_vl   = new KalmanFilter(A,B,C,Q,R,P);
    kf_vr   = new KalmanFilter(A,B,C,Q,R,P);
    kf_hdg  = new KalmanFilter(A,B,C,Q,R,P);

    Eigen::VectorXd x0(n);
    x0 << 0.0, 0.0, 0.0;
    kf_vl->init(x0);
    kf_vr->init(x0);
    kf_hdg->init(x0);
    
    //initial state;
    vel_ls.push_back(0.0);
    vel_rs.push_back(0.0);
    hdgs.push_back(0.0);
  
  }


  double roundto3DP(double value)
  {
    return roundf(value * 10000) / 10000;
  }
  
  double normalizeAngle(double angle)
  {

    while (angle > 3.14159) angle -= 2.0 * 3.14159;
    while (angle < -3.14159) angle += 2.0 * 3.14159;
    return roundto3DP(angle);
  }

  ImuData toEuler(const sensor_msgs::Imu::ConstPtr& msg_imu)
  {
    //quaternion 
    double qw = msg_imu ->orientation.w;   
    double qx = msg_imu ->orientation.x;   
    double qy = msg_imu ->orientation.y;   
    double qz = msg_imu ->orientation.z;   
    
    ImuData imuData;

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (qw * qx + qy * qz);
    double cosr_cosp = +1.0 - 2.0 * (qx * qx + qy * qy);
    imuData.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1)
        imuData.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        imuData.pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (qw * qz + qx * qy);
    double cosy_cosp = +1.0 - 2.0 * (qy * qy + qz * qz);  
    imuData.hdg = atan2(siny_cosp, cosy_cosp);

    return imuData;
  }


  void callback_imu(const sensor_msgs::Imu::ConstPtr& msg_imu)
  {
    tf::Quaternion q(msg_imu->orientation.x, msg_imu->orientation.y, msg_imu->orientation.z, msg_imu->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ImuData _imuData = toEuler(msg_imu);

    if(init_step < init_step_target){
      ROS_INFO("Initialising IMU to get initial position, loading status %d / %d", init_step, init_step_target);
  
      ax.push_back(roundto3DP(-msg_imu ->linear_acceleration.x));
      ay.push_back(roundto3DP(msg_imu ->linear_acceleration.y));
      az.push_back(roundto3DP(msg_imu ->linear_acceleration.z));
  
      init_imuData.ax     = roundto3DP(std::accumulate(ax.begin(),ax.end(),0.0)/(double)ax.size()); 
      init_imuData.ay     = roundto3DP(std::accumulate(ay.begin(),ay.end(),0.0)/(double)ay.size());
      init_imuData.az     = roundto3DP(std::accumulate(az.begin(),az.end(),0.0)/(double)az.size());
      
      init_imuData.hdg    = roundto3DP(_imuData.hdg);
      init_imuData.pitch  = roundto3DP(_imuData.pitch);
      init_imuData.roll   = roundto3DP(_imuData.roll);
      init_step++;
      
    }
    else{
    
      imuData.ax = roundto3DP(-msg_imu -> linear_acceleration.x - init_imuData.ax);
      imuData.ay = roundto3DP(msg_imu -> linear_acceleration.y - init_imuData.ay);
      imuData.az = roundto3DP(msg_imu -> linear_acceleration.z - init_imuData.az);

      //imuData.hdg   = roundto3DP(normalizeAngle(_imuData.hdg - init_imuData.hdg));
      //imuData.pitch = roundto3DP(normalizeAngle(_imuData.pitch - init_imuData.pitch));
      //imuData.roll  = roundto3DP(normalizeAngle(_imuData.roll - init_imuData.roll));
      
      imuData.hdg = roundto3DP(normalizeAngle(yaw - init_imuData.hdg));
      imuData.pitch = roundto3DP(normalizeAngle(pitch - init_imuData.pitch));
      imuData.roll = roundto3DP(normalizeAngle(roll - init_imuData.roll));

      
      //ROS_INFO("%s: %f ","YAW 1", yaw);
      //ROS_INFO("%s: %f ","YAW 2", _imuData.hdg);
      //ROS_INFO("-------------------");

      
      
      fusionAlgorithm();
    }

  }

  void callback_encoder(const rrex::Encoder::ConstPtr& msg_encoder)
  {
    encoderData.x       = roundto3DP(msg_encoder	->x);
    encoderData.y       = roundto3DP(msg_encoder	->y);
    encoderData.hdg     = roundto3DP(msg_encoder	->hdg);
    encoderData.vel     = roundto3DP(msg_encoder	->vel);
    encoderData.vel_l   = roundto3DP(msg_encoder	->vel_l);
    encoderData.vel_r   = roundto3DP(msg_encoder	->vel_r);

    if(init_step >= init_step_target) fusionAlgorithm();
  }

  void fusionAlgorithm()
  {
    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    double dt =  (end_time - start_time).count()/1000000000.0;


    vel_ls.push_back(encoderData.vel_l);
    vel_rs.push_back(encoderData.vel_r);
    
    hdgs.push_back(roundf(imuData.hdg*100)/100);

    y_vl << encoderData.vel_l;
    y_vr << encoderData.vel_r;
    y_hdg << encoderData.hdg;
    
    u << 0;
    
    kf_vl   ->predict(u);
    kf_vr   ->predict(u);
    kf_hdg  ->predict(u);
    
    kf_vl   ->update(y_vl);
    kf_vr   ->update(y_vr);
    kf_hdg  ->update(y_hdg);
    

    //50Hz
    if(dt > fps ){
      start_time = std::chrono::steady_clock::now();

      double _vel_l =  roundto3DP(kf_vl->state().transpose()(1));
      double _vel_r =  roundto3DP(kf_vr->state().transpose()(1));
      double _vel = roundto3DP(0.5 * (_vel_l + _vel_r));
      double _hdg = imuData.hdg;//std::accumulate(hdgs.begin(),hdgs.end(),0.0)/(double)hdgs.size(); //roundto3DP(imuData.hdg);//roundto3DP(kf_hdg->state().transpose()(1));
      
      if(!isinf(_vel) && !isinf(_hdg)){
        state_hdg += (_vel_l - _vel_r)*dt/0.48;
        
        state_x += roundto3DP(_vel*cos(state_hdg)*dt);
        state_y += roundto3DP(_vel*sin(state_hdg)*dt);
        
        //state_x += roundto3DP(_vel*cos(_hdg)*dt);
        //state_y += roundto3DP(_vel*sin(_hdg)*dt);
      }
      
      //ROS_INFO("%s: %.3f",                   "State x..  : "  , _vel*cos(_hdg)*dt);

      ROS_INFO("%s: %.3f", "State x  : "  , state_x);
      ROS_INFO("%s: %.3f", "State y  : "  , state_y);
      ROS_INFO("%s: %.3f", "Heading  : "  , state_hdg );
      //ROS_INFO("%s: %.3f", "Heading  : "  , _hdg);
      ROS_INFO("%s: %.3f", "Velocity : "  , _vel);
      ROS_INFO("%s: %.3f", "Left vel : "  , _vel_l);
      ROS_INFO("%s: %.3f", "Right vel: "  , _vel_r);
      ROS_INFO("----------------------------------");
      
        
      rrex::FusedPose fused_pose_msg;
      fused_pose_msg.x      = state_x;
      fused_pose_msg.y      = state_y;
      fused_pose_msg.hdg    = state_hdg;
      fused_pose_msg.vel    = _vel;
      fused_pose_msg.vel_l  = _vel_l;
      fused_pose_msg.vel_r  = _vel_r;

      pub_.publish(fused_pose_msg);
      
      vel_ls.clear();
      vel_rs.clear();
      hdgs.clear();
    } 

  }
  


private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_encoder;
  ros::Subscriber sub_imu;
  
  std::vector<double> ax,ay,az;
  std::vector<double> vel_ls,vel_rs,hdgs;
  int init_step =0;
  int init_step_target = 500;
  
  double fps = 1.0/60.0;
  
  double state_x = 0.0;
  double state_y = 0.0;
  double state_hdg = 0.0;

  ImuData imuData;
  ImuData init_imuData;
  EncoderData encoderData;

  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
  Eigen::MatrixXd P;
  
  KalmanFilter *kf_vl;
  KalmanFilter *kf_vr;
  KalmanFilter *kf_hdg;

  Eigen::VectorXd y_vl,y_vr,y_hdg,u;

  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

};//End of class Localization

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "Localization");

  //Create an object of class Localization that will take care of everything
  Localization LOCObject;
 
  ros::spin();

  return 0;
}
