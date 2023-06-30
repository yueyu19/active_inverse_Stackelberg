// This node generates a trajectory that is parameterized by time and publishes
// a vector of setpoints as time progresses.

#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Time.h"

class TrajectoryGenerator
{
  public:
    TrajectoryGenerator()
    {
      ROS_INFO_STREAM("Starting trajectory generator");

      const auto queue_size = 100;
      pub_ = nh_.advertise<std_msgs::Float64MultiArray>("setpoint", queue_size);
      start_time_sub_ = nh_.subscribe("start_time", queue_size, &TrajectoryGenerator::startTimeCallback, this);
      ts_sub_ = nh_.subscribe("ts/ts", queue_size, &TrajectoryGenerator::tsCallback, this);
      coeffs_x_sub_ = nh_.subscribe("coeffs_x/coeffs_x", queue_size, &TrajectoryGenerator::xCoeffsCallback, this);
      coeffs_y_sub_ = nh_.subscribe("coeffs_y/coeffs_y", queue_size, &TrajectoryGenerator::yCoeffsCallback, this);
    }

    // Updates the setpoint topic with the new setpoint.
    void update()
    {
      if(start_time_.is_zero()) return;
      ros::Duration d = ros::Time::now() - start_time_;
      double t = d.toSec();
      if (t > ts_[t_idx_])
        t_idx_++;
      if (t_idx_ >= ts_.size())
      {
        t_idx_ = 0;
        start_time_ = ros::Time::now();
        return;
      }
      std::vector<double> setpoint = calculateSetpoint(t);
      publishSetpoint(setpoint);
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber start_time_sub_;
    ros::Subscriber ts_sub_;
    ros::Subscriber coeffs_x_sub_;
    ros::Subscriber coeffs_y_sub_;
    ros::Time start_time_;
    std::vector<double> ts_;
    std::vector<double> coeffs_x_;
    std::vector<double> coeffs_y_;
    int t_idx_;

    void startTimeCallback(std_msgs::Time time)
    {
      t_idx_ = 0;
      start_time_ = time.data;
    }

    void tsCallback(std_msgs::Float64MultiArray msg)
    {
      ts_ = msg.data;
    }

    void xCoeffsCallback(std_msgs::Float64MultiArray msg)
    {
      coeffs_x_ = msg.data;
    }

    void yCoeffsCallback(std_msgs::Float64MultiArray msg)
    {
      coeffs_y_ = msg.data;
    }

    // Calculates the setpoint based on the time elapsed.
    std::vector<double> calculateSetpoint(double t)
    {
      int idx = 4*(t_idx_-1);
      
      double x_d = coeffs_x_[idx]*t*t*t + coeffs_x_[idx+1]*t*t + coeffs_x_[idx+2]*t + coeffs_x_[idx+3];
      double x_dot_d = 3*coeffs_x_[idx]*t*t + 2*coeffs_x_[idx+1]*t + coeffs_x_[idx+2];
      double y_d = coeffs_y_[idx]*t*t*t + coeffs_y_[idx+1]*t*t + coeffs_y_[idx+2]*t + coeffs_y_[idx+3];
      double y_dot_d = 3*coeffs_y_[idx]*t*t + 2*coeffs_y_[idx+1]*t + coeffs_y_[idx+2];

      std::vector<double> setpoint{x_d, x_dot_d, y_d, y_dot_d};
      return setpoint;
    }

    // Publishes the setpoint to setpoint.
    void publishSetpoint(std::vector<double> setpoint)
    {
      std_msgs::Float64MultiArray msg;
      msg.data = setpoint;
      pub_.publish(msg);
    }
};

auto main(int argc, char **argv) -> int
{
  // initialize node
  ros::init(argc, argv, "trajectory_generator");

  TrajectoryGenerator tg;

  ros::AsyncSpinner spinner(3);
  spinner.start();

  ros::Rate rate(50.0);

  while (ros::ok())
  {
    tg.update();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}