// This node runs a backstepping-type controller which controls a robot on a
// flat plane.

#include <iostream>
#include <signal.h>
#include <cmath>
#include <X11/Xlib.h>
#include "X11/keysym.h"

#include <ros/ros.h>
#include "std_msgs/Time.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float32.h"
#include "jetracer_racing_ros_msgs/RolloutData.h"
#include <tf/transform_datatypes.h>

// Flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// When ctrl+C is used to stop this node, this function is called. Updates the
// shutdown flag.
void sigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

class Controller
{
  public:
    Controller() : control_lock_(false), started_(false), Kx_(0.0), Ky_(0.0), Kv_(0.0), Ktheta_(0.0)
    {
      ROS_INFO_STREAM("Starting controller");

      nh_.getParam("controller/cycle_rate", cycle_rate_);
      nh_.getParam("controller/Kx", Kx_);
      nh_.getParam("controller/Ky", Ky_);
      nh_.getParam("controller/Kphi", Ktheta_);

      const auto queue_size = 100;
      cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", queue_size);
      data_pub_ = nh_.advertise<bootcamp_07_msgs::RolloutData>("rollout_data", queue_size);
      setpoint_sub_ = nh_.subscribe("setpoint", queue_size, &Controller::setpointCallback, this);
      pose_sub_ = nh_.subscribe("vrpn_client_node/turtlebot/pose", queue_size, &Controller::poseCallback, this);
      start_time_sub_ = nh_.subscribe("start_time", queue_size, &Controller::startTimeCallback, this);
    }

    double cycle_rate(){return cycle_rate_;}

    // Stop the robot, then shutdown the node.
    void shutdown()
    {
      stop_robot();
      ROS_INFO_STREAM("Stopping and exiting..." );
      ros::shutdown();
      std::cout << "Exited." << std::endl;
    }

    // Runs a single control loop of a backstepping-type controller which
    // commands the robot to a desired x, y position using feedback data,
    // feedforward data, and controller gains. 
    void control()
    {
      // return if the controller is not started
      if (!started_)
        return;

      if (space_is_pressed())
      {
        ROS_WARN_STREAM("Spacebar pressed. Stopping robot.");
        stop_robot();
        publishRolloutData();
        ROS_INFO_STREAM("Rollout data published");
        return;
      }

      // set the control lock flag to true so that other member variables are
      // not updated by callbacks while this function is executing.
      control_lock_ = true;

      double u_x = xdot_d_ - Kx_*(x_ - x_d_);
      double u_y = ydot_d_ - Ky_*(y_ - y_d_);
      double v = sqrt(u_x*u_x + u_y*u_y);
      v = clamp(v, -0.22, 0.22);

      double theta_d;
      theta_d = atan2(u_y, u_x);

      if (abs(theta_ - theta_d) > abs(theta_ - (theta_d + 2*PI)))
        theta_d += 2*PI;
      else if (abs(theta_ - theta_d) > abs(theta_ - (theta_d - 2*PI)))
        theta_d -= 2*PI;
      
      double omega = -Ktheta_*(theta_-theta_d); 
      omega = clamp(omega, -2.84, 2.84);

      std::vector<double> command{v, omega};
      saveData(command);
      publishCommand(command);

      control_lock_ = false;
    }

  private:

    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Publisher data_pub_;
    ros::Subscriber setpoint_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber start_time_sub_;
    ros::Time start_time_;

    double cycle_rate_;
    double Kx_, Ky_, Ktheta_;
    bool started_, control_lock_;

    double x_, y_, theta_;
    double x_d_, y_d_, xdot_d_, ydot_d_;
    std::vector<double> ts_;
    std::vector<std::vector<double>> xs_, xds_, us_;

    static constexpr double PI = 3.14159265358979323846264;

    // Stops the controller and publishes a command to stop the robot.
    void stop_robot()
    {
      started_ = false;
      std::vector<double> stop_cmd{0.0, 0.0};
      publishCommand(stop_cmd);
      ROS_INFO_STREAM("Robot stopped");

    }

    void startTimeCallback(std_msgs::Time time)
    {
      started_ = !time.data.is_zero();
      if (started_)
      {
        start_time_ = time.data;
        ts_.clear(); xs_.clear(); xds_.clear(); us_.clear();
        ROS_WARN_STREAM("Control started. Press spacebar to stop...");
      }
      else
      {
        publishRolloutData();
        ROS_INFO_STREAM("Rollout data published");
        stop_robot();
      }
    }

    void setpointCallback(std_msgs::Float64MultiArray setpoint)
    {
      if (control_lock_)
        return;

      x_d_ = setpoint.data[0];
      xdot_d_ = setpoint.data[1];
      y_d_ = setpoint.data[2];
      ydot_d_ = setpoint.data[3];
    }

    void poseCallback(geometry_msgs::PoseStamped pose)
    {
      if (control_lock_)
        return;
      x_ = pose.pose.position.x;
      y_ = pose.pose.position.y;
      theta_ = headingAngle(pose);
    }

    // Takes a geometry_msgs/PoseStamped message, converts the orientation to
    // roll/pitch/yaw, and returns the yaw angle. The yaw angle is angle about
    // the veritcal z-axis.
    double headingAngle(geometry_msgs::PoseStamped pose)
    {
      tf::Quaternion q(pose.pose.orientation.x, 
                       pose.pose.orientation.y, 
                       pose.pose.orientation.z,
                       pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      return yaw;
    }

    // Saves all of the current data into vectors that will be exported
    void saveData(std::vector<double> command)
    {
      ts_.push_back((ros::Time::now() - start_time_).toSec());
      xs_.push_back({x_, y_, theta_});
      xds_.push_back({x_d_, xdot_d_, y_d_, ydot_d_});
      us_.push_back({command});
    }

    // Publishes the command to cmd_vel
    void publishCommand(std::vector<double> command)
    {
      geometry_msgs::Twist msg;
      msg.linear.x = command[0];
      msg.angular.z = command[1];
      cmd_pub_.publish(msg);
    }

    // Publishes all saved data for the rollout
    void publishRolloutData()
    {
      bootcamp_07_msgs::RolloutData msg;
      msg.ts = ts_;
      std::vector<std_msgs::Float64MultiArray> xs;
      std::vector<std_msgs::Float64MultiArray> xds;
      std::vector<std_msgs::Float64MultiArray> us;
      for (int i = 0; i < ts_.size(); i++)
      {
        std_msgs::Float64MultiArray x_msg;
        x_msg.data = xs_[i];
        xs.push_back(x_msg);

        std_msgs::Float64MultiArray xd_msg;
        xd_msg.data = xds_[i];
        xds.push_back(xd_msg);

        std_msgs::Float64MultiArray u_msg;
        u_msg.data = us_[i];
        us.push_back(u_msg);
      }
      msg.xs = xs;
      msg.xds = xds;
      msg.us = us;
      data_pub_.publish(msg);
    }

    // Clamps n between the supplied lower and upper limits.
    double clamp(double n, double lower, double upper) {
      return std::max(lower, std::min(n, upper));
    }

    bool key_is_pressed(KeySym ks)
    {
      Display *dpy = XOpenDisplay(":0");
      char keys_return[32];
      XQueryKeymap(dpy, keys_return);
      KeyCode kc2 = XKeysymToKeycode(dpy, ks);
      bool isPressed = !!(keys_return[kc2 >> 3] & (1 << (kc2 & 7)));
      XCloseDisplay(dpy);
      return isPressed;
    }

    bool space_is_pressed()
    {
      return key_is_pressed(XK_space);
    }
};



auto main(int argc, char **argv) -> int
{
  // initialize the node without using a default SigintHandler so that we can
  // use our own. The SigintHandler is used when the node is stopped with ctrl+C
  ros::init(argc, argv, "controller", ros::init_options::NoSigintHandler);

  // register our sigIntHandler
  signal(SIGINT, sigIntHandler);

  Controller controller;

  // Asynchronously start 3 spinners that run in the background for this node
  // which process callback asynchronously. We do this so that callbacks are
  // processed while the controller is executing.
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Rate rate(controller.cycle_rate());

  // main loop which exits when the g_request_shutdown flag is true
  while (!g_request_shutdown && ros::ok())
  {
    controller.control();
    ros::spinOnce();
    rate.sleep();
  }

  // shutdown gracefully if node is interrupted with ctrl+C
  controller.shutdown();

  return 0;
}