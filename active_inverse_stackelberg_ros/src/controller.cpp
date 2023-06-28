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

      // Get parameters from the parameter server (which was probably configured
      // in the launch file that started this node) and assign this values to
      // member variables
      nh_.getParam("controller/cycle_rate", cycle_rate_);
      nh_.getParam("controller/Kx", Kx_);
      nh_.getParam("controller/Ky", Ky_);
      nh_.getParam("controller/Kv", Kv_);
      nh_.getParam("controller/Ktheta", Ktheta_);

      const auto queue_size = 100;
      throttle_pub_ = nh_.advertise<std_msgs::Float32>("jetracer/throttle", queue_size);
      steering_pub_ = nh_.advertise<std_msgs::Float32>("jetracer/steering", queue_size);
      data_pub_ = nh_.advertise<jetracer_racing_ros_msgs::RolloutData>("rollout_data", queue_size);

      start_time_sub_ = nh_.subscribe("start_time", queue_size, &Controller::startTimeCallback, this);
      setpoint_sub_ = nh_.subscribe("setpoint", queue_size, &Controller::setpointCallback, this);
      pose_sub_ = nh_.subscribe("vrpn_client_node/jetracer/pose", queue_size, &Controller::poseCallback, this);
      twist_sub_ = nh_.subscribe("vrpn_client_node/jetracer/twist", queue_size, &Controller::twistCallback, this);
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
      double v_des = sqrt(u_x*u_x + u_y*u_y);

      double theta_d;
      theta_d = atan2(u_y, u_x);

      if (abs(theta_ - theta_d) > abs(theta_ - (theta_d + 2*PI)))
        theta_d += 2*PI;
      else if (abs(theta_ - theta_d) > abs(theta_ - (theta_d - 2*PI)))
        theta_d -= 2*PI;
      
      double throttle =  clamp(Kv_*(v_des - v_), 0.0, 1.0);
      double steering = clamp(Ktheta_*(theta_d - theta_), -1.0, 1.0);
      std::vector<double> command{throttle, steering};

      saveData(command);
      publishCommand(command);

      control_lock_ = false;
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher throttle_pub_;
    ros::Publisher steering_pub_;
    ros::Publisher data_pub_;
    ros::Subscriber setpoint_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber twist_sub_;
    ros::Subscriber start_time_sub_;
    ros::Time start_time_;

    double cycle_rate_;
    double Kx_, Ky_, Kv_, Ktheta_;
    bool started_, control_lock_;

    double x_, y_, v_, theta_;
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

    void twistCallback(geometry_msgs::TwistStamped twist)
    {
      if (control_lock_)
        return;
      double xdot = twist.twist.linear.x;
      double ydot = twist.twist.linear.y;
      v_ = sqrt(pow(xdot, 2.0) + pow(ydot, 2.0));
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
      xs_.push_back({x_, y_, v_, theta_});
      xds_.push_back({x_d_, xdot_d_, y_d_, ydot_d_});
      us_.push_back({command});
    }

    // Publishes the command to cmd_vel
    void publishCommand(std::vector<double> command)
    {
      std_msgs::Float32 t, s;
      t.data = command[0];
      s.data = command[1];
      throttle_pub_.publish(t);
      steering_pub_.publish(s);
    }

    // Publishes all saved data for the rollout
    void publishRolloutData()
    {
      jetracer_racing_ros_msgs::RolloutData msg;
      msg.ts = ts_;
      std::vector<std_msgs::Float64MultiArray> xs;
      for (int i = 0; i < ts_.size(); i++)
      {
        std_msgs::Float64MultiArray x_msg;
        x_msg.data = xs_[i];
        xs.push_back(x_msg);
      }
      msg.xs = xs;
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