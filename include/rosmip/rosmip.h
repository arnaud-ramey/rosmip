/*!
  \file        rosmip.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/1/7

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

\class Rosmip
  The interface between "libmip" and ROS.
\see rosmip.cpp for detailed topics and params.
 */
#ifndef ROSMIP_H
#define ROSMIP_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
// libmip
#include "libmip/src/bluetooth_mac2device.h"
#include "libmip/src/gattmip.h"

class Rosmip : public Mip {
public:
  Rosmip() : _nh_private("~") {
    main_loop = g_main_loop_new(NULL, FALSE);
    std::string device_mac, mip_mac;
    _nh_private.param("device_mac", device_mac, device_mac);
    _nh_private.param("mip_mac", mip_mac, mip_mac);
    _nh_private.param("use_odometry_speed", _use_odometry_speed, false);
    if (device_mac.empty() || mip_mac.empty()) {
      ROS_FATAL("Parameters 'device_mac' and 'mip_mac' must be set.");
      ros::shutdown();
      exit(-1);
    }
    if (!Mip::connect(main_loop, bluetooth_mac2device(device_mac).c_str(), mip_mac.c_str())) {
      ROS_FATAL("Could not connect with device MAC '%s' to MIP with MAC '%s'!",
                device_mac.c_str(), mip_mac.c_str());
      ros::shutdown();
      exit(-1);
    }
    ROS_INFO("Succesfully connected device MAC '%s' to MIP with MAC '%s' :)",
             device_mac.c_str(), mip_mac.c_str());
    // reset data
    _last_odometer_reading = -1;
    _x = _y = _th = _last_v = _last_w = _last_absspeed = 0; // reset odometry
    // advertise publishers
    _battery_voltage_pub = _nh_private.advertise<std_msgs::Float32>("battery_voltage", 1);
    _battery_percentage_pub = _nh_private.advertise<std_msgs::Float32>("battery_percentage", 1);
    _status_pub = _nh_private.advertise<std_msgs::String>("status", 1);
    _odometer_reading_pub = _nh_private.advertise<std_msgs::Float32>("odometer_reading", 1);
    _odom_pub = _nh_private.advertise<nav_msgs::Odometry>("odom", 50);
    _absspeed_pub = _nh_private.advertise<std_msgs::Float32>("absspeed", 1);
    // create subscribers
    _speed_sub = _nh_private.subscribe("speed", 1, &Rosmip::speed_cb, this);
    _sound_sub = _nh_private.subscribe("sound", 1, &Rosmip::sound_cb, this);
    _cmd_vel_sub = _nh_private.subscribe("cmd_vel", 1, &Rosmip::cmd_vel_cb, this);

    play_sound(23); // oh yeah
  }

  //////////////////////////////////////////////////////////////////////////////

  ~Rosmip() {
    play_sound(19); // see ya
    pump_up_callbacks();
  }

  //////////////////////////////////////////////////////////////////////////////

  bool spinOnce() {
    ros::Time now = ros::Time::now();
    // only request informations where there are subscribers
    if ((_status_pub.getNumSubscribers()
         || _battery_percentage_pub.getNumSubscribers()
         || _battery_voltage_pub.getNumSubscribers())
        && (now - _status_battery_request_stamp).toSec() > 1) {
      _status_battery_request_stamp = now;
      request_battery_voltage();
    }
    if ((_odometer_reading_pub.getNumSubscribers()
         || _odom_pub.getNumSubscribers()
         || _absspeed_pub.getNumSubscribers())
        && (now - _last_odometer_request_stamp).toSec() > .2) { // 5 Hz
      _last_odometer_request_stamp = now;
      request_odometer_reading();
    }
    if ((now - _last_odometry_pub_stamp).toSec() > .1) // 10 Hz
      refresh_odometry_tf(now);
    pump_up_callbacks();
    return true;
  } // end spinOnce()

  //////////////////////////////////////////////////////////////////////////////

protected:
  //////////////////////////////////////////////////////////////////////////////

  void cmd_vel_cb(const geometry_msgs::TwistConstPtr & msg) {
    _last_v = msg->linear.x;
    _last_w = msg->angular.z;
    continuous_drive_metric(_last_v, _last_w);
  }

  //////////////////////////////////////////////////////////////////////////////

  //! http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
  //! compute odometry in a typical way given the velocities of the robot
  void refresh_odometry_tf(const ros::Time & now) {
    double dt = (now - _last_odometry_pub_stamp).toSec();
    // store new values
    _last_odometry_pub_stamp = now;
    double curr_v = _last_v;
    if (_use_odometry_speed)
      curr_v = (_last_v > 0 ? 1 : -1) * _last_absspeed;
    double delta_x = (curr_v * cos(_th)) * dt;
    double delta_y = (curr_v * sin(_th)) * dt;
    double delta_th = _last_w * dt;
    _x += delta_x;
    _y += delta_y;
    _th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(_th);
    //first, we'll publish the transform over tf
    _odom_trans.header.stamp = now;
    _odom_trans.header.frame_id = "odom";
    _odom_trans.child_frame_id = "base_link";
    _odom_trans.transform.translation.x = _x;
    _odom_trans.transform.translation.y = _y;
    _odom_trans.transform.translation.z = 0.0;
    _odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(_odom_trans);

    //next, we'll publish the odometry message over ROS
    _odom.header.stamp = now;
    _odom.header.frame_id = "odom";
    //set the position
    _odom.pose.pose.position.x = _x;
    _odom.pose.pose.position.y = _y;
    _odom.pose.pose.position.z = 0.0;
    _odom.pose.pose.orientation = odom_quat;
    //set the velocity
    _odom.child_frame_id = "base_link";
    _odom.twist.twist.linear.x = curr_v;
    _odom.twist.twist.linear.y = 0;
    _odom.twist.twist.angular.z = _last_w;
    //publish the message
    _odom_pub.publish(_odom);
  } // end refresh_odometry_tf();

  //////////////////////////////////////////////////////////////////////////////

  void odometer_reading_cb(const ros::Time & now) {
    // compute speed (first derivative)
    double new_odom = get_odometer_reading();
    double dt = (now - _last_odometer_reading_stamp).toSec();
    if (_last_odometer_reading > 0) {
      _last_absspeed = (new_odom - _last_odometer_reading) / dt;
      _float_msg.data = _last_absspeed;
      _absspeed_pub.publish(_float_msg);
    }
    // store new values
    _last_odometer_reading_stamp = now;
    _last_odometer_reading = new_odom;
    // publish odometer reading
    _float_msg.data = new_odom;
    _odometer_reading_pub.publish(_float_msg);
  } // end odometer_reading_cb();

  //////////////////////////////////////////////////////////////////////////////

  //! extend this function to add behaviours upon reception of a notification
  virtual void notification_post_hook(MipCommand cmd, const std::vector<int> & /*values*/) {
    ros::Time now = ros::Time::now();
    if (cmd == CMD_MIP_STATUS) {
      // MiP spontaneously sends its status periodically
      _status_battery_request_stamp = now;
      _float_msg.data = get_battery_voltage();
      _battery_voltage_pub.publish(_float_msg);
      _float_msg.data = get_battery_percentage();
      _battery_percentage_pub.publish(_float_msg);
      _string_msg.data = get_status2str();
      _status_pub.publish(_string_msg);
    }
    if (cmd == CMD_ODOMETER_READING) {
      odometer_reading_cb(now);
    }
  } // end notification_post_hook();

  //////////////////////////////////////////////////////////////////////////////
  //! https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
  template <typename T> int signum(const T val) {
    return (val >= T(0)? 1 : -1);
  }

  bool speed2ticks(const double & v_ms, const double & w_rads,
                   int & v_int, int & w_int) {
    if (fabs(v_ms) > .95 || fabs(w_rads) > 17) {
      ROS_WARN("(v:%g, w:%g) out of bounds!", v_ms, w_rads);
      return false;
    }
    // v
    if (fabs(v_ms) < 0.01)
      v_int = 0;
    else if (fabs(v_ms) < 0.7) // NORMAL: v = 0.0217453422621 * b1
      v_int = clamp(45.98685952820811545096 * v_ms, -32, 32);
    else // CRAZY: v = 0.0290682838088 * b1
      v_int = clamp(34.40175576162719827641 * v_ms, -32, 32) + 32 * signum(v_ms);
    // w
    if (fabs(w_rads) < 0.2)
      w_int = 0;
    else if (fabs(w_rads) < 13) // NORMAL: w = 0.4177416860037 * b2 - 0.6876060987078
      w_int = clamp(2.39382382344084006941 * w_rads + 1.6460078602299454762, -32, 32);
    else // CRAZY: W = 0.7208400618386 * b2 + 0.0191642762841
      w_int = clamp(1.38727028773812161461 * w_rads - 0.02658603107493626709, -32, 32)
          + 32 * signum(w_rads);
    printf("speed2ticks(%g, %g) -> (%i, %i)\n", v_ms, w_rads, v_int, w_int);
    return true;
  } // end speed2ticks

  //////////////////////////////////////////////////////////////////////////////

  void speed_cb(const geometry_msgs::TwistConstPtr & msg) {
    int v_int, w_int;
    if (!speed2ticks(msg->linear.x, msg->angular.z, v_int, w_int)) {
      ROS_WARN("Fail in speed2ticks()");
      return;
    }
    if (!continuous_drive(v_int, w_int)) {
      ROS_WARN("Fail in continuous_drive()");
      return;
    }
  } // end speed_cb();

  //////////////////////////////////////////////////////////////////////////////

  void sound_cb(const std_msgs::Int8ConstPtr & msg) {
    play_sound((unsigned int) msg->data);
  } // end sound_cb();

  //////////////////////////////////////////////////////////////////////////////

  GMainLoop *main_loop;
  ros::NodeHandle _nh_public, _nh_private;
  ros::Time _status_battery_request_stamp;
  double _last_odometer_reading, _last_v, _last_w, _x, _y, _th, _last_absspeed;
  bool _use_odometry_speed;
  nav_msgs::Odometry _odom;
  geometry_msgs::TransformStamped _odom_trans;
  ros::Time _last_odometer_request_stamp, _last_odometer_reading_stamp;
  ros::Time _last_odometry_pub_stamp;
  std_msgs::Float32 _float_msg;
  std_msgs::String _string_msg;
  // publishers
  ros::Publisher _battery_voltage_pub, _battery_percentage_pub;
  ros::Publisher _status_pub;
  ros::Publisher _odometer_reading_pub, _absspeed_pub, _odom_pub;
  tf::TransformBroadcaster odom_broadcaster;
  // subscribers
  ros::Subscriber _cmd_vel_sub, _speed_sub, _sound_sub;
}; // end class Rosmip

#endif // ROSMIP_H

