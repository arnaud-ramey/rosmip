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
 */
#ifndef ROSMIP_H
#define ROSMIP_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
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
    }
    if (!Mip::connect(main_loop, bluetooth_mac2device(device_mac).c_str(), mip_mac.c_str())) {
      ROS_FATAL("Could not connect with device MAC '%s' to MIP with MAC '%s'!",
                device_mac.c_str(), mip_mac.c_str());
      ros::shutdown();
    }
    ROS_INFO("Succesfully connected device MAC '%s' to MIP with MAC '%s' :)",
             device_mac.c_str(), mip_mac.c_str());
    // reset data
    _last_odometer_reading = -1;
    _x = _y = _th = _last_v = _last_w = _last_absspeed = _tilt_angle = 0; // reset odometry
    // advertise publishers
    _battery_voltage_pub = _nh_public.advertise<std_msgs::Float32>("battery_voltage", 1);
    _battery_percentage_pub = _nh_public.advertise<std_msgs::Int16>("battery_percentage", 1);
    _status_pub = _nh_public.advertise<std_msgs::String>("status", 1);
    _odometer_reading_pub = _nh_public.advertise<std_msgs::Float32>("odometer_reading", 1);
    _odom_pub = _nh_public.advertise<nav_msgs::Odometry>("odom", 50);
    _absspeed_pub = _nh_public.advertise<std_msgs::Float32>("absspeed", 1);
    _tilt_pub = _nh_public.advertise<std_msgs::Int16>("tilt", 1);
    // create subscribers
    _sound_sub = _nh_public.subscribe("sound", 1, &Rosmip::sound_cb, this);
    _chest_led_sub = _nh_public.subscribe("chest_led", 1, &Rosmip::chest_led_cb, this);
    _chest_led_blink_sub = _nh_public.subscribe("chest_led_blink", 1, &Rosmip::chest_led_blink_cb, this);
    _head_led_sub = _nh_public.subscribe("head_led", 1, &Rosmip::head_led_cb, this);
    _head_led1_sub = _nh_public.subscribe("head_led1", 1, &Rosmip::head_led1_cb, this);
    _head_led2_sub = _nh_public.subscribe("head_led2", 1, &Rosmip::head_led2_cb, this);
    _head_led3_sub = _nh_public.subscribe("head_led3", 1, &Rosmip::head_led3_cb, this);
    _head_led4_sub = _nh_public.subscribe("head_led4", 1, &Rosmip::head_led4_cb, this);
    _cmd_vel_sub = _nh_public.subscribe("cmd_vel", 1, &Rosmip::cmd_vel_cb, this);
    _sharp_turn_sub = _nh_public.subscribe("sharp_turn", 1, &Rosmip::sharp_turn_cb, this);
    _sharp_turn_speed_sub = _nh_public.subscribe("sharp_turn_speed", 1, &Rosmip::sharp_turn_speed_cb, this);

    play_sound(23); // oh yeah
  }

  //////////////////////////////////////////////////////////////////////////////

  ~Rosmip() {
    play_sound(19); // see ya
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
         || _absspeed_pub.getNumSubscribers()
         || _tilt_pub.getNumSubscribers())
        && (now - _last_odometer_request_stamp).toSec() > .2) { // 5 Hz
      _last_odometer_request_stamp = now;
      request_odometer_reading();
      request_weight_update();
    }
    if ((now - _last_odometry_pub_stamp).toSec() > .1) // 10 Hz
      refresh_odometry_tf(now);
    return true;
  } // end spinOnce()

  //////////////////////////////////////////////////////////////////////////////

protected:
  //////////////////////////////////////////////////////////////////////////////

  void cmd_vel_cb(const geometry_msgs::TwistConstPtr & msg) {
    if (fabs(_last_v) < 1E-2 && fabs(msg->linear.x) < 1E-2
        && fabs(_last_w) < 1E-2 && fabs(msg->angular.z) < 1E-2) // do not pollute with "stop"
      return;
    _last_v = msg->linear.x;
    _last_w = msg->angular.z;
    continuous_drive_metric(_last_v, _last_w);
  }

  //////////////////////////////////////////////////////////////////////////////

  void sharp_turn_cb(const std_msgs::Float32Ptr & msg) { angle_drive(msg->data, 20); }

  void sharp_turn_speed_cb(const std_msgs::Float32MultiArrayConstPtr & msg) {
    if (msg->data.size() == 2)
      angle_drive(msg->data[0], msg->data[1]);
    else ROS_WARN("sharp_turn_cb(): incorrect data size %li",
                  msg->data.size());
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

  //! extend this function to add behaviours upon reception of a notification
  virtual void notification_post_hook(MipCommand cmd, const std::vector<int> & /*values*/) {
    ros::Time now = ros::Time::now();
    if (cmd == CMD_MIP_STATUS) {
      // MiP spontaneously sends its status periodically
      _float_msg.data = get_battery_voltage();
      _battery_voltage_pub.publish(_float_msg);
      _int_msg.data = get_battery_percentage();
      _battery_percentage_pub.publish(_int_msg);
      _string_msg.data = get_status2str();
      _status_pub.publish(_string_msg);
      _nh_public.setParam("battery_voltage", get_battery_voltage());
      _nh_public.setParam("battery_percentage", get_battery_percentage());
      _nh_public.setParam("status", get_status2str());
    }
    else if (cmd == CMD_ODOMETER_READING) {
      double dt = (now - _last_odometer_reading_stamp).toSec();
      if (_last_odometer_reading > 0) {
        _last_absspeed = (get_odometer_reading() - _last_odometer_reading) / dt;
        _float_msg.data = _last_absspeed;
        _absspeed_pub.publish(_float_msg);
        _nh_public.setParam("absspeed", _last_absspeed);
      }
      // store new values
      _last_odometer_reading_stamp = now;
      _last_odometer_reading = get_odometer_reading();;
      // publish odometer reading
      _float_msg.data = get_odometer_reading();;
      _odometer_reading_pub.publish(_float_msg);
    }
    else if (cmd == CMD_WEIGHT_UPDATE) {
      _int_msg.data = get_weight_update();
      _tilt_pub.publish(_int_msg);
      _nh_public.setParam("tilt", get_weight_update());
    }
  } // end notification_post_hook();

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

  void sound_cb(const std_msgs::Int16ConstPtr & msg) {
    play_sound((unsigned int) msg->data);
  } // end sound_cb();

  //////////////////////////////////////////////////////////////////////////////

  void chest_led_cb(const std_msgs::ColorRGBAConstPtr & msg) {
    set_chest_LED(255 * msg->r, 255 * msg->g, 255 * msg->b);
  } // end chest_led_cb();

  //////////////////////////////////////////////////////////////////////////////

  void chest_led_blink_cb(const std_msgs::Float32MultiArrayConstPtr & msg) {
    ROS_WARN("chest_led_blink_cb()");
    if (msg->data.size() == 5)
      set_chest_LED(msg->data[0], msg->data[1], msg->data[2],
          msg->data[3], msg->data[4]);
    else ROS_WARN("chest_led_blink_cb(): incorrect data size %li",
                  msg->data.size());
  } // end chest_led_blink_cb();

  //////////////////////////////////////////////////////////////////////////////

  void head_led_cb(const std_msgs::UInt8MultiArrayConstPtr & msg) {
    if (msg->data.size() == 4)
      set_head_LED(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
  } // end head_led_cb();

  //////////////////////////////////////////////////////////////////////////////

  void head_led1_cb(const std_msgs::BoolConstPtr & msg) { set_head_LED(1, msg->data); }
  void head_led2_cb(const std_msgs::BoolConstPtr & msg) { set_head_LED(2, msg->data); }
  void head_led3_cb(const std_msgs::BoolConstPtr & msg) { set_head_LED(3, msg->data); }
  void head_led4_cb(const std_msgs::BoolConstPtr & msg) { set_head_LED(4, msg->data); }

  //////////////////////////////////////////////////////////////////////////////

  GMainLoop *main_loop;
  ros::NodeHandle _nh_public, _nh_private;
  ros::Time _status_battery_request_stamp;
  double _last_odometer_reading, _last_v, _last_w, _x, _y, _th, _last_absspeed;
  bool _use_odometry_speed;
  nav_msgs::Odometry _odom;
  double _tilt_angle;
  geometry_msgs::TransformStamped _odom_trans;
  ros::Time _last_odometer_request_stamp, _last_odometer_reading_stamp;
  ros::Time _last_odometry_pub_stamp;
  std_msgs::Float32 _float_msg;
  std_msgs::String _string_msg;
  std_msgs::Int16 _int_msg;
  // publishers
  ros::Publisher _battery_voltage_pub, _battery_percentage_pub;
  ros::Publisher _status_pub;
  ros::Publisher _odometer_reading_pub, _absspeed_pub, _tilt_pub, _odom_pub;
  tf::TransformBroadcaster odom_broadcaster;
  // subscribers
  ros::Subscriber _cmd_vel_sub, _sound_sub, _sharp_turn_sub, _sharp_turn_speed_sub,
  _chest_led_sub, _chest_led_blink_sub,
  _head_led_sub, _head_led1_sub, _head_led2_sub, _head_led3_sub, _head_led4_sub;
}; // end class Rosmip

#endif // ROSMIP_H

