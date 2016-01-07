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

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "libmip/src/bluetooth_mac2device.h"
#include "libmip/src/gattmip.h"

class Rosmip : public Mip {
public:
  Rosmip() : _nh_private("~") {
    main_loop = g_main_loop_new(NULL, FALSE);
    std::string device_mac, mip_mac;
    _nh_private.param("device_mac", device_mac, device_mac);
    _nh_private.param("mip_mac", mip_mac, mip_mac);
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
    // advertise publishers
    _battery_voltage_pub = _nh_private.advertise<std_msgs::Float32>("battery_voltage", 1);
    _battery_percentage_pub = _nh_private.advertise<std_msgs::Float32>("battery_percentage", 1);
    _status_pub = _nh_private.advertise<std_msgs::String>("status", 1);
    _odometer_reading_pub = _nh_private.advertise<std_msgs::Float32>("odometer_reading", 1);
    // create subscribers
    _speed_sub = _nh_private.subscribe("speed", 1, &Rosmip::speed_cb, this);

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
        && (now - _status_battery_stamp).toSec() > 1) {
      _status_battery_stamp = now;
      request_battery_voltage();
    }
    if (_odometer_reading_pub.getNumSubscribers()
        && (now - _odometer_reading_stamp).toSec() > 1) {
      _odometer_reading_stamp = now;
      request_odometer_reading();
    }
    pump_up_callbacks();
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

protected:

  //////////////////////////////////////////////////////////////////////////////
  //! https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
  template <typename T> int signum(const T val) {
    return (T(0) < val) - (val < T(0));
  }

  bool speed2ticks(const double & v_ms, const double & w_rads,
                   int & v_int, int & w_int) {
    printf("speed2ticks(%g, %g)\n", v_ms, w_rads);
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
    return true;
  } // end speed2ticks

  //////////////////////////////////////////////////////////////////////////////

  //! extend this function to add behaviours upon reception of a notification
  virtual void notification_post_hook(MipCommand cmd, const std::vector<int> & /*values*/) {
    if (cmd == CMD_MIP_STATUS) {
      // MiP spontaneously sends its status periodically
      _status_battery_stamp = ros::Time::now();
      _float_msg.data = get_battery_voltage();
      _battery_voltage_pub.publish(_float_msg);
      _float_msg.data = get_battery_percentage();
      _battery_percentage_pub.publish(_float_msg);
      _string_msg.data = get_status2str();
      _status_pub.publish(_string_msg);
    }
    if (cmd == CMD_ODOMETER_READING) {
      _float_msg.data = get_odometer_reading();
      _odometer_reading_pub.publish(_float_msg);
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

  GMainLoop *main_loop;
  ros::NodeHandle _nh_public, _nh_private;
  // publishers
  ros::Publisher _battery_voltage_pub, _battery_percentage_pub;
  ros::Publisher _status_pub;
  ros::Time _status_battery_stamp;
  ros::Publisher _odometer_reading_pub;
  ros::Time _odometer_reading_stamp;
  std_msgs::Float32 _float_msg;
  std_msgs::String _string_msg;
  // subscribers
  ros::Subscriber _speed_sub;
}; // end class Rosmip

#endif // ROSMIP_H

