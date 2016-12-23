/*!
  \file        mip_teleop_joy->cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/1/18

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

A simple node for teleoperating the mip
 */
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

int axis_linear = -1, axis_angular = -1;
int button_90left = -1, button_90right= -1, button_180turn = -1, button_360turn = -1;
int axis_90turn = -1, axis_180_360turn = -1;
int button_deadman = -1, button_sound = -1;
double scale_linear = 1.0, scale_angular = 1.0;
double offset_linear = 0.0, offset_angular = 0.0;
bool sharp_turn_before = false, sound_before = false;
std_msgs::String string_msg;
std_msgs::Int16 int_msg;
ros::Publisher cmd_vel_pub, sharp_turn_pub, sound_pub;

////////////////////////////////////////////////////////////////////////////////

void joy_cb(const sensor_msgs::Joy::ConstPtr& joy) {
  int naxes = joy->axes.size(), nbuttons = joy->buttons.size();
  bool command_sent = false,
      deadman_ok = (button_deadman < 0
                    || (nbuttons > button_deadman && joy->buttons[button_deadman]));
  if (!deadman_ok) {
    ROS_INFO_THROTTLE(10, "Dead man button %i is not pressed, sending a 0 speed order.", button_deadman);
    geometry_msgs::Twist vel;
    cmd_vel_pub.publish(vel);
    return;
  }
  // sharp turns at 90°
  bool left90now = (button_90left >= 0 && button_90left < nbuttons
                    && joy->buttons[button_90left]);
  bool right90now = (button_90right >= 0 && button_90right < nbuttons
                     && joy->buttons[button_90right]);
  bool axis90now = (axis_90turn >= 0 && axis_90turn < naxes
                    && fabs(joy->axes[axis_90turn]) > 0.9);
  // sharp turns at 180°
  bool button_180now = (button_180turn >= 0 && button_180turn < nbuttons
                      && joy->buttons[button_180turn]);
  bool button_360now = (button_360turn >= 0 && button_360turn < nbuttons
                      && joy->buttons[button_360turn]);
  bool axis_180_360now = (axis_180_360turn >= 0 && axis_180_360turn < naxes
                          && fabs(joy->axes[axis_180_360turn]) > 0.9);
  bool sharp_turn_now = left90now || right90now || axis90now
      || button_180now || button_360now || axis_180_360now;
  if (sharp_turn_now && !sharp_turn_before) {
    std_msgs::Float32 msg;
    if (left90now)            msg.data = -M_PI_2;
    else if (right90now)      msg.data = M_PI_2;
    else if (axis90now)       msg.data = (joy->axes[axis_90turn] < 0 ? M_PI_2 : -M_PI_2);
    else if (button_180now)   msg.data = -M_PI;
    else if (button_360now)   msg.data = 2 * M_PI;
    else if (axis_180_360now) msg.data = (joy->axes[axis_180_360turn] > 0 ? 2 * M_PI : -M_PI);
    ROS_INFO("Starting sharp turm of %i degrees!", (int)(msg.data * 180 / M_PI));
    //  ROS_INFO("%i %i %i %i %i %i", left90now, right90now, axis90now
    //          , button_180now, button_360now, axis_180_360now);
    sharp_turn_pub.publish(msg);
    command_sent = true;
  }
  sharp_turn_before = sharp_turn_now;

  // sounds
  bool sound_now = button_sound < nbuttons && joy->buttons[button_sound];
  if (sound_now && !sound_before) {
    int_msg.data = 103; // laser sound
    sound_pub.publish(int_msg);
    ROS_INFO("Starting sound!");
    command_sent = true;
  }
  sound_before = sound_now;

  // if no command was sent till here: move robot with directions of axes
  if (command_sent)
    return;
  geometry_msgs::Twist vel;
  if (axis_linear < naxes)
    vel.linear.x = ((joy->axes[axis_linear]-offset_linear) * scale_linear);
  if (axis_angular < naxes)
    vel.angular.z = ((joy->axes[axis_angular]-offset_angular) * scale_angular);
  cmd_vel_pub.publish(vel);

} // end joy_cb();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "mip_teleop_joy");
  ros::NodeHandle nh_public, nh_private("~");
  // params
  nh_public.param("axis_180_360turn", axis_180_360turn, axis_180_360turn);
  nh_public.param("axis_90turn", axis_90turn, axis_90turn);
  nh_public.param("axis_angular", axis_angular, axis_angular);
  nh_public.param("axis_linear", axis_linear, axis_linear);
  nh_public.param("button_180turn", button_180turn, button_180turn);
  nh_public.param("button_360turn", button_360turn, button_360turn);
  nh_public.param("button_90left", button_90left, button_90left);
  nh_public.param("button_90right", button_90right, button_90right);
  nh_public.param("button_deadman", button_deadman, button_deadman);
  nh_public.param("button_sound", button_sound, button_sound);
  nh_public.param("offset_angular", offset_angular, offset_angular);
  nh_public.param("offset_linear", offset_linear, offset_linear);
  nh_public.param("scale_angular", scale_angular, scale_angular);
  nh_public.param("scale_linear", scale_linear, scale_linear);
  // subscribers
  ros::Subscriber joy_sub = nh_public.subscribe<sensor_msgs::Joy>("joy", 1,  joy_cb);
  // publishers
  sound_pub = nh_public.advertise<std_msgs::Int16>("sound", 1);
  cmd_vel_pub = nh_public.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  sharp_turn_pub = nh_public.advertise<std_msgs::Float32>("sharp_turn", 1);
  ros::spin();
  return 0;
}


