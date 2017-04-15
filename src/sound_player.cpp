/*!
  \file        sound_player->cpp
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

A simple node for playing recordings of the sounds the MiP can emit.
 */
#include <std_msgs/Int16.h>
#include <ros/ros.h>
#include <ros/package.h>

std::string foldername = ros::package::getPath("rosmip") + "/third_parties/libmip/data/sounds/";

void sound_cb(const std_msgs::Int16::ConstPtr& sound) {
  std::ostringstream command;
  command << "play -q " << foldername << sound->data << ".flac";
  if (system(command.str().c_str()) < 0)
    ROS_WARN("Command '%s' returned an error!", command.str().c_str());
} // end sound_cb();

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "sound_player");
  ros::NodeHandle nh_public;
  // subscribers
  ros::Subscriber sound_sub = nh_public.subscribe("sound", 1,  sound_cb);
  ros::spin();
  return 0;
}



