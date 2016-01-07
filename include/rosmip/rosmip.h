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
#include "libmip/src/bluetooth_mac2device.h"
#include "libmip/src/gattmip.h"

class Rosmip : public Mip {
public:
  Rosmip() {}

  bool connect();

protected:
  ros::NodeHandle nh_public, nh_private;
}; // end class Rosmip

#endif // ROSMIP_H

