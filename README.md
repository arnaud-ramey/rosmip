# rosmip

[![Build Status](https://travis-ci.org/arnaud-ramey/rosmip.svg)](https://travis-ci.org/arnaud-ramey/rosmip)

<p align="center">
<img src="doc/MiP.png" alt="MiP" style="width: 200px"/>
<img src="doc/ros_indigoigloo_600.png" alt="MiP" style="width: 200px"/>
</p>

Description
===========

"rosmip" is a driver to use the
[WoWWee MiP](http://wowwee.com/mip) robot
in [ROS](http://ros.org).

It is written in C++.
It relies on
[`libmip`](https://github.com/arnaud-ramey/libmip).
`libmip`
uses the Bluetooth Low Energy (BTLE, part of Bluetooth 4.0)
protocol for communication with the robot.
Your computer might not support it natively, in which case you need to
use a BTLE dongle.

Supported hardware
==================

The library is developed for the original
[WoWWee MiP](http://wowwee.com/mip).

Licence
=======

LGPL v3 (GNU Lesser General Public License version 3).
See LICENCE.

ROS driver node
===============

To launch the mip driver:

```bash
$ roslaunch rosmip rosmip.launch
```

Node parameters
---------------

- `device_mac`
  [std::string, default: ""]

  The MAC address of the Bluetooth Low Energy device to use.
  You can obtain the list of devices by running in a terminal

  ```
  $ hciconfig -a
  ```

  The Bluetooth Low Energy (BTLE) devices use Bluetooth 4.0 and
  can be identified by the line
  "HCI Version: 4.0"
  Also note the name of your interface, for instance `hci1`.

- `mip_mac`
  [std::string, default: ""]

  The MAC address of the MIP to use.
  To obtain it, start with resetting Bluetooth (from [ubuntu-fr.org](http://doc.ubuntu-fr.org/bluetooth#problemes_connus)) ,
  then perform a LE scan

  ```
  $ sudo rfkill unblock all
  $ sudo hciconfig hci1 up
  $ sudo hcitool -i hci1 lescan
  D0 :39:72: B7 : AF :66 ( unknown )
  D0 :39:72: B7 : AF :66 Bubi
  ```

Subscriptions
-------------

- `chest_led_blink`
  [std_msgs::Float32MultiArray, size:5 [r:0~255, g:0~255, b::0~255, time_flash_on_sec, time_flash_off_sec] ]

  Set the color of the chest and make it blink at the same time.
  The color is encoded as a RGB triplet (r, g, b),
  where each component is in 0~255.
  The blink is made of two periods: the time during which the light is ON,
  and the time during which the light is OFF.

- `chest_led`
  [std_msgs::UInt8MultiArray, size:3 [r:0~255, g:0~255, b::0~255] ]

  Set the color of the chest.
  The color is encoded as a RGB triplet (r, g, b),
  where each component is in 0~255.

- `cmd_vel`
  [geometry_msgs::Twist, (m/s, rad/s)]

  The instantaneous speed order.
  Send it every 10 Hz to obtain continuous motion.

- `sharp_turn`
  [std_msgs::Float32, radians,-22.25~22.25]

  Make a on-the-spot turn.
  Positive angles generate CCW turns.
  Equivalent to `sharp_turn_speed`, with a speed of 20.

- `sharp_turn_speed`
  [std_msgs::Float32MultiArray, size:2,
    [r:radians:-22.25~22.25, w:speed:0~24] ]

  Make a on-the-spot turn.
  Positive angles generate CCW turns.

- `sound`
  [std_msgs::Int16, 0~106]

  Play a sound on board the robot.
  Value in 1~106. Send 105 to stop playing current sound.


Publications
------------

- `absspeed`
  [std_msgs::Float32, m/s]

  The instantaneous absolute speed of the robot, in m/s.
  It is obtained by computing the first derivative of the motors odometry.
  This odometry is published on `odometer_reading`.

- `battery_percentage`
  [std_msgs::Int16, 0~100]

  The percentage of remaining battery.
  In 0~100, or < 0 if error.

- `battery_voltage`
  [std_msgs::Float32]

  The instantaneous voltage of the battery.
  Between 4.0V and 6.4V, or < 0 if error.

- `odometer_reading`
  [std_msgs::Float32, meters]

  The motors odometry.
  Caution, it is not signed, which means it is positive
  even if the robot goes backwards.

- `odom`
  [nav_msgs::Odometry]

  The odometry of the robot, obtained by integrating the speed comands.
  It is subject to a strong drift and should not be trusted too much.

- `status`
  [std_msgs::String]

  Returns one of the following:
  "ERROR", "FACE_DOWN", "FACE_DOWN_ON_TRAY", "HAND_STAND", "ON_BACK", "ON_BACK_WITH_KICKSTAND" "PICKED_UP", "UPRIGHT"


Keyboard remote control
=======================

To launch remote control of the mip thanks to keyboard:

```bash
$ roslaunch rosmip joy_teleop.launch
```

It is based on the [`turtlebot_teleop`](https://github.com/turtlebot/turtlebot/tree/indigo/turtlebot_teleop) package.

Joystick remote control
=======================

To launch remote control of the mip thanks to joystick:

```bash
$ roslaunch rosmip joy_teleop.launch
```

It is based on the [`joy`](http://wiki.ros.org/joy) package.

Installation
============

Dependencies
------------

You need the following libraries before compiling :

  * cmake  ( `sudo apt-get install cmake` )
  * glib   ( `sudo apt-get install libglib2.0-dev` )
  * bluez  ( `sudo apt-get install libbluetooth-dev` )
  * curses ( `sudo apt-get install libncurses5-dev` )
  * rfkill ( `sudo apt-get install rfkill` )

The [libgatt](https://github.com/jacklund/libgatt) library is
embedded into the project
and does not require any specific installation.

Build rosmip
-------------

```bash
$ catkin_make --only-pkg-with-deps rosmip
```
