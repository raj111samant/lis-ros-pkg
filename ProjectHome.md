# Barrett WAM/Hand interface ros-pkg #

This ROS package repository contains an interface for the Barrett WAM and Hand, written for/by MIT's Learning and Intelligent Systems group (run by Profs. Leslie Kaelbling and Tomas Lozano-Perez).

It includes:
  * A plain socket interface (that can be used without ROS) for both 7-DOF arm and hand
  * ROS interface for the arm and hand
  * Two versions of inverse kinematics (analytical and optimization-based) for the 7-DOF arm
  * Interface for using ATI Nano17 6-axis force/torque sensors as fingertip sensors.

(more details in the [README](README.md))