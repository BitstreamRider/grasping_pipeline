.. Grasping Pipeline documentation master file, created by
   sphinx-quickstart on Fri Mar  1 16:09:36 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Grasping Pipeline's documentation!
=============================================

The Grasping Pipeline implements a somewhat modular pipeline for grasping, placing and handing over objects with the Toyota HSR robot. 
The pipeline is implemented in Python and is now using the Robot Operating System 2 (ROS2) for communication with the robot.

.. note:: The current ROS2 version is ros2 humble (build for ubuntu 22.04LTS). Upgrading the ros2 version should be straight forward, but different ros2 version cannot talk to each other (changes in message serialization).

.. note:: This documentation is a work in progress.

.. toctree::
   :numbered:
   :maxdepth: 2
   :caption: Contents:

   installation
   switching_ros_version
   startup
   adding_new_estimators
   adding_new_objects
   overview_state_machine
   details_state_machine
   api



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
