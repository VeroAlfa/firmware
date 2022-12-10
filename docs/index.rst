.. MBSE-2022-1/Firmware-Team documentation master file, created by
   sphinx-quickstart on Mon Dec  5 15:21:19 2022.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

MBSE-2022-1/Firmware-Team Documentation
=======================================
Hello world!!! *This* is firm ware team **MBSE-2022-1**

``code code code``

System Architecture
-------------------
- **Firmware Architecture**

   .. image:: /images/firmware_architecture.png

  * Hardwares and Connection
  
    + :doc:`Microcontroller </pages/microcontroller>`
    + :doc:`Hub Motor and Driver </pages/hub-motor-and-driver>`
    + :doc:`IMU Sensor </pages/imu-sensor>`

  * Microcontroller Subsystem

    + :doc:`Hub Motor Interface </pages/hub-motor-interface>`
    + :doc:`IMU Interface </pages/imu-interface>`
    + :doc:`Wheel Velocity Estimation </pages/wheel-velocity-estimation>`
    + :doc:`Forward Kinematics </pages/forward-kinematics>`
    + :doc:`Inverse Kinematics </pages/inverse-kinematics>`
    + :doc:`Wheel Odematry Computation </pages/wheel-odometry-computation>`
    + :doc:`ROS2 Interface </pages/ros2-interface>`

- **ROS2 Architecture**

   .. image:: /images/ros2_architecture.png
      
  * :doc:`ROS2 Setup </pages/ros2-setup>`
  
    + Calibration and xicro installation
    + Microcontroller port check
  
  * :doc:`Xicro Package </pages/xicro-package>`
  * :doc:`Calibration Package </pages/calibration-package>`
  * :doc:`Steps to open ROS2 nodes </pages/step-to-open>`

.. toctree::
   :maxdepth: 3
   :caption: Hardwares and Connection:
   :hidden:

   pages/microcontroller
   pages/hub-motor-and-driver
   pages/imu-sensor

.. toctree::
   :maxdepth: 3
   :caption: Microcontroller Subsystem:
   :hidden:

   pages/hub-motor-interface
   pages/imu-interface
   pages/wheel-velocity-estimation
   pages/forward-kinematics
   pages/inverse-kinematics
   pages/wheel-odometry-computation
   pages/ros2-interface

.. toctree::
   :maxdepth: 3
   :caption: ROS2:
   :hidden:

   pages/ros2-setup
   pages/xicro-package
   pages/calibration-package
   pages/step-to-open


.. toctree::
   :maxdepth: 3
   :caption: Contents:
   :hidden:

   pages/dummy1
   pages/manipulator-firmware
   pages/mobile-robot-firmware

.. toctree::
   :maxdepth: 3
   :caption: File:
   :hidden:

   pages/hardware-uses
   pages/hardware-references

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
* Source code: https://github.com/MBSE-2022-1/Firmware-Team
