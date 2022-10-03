Jackal Tests
============

Jackal robots come preinstalled with a set of test scripts as part of the ``jackal_tests`` ROS package, which can be run to verify robot functionality at the component and system levels. 

If your Jackal does not have the ``jackal_tests`` ROS package installed already, you can manually install it by opening terminal and running:

.. code-block:: bash

  sudo apt-get install ros-noetic-jackal-tests

ROS Tests
----------

The ``ros_tests`` script exposes a set of interactive tests to verify the functionality of core features. These tests run at the ROS-level via ROS topics, and serve as a useful robot-level diagnostic tool for identifying the root cause of problems, or at the very least, narrowing down on where the root cause(s) may be.

Running ROS Tests
------------------

To run ``ros_tests`` on a Jackal robot, open terminal and run:

.. code-block:: bash

  rosrun jackal_tests ros_tests

Upon running ``ros_tests``, a list of available tests will be shown in a menu. From the menu, you can choose individual tests to run, or simply choose the option to automatically run all the tests.

The details of each test are shown below.

**Motion Stop Test**

.. note::

  This test is formerly called the E-Stop Test.

The **Motion Stop Test** checks that the robot's motion stop is working properly. 

This test subscribes to the ``/status`` ROS topic and checks that when the motion stop is manually engaged by the user, the motion stop state is correctly reported on the ``/status`` ROS topic.

**ADC Test**

The **ADC Test** checks that the robot's voltage and current values across its internal hardware components are within expected tolerances.

This test subscribes to the ``/status`` ROS topic and checks that the voltage and current values across the internal hardware are within expected tolerances.

**Rotate Test**

The **Rotate Test** rotates the robot counter clockwise 2 full revolutions and checks that the motors, IMU, and EKF odometry are working properly.

This test:

- Subscribes to the ``/imu/data`` ROS topic to receive angular velocity measurements from the IMU's Gyroscope. These measurements are converted into angular displacement estimations, and the robot will rotate until 2 full revolutions are estimated.
- Subscribes to the ``/odometry/filtered`` ROS topic to receive angular velocity estimations from the EKF odometry. These measurements are converted into angular displacement estimations, and are output as comparison to the angular displacement estimations from the IMU's Gyroscope.
- Publishes to the ``/cmd_vel`` ROS topic to send drive commands to rotate the robot.
- The user will be asked to verify that the robot rotates 2 full revolutions.

.. note::

  The **Rotate Test** rotates the robot using the IMU's Gyroscope data, which inherently will not be 100% accurate. Therefore, some undershoot/overshoot is to be expected.

**Drive Test**

The **Drive Test** drives the robot forward 1 meter and checks that the motors, encoders, and encoder-fused odometry are working properly.

This test:

- Subscribes to the ``/jackal_velocity_controller/odom`` ROS topic to receive linear displacement estimations from the encoder-fused odometry. The robot will drive forward until 1 meter is estimated.
- Subscribes to the ``/feedback`` ROS topic to receive linear displacement measurements from the individual encoders. These measurements are output as comparison to the linear displacement estimations from the encoder-fused odometry.
- Subscribes to the ``/joint_state`` ROS topic to receive linear displacement measurements from individual the encoders. These measurements are output as comparison to the linear displacement estimations from the encoder-fused odometry.
- Publishes to the ``/cmd_vel`` ROS topic to send drive commands to drive the robot.
- The user will be asked to verify that the robot drives forward 1 meter.

.. note::

  The **Drive Test** drives the robot using the Odometry data, which inherently will not be 100% accurate. Therefore, some undershoot/overshoot is to be expected.