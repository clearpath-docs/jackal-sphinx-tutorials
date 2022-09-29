Driving Jackal
==============

There are four ways to drive Jackal, and each way will work on a physical Jackal robot as well as on a simulated Jackal.

The first two ways are teleoperation using a remote controller, and manually through publishing ROS2 messages. These two ways are covered in this section.

The third way is using the interactive controller in ``rviz``. This is covered in the :doc:`Simulation <JackalSimulation>` section.

The fourth way is through autonomous navigation. This will be covered in this tutorial in the future!

Safety Precautions
-------------------

.. Warning::

	Jackal is capable of reaching high speeds. Careless driving can cause harm to the operator, bystanders, the robot, or other property. Always remain vigilant, ensure you have a clear line of sight to the robot, and operate the robot at safe speeds. We strongly recommend driving in normal (slow) mode first, and only enabling turbo in large, open areas that are free of people and obstacles.

Driving with Remote Controller
---------------------------------

.. note::

	For instructions on controller pairing, see :doc:`Remote Controller Pairing <JackalControllerPairing>`.

Teleoperation is launched via the base launch which allows you to drive Jackal with the paired remote controller.

To drive the Jackal, Axis 0 controls the robot's steering, Axis 1 controls the forward/backward velocity, and Buttons 4 and 5 act as enable & enable-turbo respectively. On common controllers, these correspond to the following physical controls:

============= ==================================== ===== ===== ========= =======================
Axis/Button   Physical Input                       PS4   F710  Xbox One  Action
============= ==================================== ===== ===== ========= =======================
Axis 0        Left thumb stick horizontal          LJ    LJ    LJ        Drive forward/backward
Axis 1        Left thumb stick vertical            LJ    LJ    LJ        Rotate
Button 4      Left shoulder button or trigger      L1    LB    LB        Enable normal speed
Button 5      Right shoulder button or trigger     R1    RB    RB        Enable turbo
============= ==================================== ===== ===== ========= =======================

You must hold either Button 4 or Button 5 at all times while driving the robot.

Publishing ROS2 Messages
-------------------------

You can manually publish ``geometry_msgs/msg/Twist`` ROS2 messages to either the ``/jackal_velocity_controller/cmd_vel_unstamped`` or the ``/cmd_vel`` ROS2 topics to drive Jackal. 

For example, in terminal, run:

.. code-block:: bash

	ros2 topic pub --once /jackal_velocity_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

The command above makes Jackal drive forward momentarily at 0.5 m/s without any rotation. 

Motor Stop
---------------

Jackal has a motor stop button on its HMI panel. Pressing it will disable the motors. To re-enable the motors, simply press the button again.

Whenever you need to perform maintenance on Jackal, we recommend disabling the motors if the robot cannot be fully powered down.
