Upgrading ROS1 to ROS2
=======================

This section outlines the upgrade from ROS1 to ROS2 on a desired computer. More specifically, this section details how to install ROS2 Foxy regardless of whether or not ROS1 is already installed, since ROS2 is able to coexist with ROS1 on the same computer!

Upgrading Ubuntu OS Version
----------------------------

ROS2 specifically targets Ubuntu 20.04; therefore, if your computer is currently running an older Ubuntu OS version (e.g. Ubuntu 18.04 for ROS1 Melodic, or Ubuntu 16.04 for ROS2 Kinetic), you will need to update it to Ubuntu 20.04. 

The recommended method of installing Ubuntu 20.04 is from an ISO via a bootable USB drive. The Ubuntu 20.04 ISO can be found from `Ubuntu's official releases <https://releases.ubuntu.com/20.04/>`_.

On any computer:

1. Download the ``.iso`` file from the link above.

2. Insert a USB drive.

3. Write the downloaded Ubuntu 20.04 ISO to the USB drive using a software such as ``Rufus``, ``Etcher``, or ``UNetbootin``. This will erase all data already on the USB drive, so make sure you have backed up anything important!

On the computer to be upgraded to Ubuntu 20.04:

1. Ensure that it is turned off and has internet access via ethernet. 

2. Insert the newly formatted USB drive.

3. Turn on and choose to boot from the USB drive.

4. The installer should run automatically. Step through any prompts that come up. The computer will turn off automatically when the installation completes.

5. Once the computer turns off, remove the USB drive and turn on the computer. It will now be running your fresh install of Ubuntu 20.04.

Removing ROS1 Noetic
--------------------
If ROS1 Noetic was previous installed and there is a systemd job to start it on boot, it must be disabled to use ROS2 on boot.  This can be done by running `source /opt/ros/noetic/setup.bash; rosrun robot_upstart uninstall ros`.

Installing ROS2 Foxy
---------------------

Simply follow the `official ROS2 Foxy installation instructions <https://docs.ros.org/en/foxy/Installation.html>`_ to install ROS2 Foxy on your Ubuntu 20.04 computer.

If your Ubuntu 20.04 computer for some reason has ROS1 Noetic installed already, you will just need to ``source`` the desired ROS version and distribution before using it.

For example:

.. code-block:: bash

  source /opt/ros/foxy/setup.bash # Run this command to use ROS2 Foxy
  source /opt/ros/noetic/setup.bash # Run this command to use ROS1 Noetic
