Installing Jackal Robot Software
================================

.. note::

  The physical Jackal robot comes pre-configured with ROS2 and the necessary Jackal packages already installed; therefore, you will only need to follow the instructions below if re-installing software on the Jackal or updating from ROS1.

Updating firmware
-----------------

For ROS2 Foxy, Jackal uses new microROS_ based firmware. 

.. _microROS: https://micro.ros.org/

Install the Jackal firmware package on the Jackal:

.. code-block:: bash

    sudo apt-get update
    sudo apt-get install ros-foxy-jackal-firmware

To update the firmware, place the Jackal MCU into bootloader mode by switching the `PWR_MODE` switch from NORM to ALT, then pressing the `M_RST` button. 

Then, on the Jackal, run:

.. code-block:: bash

    jackal_firmware_flash.sh

Wait for the firmware to be flashed. 

Once the firmware has been flashed, place the Jackal MCU back into normal mode by switching the `PWR_MODE` switch from ALT to NORM.
The robot will power off. Turn the robot back on by pressing the power button.

Installing from Debian Packages
--------------------------------

The preferred way to install Jackal’s packages is using precompiled Debian packages. These packages are available for Ubuntu 20.04.

On the physical robot you need to install the ``jackal_robot`` package. In terminal, run:

.. code-block :: bash

    sudo apt-get install ros-foxy-jackal-robot

Installing from Source
-----------------------

Jackal packages are available on GitHub_, and can be compiled and installed from source if desired:

.. _GitHub: https://github.com/jackal/

1. Create a workspace directory. In terminal, run:

.. code-block:: bash

    mkdir -p ~/jackal_ws/src

2. Clone the Jackal repositories into your workspace directory. In terminal, run:

.. code-block:: bash

    cd ~/jackal_ws/src
    git clone -b foxy-devel https://github.com/jackal/jackal.git
    git clone -b foxy-devel https://github.com/jackal/jackal_robot.git
    cd ..

3. Source the ROS2 Foxy installation. In terminal, run:

.. code-block:: bash

    source /opt/ros/foxy/setup.bash

4. Install additional dependencies. In terminal, run:

.. code-block:: bash

    rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y

5. Build the workspace. In terminal, run:

.. code-block:: bash

    colcon build

6. You can now source your workspace to make use of the packages you just built. In terminal, run:

.. code-block:: bash

    source install/setup.bash

Installing the microROS agent
-----------------------------

The microROS agent is not currently released as a debian package, so we must build it from source.

1. If you don't yet have a workspace directory, create one. In terminal, run:

.. code-block:: bash

    mkdir -p ~/jackal_ws/src

2. Clone the `micro_ros_setup` repositories into your workspace directory. In terminal, run:

.. code-block:: bash

    cd ~/jackal_ws/src
    git clone -b foxy https://github.com/micro-ROS/micro_ros_setup.git
    cd ..

3. Source the ROS2 Foxy installation. In terminal, run:

.. code-block:: bash

    source /opt/ros/foxy/setup.bash

4. Install additional dependencies. In terminal, run:

.. code-block:: bash

    sudo apt update && rosdep update
    rosdep install --from-paths src --ignore-src -y

5. Build the workspace. In terminal, run:

.. code-block:: bash

    colcon build

6. You can now source your workspace to make use of the packages you just built. In terminal, run:

.. code-block:: bash

    source install/setup.bash

7. Create and build the microROS agent. In terminal, run:

.. code-block:: bash

    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh
    source install/setup.bash

Installing the systemd job
--------------------------

If you would like ROS2 to run on boot on the Jackal, first make sure your ``ROBOT_SETUP`` environment variable is set to your workspace before installing:

.. code-block:: bash

    export ROBOT_SETUP=~/jackal_ws/install/setup.bash

Then install the job:

.. code-block:: bash

    ros2 run jackal_robot install

By default this will install the job with a ROS_DOMAIN_ID of 0. To use a ROS_DOMAIN_ID of 5, for example, run:

.. code-block:: bash

    ros2 run jackal_robot install 5

Launching Jackal software manually
----------------------------------

To run the ROS2 software in the terminal, first source the workspace:

.. code-block:: bash

    source ~/jackal_ws/install/setup.bash

Then launch Jackal bringup. In terminal, run:

.. code-block:: bash

    ros2 launch jackal_robot bringup.launch.py
