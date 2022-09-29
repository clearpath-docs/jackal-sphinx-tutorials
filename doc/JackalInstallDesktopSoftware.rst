Installing Jackal Desktop Software
==================================

.. note::

  If you wish to install the Jackal packages on your computer (e.g. to interface with the physical Jackal robot and/or to simulate Jackal), then proceed with the following instructions below. A prequisite is to make sure you have a working ROS2 Foxy installation set up on your computer.

Add Clearpath Debian Package Repository
----------------------------------------

Before you can install the Jackal packages, you need to configure Ubuntu's APT package manager to
add Clearpath's package server:

1. Install the authentication key for the packages.clearpathrobotics.com repository. In terminal, run:

.. code-block:: bash

    wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -

2. Add the debian sources for the repository. In terminal, run:

.. code-block:: bash

    sudo sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'

3. Update your computer's package cache. In terminal, run:

.. code-block:: bash

    sudo apt-get update

Installing from Debian Packages
--------------------------------

The preferred way to install Jackal’s packages is using precompiled Debian packages. These packages are available for Ubuntu 20.04.

After your computer is configured to use Clearpath's debian package repository, you can install the Jackal packages needed for this tutorial. In terminal, run:

.. code-block :: bash

    sudo apt-get install ros-foxy-jackal-desktop ros-foxy-jackal-simulator


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
    git clone -b foxy-devel https://github.com/jackal/jackal_desktop.git
    git clone -b foxy-devel https://github.com/jackal/jackal_simulator.git
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
