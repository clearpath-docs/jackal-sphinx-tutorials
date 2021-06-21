Extending Jackal's Startup
==========================

Now that's you've had Jackal for a while, you may be interested in how to extend it— perhaps add some more payloads,
or augment the URDF.


Startup Launch Context
----------------------

When ROS packages are grouped together in a directory and then built as one, the result is referred to as a
workspace. Each workspace generates a ``setup.bash`` file which the user may source in order to correctly
set up important environment variables such as ``PATH``, ``PYTHONPATH``, and ``CMAKE_PREFIX_PATH``.

The standard system-wide setup file is in ``opt``:

.. code:: bash

    source /opt/ros/noetic/setup.bash

When you run this command, you'll have access to `rosrun`, `roslaunch`, and all the other tools and packages
installed on your system from debian packages.

However, sometimes you want to add additional system-specific environment variables, or perhaps packages built
from source. For this reason, Clearpath platforms use a wrapper setup file, located in ``/etc/ros``:

.. code-block:: bash

    source /etc/ros/setup.bash

This is the setup file which gets sourced by Jackal's background launch job, and in the default configuration,
it is also sourced on your login session. For this reason it can be considered the "global" setup file for
Jackal's ROS installation.

This file sets some environment variables and then sources a chosen ROS workspace, so it is one of your primary
modification points for altering how Jackal launches.


Launch Files
------------

The second major modification point is the ``/etc/ros/noetic/ros.d`` directory. This location contains the
launch files associated with the ``ros`` background job. If you add launch files here, they will be launched with
Jackal's startup.

However, it's important to note that in the default configuration, any launch files you add may only reference ROS
software installed in ``/opt/ros/noetic/``. If you want to launch something from workspace in
the home directory, you must change ``/etc/ros/setup.bash`` to source that workspace's setup file rather than the
one from ``opt``.


Adding URDF
-----------

There are two possible approaches to augmenting Jackal's URDF. The first is that you may simply set the
``JACKAL_URDF_EXTRAS`` environment variable in ``/etc/ros/setup.bash``. By default, it points to an empty dummy file,
but you can point it to a file of additional links and joints which you would like mixed into Jackal's URDF (via
xacro) at runtime.

The second, more sophisticated way to modify the URDF is to create a *new* package for your own robot, and build
your own URDF which wraps the one provided by :roswiki:`jackal_description`.


Controllers
--------------------

Jackal ships with a PS4 controller for teleoperation.  If you need to re-pair the controller, you can do so
using the ``bluetoothctl`` command.  Ensure that the controller is in pairing mode by pressing and holding the
Share and PS buttons, run ``bluetoothctl`` on the robot, and enter the following commands:

.. code-block:: text

    agent on
    scan on

Look for your game controller; it should be identified with "Wireless Controller".  Copy the MAC address of the
controller, and then enter the following:

.. code-block:: text

    scan off
    pair <MAC ADDRESS>
    trust <MAC ADDRESS>
    connect <MAC ADDRESS>

Once the controller is connected the light should turn blue.  Press `ctrl+d` to exit bluetoothctl.

To drive the robot, hold the left shoulder button (L1) and use the left thumb-stick.  Holding the right shoulder button
(R1) will enable turbo, and increase the robot's maximum speed.

.. warning::

    Only enable turbo when you are familiar with how Jackal operates.  Do not use turbo in narrow, enclosed
    environments.  Always make sure you have a clear line of sight to the robot when operating it.

Other controllers can also be used with Jackal.  Some older robots shipped with a Logitech F710 controller.  This
controller uses a USB dongle and will pair automatically when the dongle is connected to the robot.  To enable
the Logitech controller add the following to ``/etc/ros/setup.bash``:

.. code-block:: bash

    export JACKAL_LOGITECH=1

and then run

.. code-block:: bash

    source /etc/ros/setup.bash
    sudo systemctl restart ros

The Logitech controller uses the same button layout as the PS4 controller: holding LB will enable driving and holding
RB will enable turbo.

Certain very old Jackal robots may have shipped with a PS3 controller.  If this is the case, you will need to follow
some additional steps.

First, add the PS3 driver apt repository:

.. code-block:: bash

    sudo add-apt-repository ppa:clearpath-robotics/ps3-joystick
    sudo apt-get update

Then install the driver:

.. code-block:: bash

    sudo apt-get install sixad

Finally, pair the PS3 controller with the ``sixpair`` command.
