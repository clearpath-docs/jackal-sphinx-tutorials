Extending Jackal on Windows 
===========================

Now that's you've had Jackal for a while, you may be interested in how to extend it— perhaps add some more payloads,
or augment the URDF.


Startup Launch Context
----------------------

When ROS packages are grouped together in a directory and then built as one, the result is referred to as a
workspace. Each workspace generates a ``setup.bat`` file which the user may source in order to correctly
set up important environment variables such as ``PATH``, ``PYTHONPATH``, and ``CMAKE_PREFIX_PATH``.

The standard system-wide setup file is in ``c:\opt``:

.. code:: batch

    c:\opt\ros\melodic\x64\setup.bat

When you run this command, you'll have access to `rosrun`, `roslaunch`, and all the other tools and packages
installed on your system from debian packages.

Supporting ROS packages on Windows
----------------------------------

While many ROS packages are supported on Windows, most are not. Microsoft has provided `documentation`_ for 
updating and consuming ROS nodes. Please refer to Microsoft's documentation for porting and support options.


.. _documentation: http://aka.ms/ros/docs

Adding URDF
-----------

There are two possible approaches to augmenting Jackal's URDF. The first is that you may simply set the
``JACKAL_URDF_EXTRAS`` environment variable in Windows using the ``setx`` command. By default, it points to an empty dummy file,
but you can point it to a file of additional links and joints which you would like mixed into Jackal's URDF (via
xacro) at runtime.

The second, more sophisticated way to modify the URDF is to create a *new* package for your own robot, and build
your own URDF which wraps the one provided by :roswiki:`jackal_description`.

Starting ROS at boot
--------------------

To automatically start on ROS boot, you'll use the Windows Task Scheduler to start the task. The Task scheduler is 
passed a Windows command file, which needs to start the ROS environment, and your code.

Create a Windows command file, which includes the ROS environment and Install environment:

.. code:: batch

    c:\catkin_ws\start_ros.bat

Inside that file, place the commands to initiatize ROS, initialize your workspace, and launch your top level launch file:

.. code:: batch

    call c:\opt\ros\melodic\x64\setup.bat
    call c:\catkin_ws\install\setup.bat
    roslaunch <package> <launch file>

Use the command Schtasks, to add a task which calls this script:

.. code:: batch

    schtasks /Create /RU <User> /RP <password> /SC ONLOGON /TN ROS /TR "c:\catkin_ws\start_ros.bat"

The next time the system starts, the ROS task will run.

