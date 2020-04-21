Setting up Jackal with Windows
==============================

The Clearpath Jackal does not ship with Windows installed by default. However, the core Jackal ROS nodes 
are enabled on Windows 10, and ROS on Windows is supported by Microsoft. This guide will cover installing 
Windows on your Jackal, installing ROS & Jackal ROS components, updating firmware and configuring Windows 
for use.

The following instructions assume that the Jackal is connected to a keyboard and mouse during setup. Once 
configured, these can be removed and remote access can be leveraged.

For more information on ROS on Windows, please visit `Microsoft's ROS documentation`_.

.. _Microsoft's ROS documentation: http://aka.ms/ros

Installing Windows 10
---------------------

ROS1 is supported by Microsoft for Windows 10 Desktop and Windows 10 IoT Enterprise. Windows 10 IoT Enterprise
is an embedded version of Windows 10 for dedicated purpose devices, such as a Robot.

1. You can `download a trial`_ of Windows 10 IoT Enterprise Long Term Service (LTSC) x64 build, use a Windows license from your Enterprise agreement, or a commercially purchased version of Windows 10 Desktop.
2. Create a `USB Boot stick`_ and copy the contents of the iso generated from the image above.
3. Boot Windows and install following the prompts.
4. Install ROS on Windows using the `ROS Wiki instructions`_.
5. Please follow the instructions closely, as ROS is sensitive to the operating environment. 

This will install the development platform on the Jackal, which allows you to develop your robotics solutions. Once you've 
finalized your development, you can build a deployment package and deploy your solutions using Chocolatey. 

If you see any problems with Jackal, please `get in touch`_ and we'll see if we can get you sorted out.

If you have Windows specific questions or problems, Microsoft offers `community support through Github`_ and `paid support`_. 

.. _download a trial: https://www.microsoft.com/en-us/evalcenter/evaluate-windows-10-enterprise
.. _ROS Wiki instructions: https://wiki.ros.org/Installation/Windows
.. _USB Boot Stick: https://docs.microsoft.com/en-us/windows-hardware/manufacture/desktop/install-windows-from-a-usb-flash-drive
.. _get in touch: https://support.clearpathrobotics.com/hc/en-us/requests/new
.. _community support through Github: https://github.com/ms-iot/rosonwindows
.. _paid support: http://aka.ms/ros/support

Installing Jackal software
--------------------------

Once you've installed Windows and ROS, It's time to install the Jackal software. Please open an Administrative ROS command prompt created during the ROS install.

.. code-block:: batch

    :: Only need to install wget once.
    choco install wget
    :: Close and Reopen the command Window.
    
    mkdir c:\ws
    cd c:\ws
    wget https://raw.githubusercontent.com/jackal/jackal-sphinx-tutorials/master/jackal.rosinstall
    wstool init src jackal.rosinstall
    
    :: Later versions of pyproj are not deployed as a binary and do not build automatically
    pip install pyproj==1.9.6

    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make_isolated
    devel_isolated\setup.bat

.. note:: rosdep will not find several components. These include rosserial_server, robot_upstart, joint_state_publisher_gui and others. gmapping is coming soon.

.. note:: The Jackal ROS Nodes and support software is distributed as source code on Github. Other Clearpath repositories have not been enabled as of April 2020, but will be evaluated based on demand.

MCU Firmware Update
-------------------

When you update packages, there is periodically a new version of Jackal's firmware available. 

If new firmware is available, follow the below procedure to flash it to Jackal's MCU:

1. Place Jackal up on blocks. Firmware loading does not usually result in unintended motion, but it's safest when
   off the ground.
2. Ensure that Jackal is on and open.
3. In the top-middle of the main circuit board, look for a small left-right switch in between two buttons.
4. By default it is in the left position, but move it now to the right position.
5. Press the reset button to the left, labeled ``M_RST``. Jackal is now in its bootloader, ready to receive new
   firmware.

Now, from Jackal's PC (connected via Windows Remote Desktop or screen/keyboard), in the ROS command Window created during setup:

.. code-block:: batch

    cd c:\ws
    devel_isolated\setup.bat
    rosrun jackal_firmware_ms scripts/upload.bat
    
During execution of the script, it will display a dialog. Please follow the instructions in the command window.

You should see about 20 seconds worth of lines output beginning with "Download from image ...". When this is
complete, move the switch back to the leftmost position and quickly push the reset button again. You're now
running the updated firmware!

If you're too slow on pushing the reset button, Jackal will power off, including the internal PC. It's okay
if that happens; just press the external power button again, and you should be back in business.


Wireless Network Setup
----------------------
Windows provides an enterprise class network stack. To connect to a `Wifi`_, use the Windows networking user interface to connect 
to an access point. 

.. _Wifi: https://support.microsoft.com/en-us/help/17137/windows-setting-up-wireless-network


Cellular Network Setup
----------------------

Windows also provides cellular connectivity using an external Cellular modem. Microsoft provides documentation for `consumer`_ and `enterprise`_ cellular deployments.


For enterprise cellular deployments, please refer to this documentation:

.. _consumer: https://support.microsoft.com/en-us/help/10739/windows-10-cellular-settings
.. _enterprise: https://docs.microsoft.com/en-us/windows/configuration/provisioning-apn


Bluetooth Controller Pairing
----------------------------

Windows provides a robust Bluetooth stack. To pair a controller, use the Windows Bluetooth paring user interface.

Once the pairing is complete you should be able to control the robot using your controller.  Note that the first time
you pair the controller it may be enumerated as the wrong device.  If the robot does not respond to your commands,
power-cycle the full system and you should be set.

To use teleop on Jackal, clone Microsoft's fork of the joystick driver, which leverages the Open Source SDL2 library for controller interfaces. 
You'll need to ensure that this joystick ROS node is started by your launch files.

.. code-block:: batch

    cd c:\ws
    devel_isolated\setup.bat
    git clone -b init_windows https://github.com/ms-iot/joystick_drivers
    catkin_make

