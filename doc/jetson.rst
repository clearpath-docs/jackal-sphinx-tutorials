Installing a Jetson TX2 on a Jackal
=====================================

Start by opening up the Jackal to reveal the computer tray. Use the lever on the front of the Jackal then the thumb screws on the lid. For more instructions refer to the `Jackal User's Manual <http://bit.ly/1f4hmqP>`_.

Step 1: Remove mini-ITX Computer
--------------------------------

(Skip this step if you don't have a computer)

If you have a mini-ITX computer installed it will need to be removed. With the computer tray open locate each of the cables connected to the motherboard and harddrive. Remove the power, power switch, USB and SATA cables. The two cable ties can be cut to completely remove the power and SATA cables. The two antenna cables are connected underneath. Keep this in mind for the next steps.

.. image:: img1.JPG

Remove the four (4) hex screws from the motherboard using a 2.5mm wrench. Gently lift it out disconnecting the antenna connectors as you do so.

.. image:: img2.JPG

Remove the zipties holding the USB header, power switch signal and antenna wire. The Jetson TX2 doesn't have a USB header so it will have to be replaced with a USB mini cable. You will need extra slack in both the antenna cable and power switch signal as well.

.. image:: img3.JPG

Step 2: Install the TX2
------------------------
Install the Jetson TX2 using M3 screws or the screws removed from the computer. The holes will only line up in one orientation. Plug a USB mini cable from the Jetson to the Jackal MCU board.

.. image:: img4.JPG

This is beside where the USB header was removed if you had a computer. We recommend using a USB hub to increase the number of connected devices.  Plug the power switch cable into the power switch header (J6). If you wish to use the antenna cables attached to the Jackal remove the U.FL connectors from the Jetson module and connect the antenna cables. Caution: Take your time with this step the UF.L connectors are fragile.  If you wish not to disconnect the UF.L cables you can replace the antenna mounts on the Jackal with SMA extensions.

In order to power the Jetson make a cable which connects to the 12V user power to a barrel connector with center positive. Refer to the Jackal manual for the pinout of the user power. We used one of the molex user power connectors.

.. image:: img5.JPG

**Warning:** Make sure not to plug this into the ITX power plug on the power distribution board.

Step 3: Installing the software
--------------------------------

.. note::

  You will need at least 40GB of free disk space in order to download and extract the disk image for the TX2.

Using a computer running Ubuntu 16.04 LTS 64-bit, download JetPack 3.3 and follow the installation instructions.  Be sure to install at least the Linux for Tegra Host Side Image Setup.

* `Download Jetpack <https://developer.nvidia.com/embedded/dlc/jetpack-l4t-3_3>`_
* `Jetpack Install Instructions <https://docs.nvidia.com/jetson/archives/jetpack-archived/jetpack-321/index.html#jetpack/3.2.1/install.htm%3FTocPath%3D_____3>`_

.. image:: minimuminstall.png

Download Clearpath's pre-configured image for the Jetson TX2.

* `Jetson TX2 image <https://s3.amazonaws.com/cprjetsonimages/TX2/TX2_202003201337.img.raw.bz2>`_
* ``md5sum TX2_202003201337.img.raw.bz2``: 3e9e1ab4fd0228862a047bed85f7b891
* ``md5sum TX2_202003201337.img.raw``: f1587f1a4a0c2606a081cd1cd5e4c2db

Unzip the image by running the followng commands:

.. code-block:: bash

  cd {download location}
  pbzip2 -d TX2_202003201337.img.raw.bz2
  
``pbzip2`` is a parallel version of bzip2, and will unpack the image much faster than using the normal ``bzip2`` command.  You can install pbzip2 on your computer by running ``sudo apt-get install pbzip2`` if it is not installed already.
  
Now move the unzipped image to the bootloader directory of JetPack:

.. code-block:: bash

  mv TX2_202003201337.img.raw {jetpack install location}/64_TX2/Linux_for_Tegra_tx2/bootloader/system.img
  
You are now ready to install the image to the Jetson TX2:

1. Make sure the Jetson TX2 is powered off.  We recommend connecting an HDMI monitor, ethernet cable, and a USB mouse/keyboard to the Jetson TX2.
2. Connect the Jetson to your computer using the provided MicroUSB cable. Connect the other end of the MicroUSB cable to your computer
3. Connect the Jetson's power cable.
4. Start the Jetson TX2 in recovery mode by holding the REC button and pressing the power button.
5. Run the following commands on your computer:

.. code-block:: bash

  cd {jetpack install location}/64_TX2/Linux_for_Tegra_tx2
  sudo ./flash -r -L bootloader/cboot.bin jetson-tx2 mmcblk0p1

Writing the image to the jetson may take several minutes.

.. image:: flashcomplete.png

The TX2 will reboot automatically after installation is complete and will have ROS Kinetic installed along with the Jackal drivers.

To setup the Jetson to work with the Jackal, run ``bash ~/JACKAL_SETUP.sh`` on the Jetson and restart. When the Jetson starts up again, it should be connected to the Jackal. To see that the Jackal is connected by opening a terminal and executing ``rostopic echo /status``. You should see a 1hz message containing the Jackal's diagnostic information.

If you would like to pair a PS4 controller to drive the Jackal, hold down the PS and Share buttons on the controller until the light bar starts to flash. In a terminal on the Jackal, run ``sudo ds4drv-pair`` and wait for the controller to connect.  With the controller paired you should be able to control the Jackal by pressing L1 and using the left stick to drive. For more information see the Jackal manual.

To use your host computer with the Jackal first install ROS (http://wiki.ros.org/kinetic/Installation) and setup a catkin workspace (http://wiki.ros.org/catkin/Tutorials/create_a_workspace). Clone the general Jackal repo and the desktop specific repo in to the src folder and compile it. Installing rosdeps if necessary with ``rosdep install --from-paths src --ignore-src -r -y``. https://github.com/jackal/jackal and https://github.com/jackal/jackal_desktop. Note the network ip of the TX2 and setup your host computer to use it as the master. http://wiki.ros.org/ROS/Tutorials/MultipleMachines

You can then run ``roslaunch jackal_viz view_robot.launch`` on your host machine.  You should see a model of the robot and be able to move the Jackal using the interactive markers. See: http://www.clearpathrobotics.com/assets/guides/jackal/navigation.html
