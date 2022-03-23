Setting Up Jackal's Network
===========================

Jackal is equipped with a combination Wifi + Bluetooth wireless module. On currently-shipping units, this
is an `Intel Centrino Advanced-N 6235`__. If this is your first unboxing, ensure that Jackal's wireless
antennas are firmly screwed on to the chassis.

.. _Centrino: http://www.intel.com/content/www/us/en/wireless-products/centrino-advanced-n-6235.html
__ Centrino_


First Connection
----------------

By default, Jackal's wireless is in client mode, looking for the wireless network at the Clearpath factory. In
order to set it up to connect to your own network, you'll have to open up the chassis and connect a network cable to
the PC's ``STATIC`` port. The other end of this cable should be connected to your laptop, and you should give yourself an IP address in the ``192.168.131.x`` space, such as ``192.168.131.50``. Then, make the connection to Jackal's default static IP:

.. code-block:: bash

    ssh administrator@192.168.131.1

The default password is ``clearpath``. You should now be logged into Jackal as the administrator user.

Changing the Default Password
-----------------------------

.. Note::

  All Clearpath robots ship from the factory with their login password set to ``clearpath``.  Upon receipt of your
  robot we recommend changing the password.

To change the password to log into your robot, run the

.. code-block:: bash

  passwd

command.  This will prompt you to enter the current password, followed by the new password twice.  While typing the
passwords in the ``passwd`` prompt there will be no visual feedback (e.g. "*" characters).

To further restrict access to your robot you can reconfigure the robot's SSH service to disallow logging in with a
password and require SSH certificates to log in.  This_ tutorial covers how to configure SSH to disable password-based
login.

.. _This: https://linuxize.com/post/how-to-setup-passwordless-ssh-login/

Connecting to Wifi Access Point
--------------------------------

Jackal's standard wireless network manager is wicd_. To connect to an access point in your lab, run:

.. code-block:: bash

    wicd-curses

You should see a browsable list of networks which the robot has detected. Use arrow keys to select the one you
would like to connect to, and then press the right arrow to configure it. You can enter your network's password
near the bottom of the page, and note that you must select the correct encryption scheme; most modern networks
use ``WPA1/2 Passphrase``, so if that's you, make sure that option is selected. You also likely want to select
the option to automatically reconnect to this network, so that Jackal will be there for you on your wireless
automatically in the future.

When you're finished, press F10 to save, and then C to connect.

Wicd will tell you in the footer what IP address it was given by your lab's access point, so you can now log out,
remove the network cable, and reconnect over wireless. When you've confirmed that all this is working as expected,
close up Jackal's chassis.

.. _wicd: https://launchpad.net/wicd


.. _remote:

Remote ROS Connection
---------------------

To use ROS desktop tools, you'll need your computer to be able to connect to Jackal's ROS master. This can be a
tricky process, but we've tried to make it as simple as possible.

In order for the ROS tools on your computer to talk to Jackal, they need to know two things:

- How to find the ROS master, which is set in the ``ROS_MASTER_URI`` environment variable, and
- How processes on the other computer can find *your computer*, which is the ``ROS_IP`` environment variable.

The suggested pattern is to create a file in your home directory called ``remote-jackal.sh`` with the following
contents:

.. code-block:: bash

    export ROS_MASTER_URI=http://cpr-jackal-0001:11311  # Jackal's hostname
    export ROS_IP=10.25.0.102                           # Your laptop's wireless IP address

If your network doesn't already resolve Jackal's hostname to its wireless IP address, you may need to add
a corresponding line to your computer's ``/etc/hosts`` file:

.. code-block:: bash

    10.25.0.101 cpr-jackal-0001

Then, when you're ready to communicate remotely with Jackal, you can source that script like so, thus defining
those two key environment variables in the present context.

.. code-block:: bash

    source remote-jackal.sh

Now, when you run commands like ``rostopic list``, ``rostopic echo``, ``rosnode list``, and others, the output
you see should reflect the activity on Jackal's ROS master, rather than on your own machine. Once you've
verified the basics (list, echo) from the prompt, try launching some of the standard visual ROS tools:

.. code-block:: bash

    roslaunch jackal_viz view_robot.launch
    rosrun rqt_robot_monitor rqt_robot_monitor
    rosrun rqt_console rqt_console

If there are particular :roswiki:`rqt` widgets you find yourself using a lot, you may find it an advantage to dock them together
and then export this configuration as the default RQT perspective. Then, to bring up your standard GUI, you can simply
run:

.. code-block:: bash

    rqt


Configuring Network Bridge
---------------------------

Jackal is configured to bridge its physical ethernet ports together.  This allows any ethernet port to be used as a
connection to the internal ``192.168.131.1/24`` network -- for connecting sensors, diagnostic equipment, or
manipulators -- or for connecting the robot to the internet for the purposes of installing updates.

Depending on which version of `Clearpath's OS installer <https://packages.clearpathrobotics.com/stable/images/latest/melodic-bionic/amd64/>`_
was used to install the OS on the robot, the bridge can be configured in one of two ways:

**Netplan**

Netplan is the default network configuration tool for Ubuntu 18.04 onward.  Instead of using the ``/etc/network/interfaces``
file, as was done in Ubuntu 16.04 and earlier, netplan uses YAML-formatted files located in ``/etc/netplan``.  The
default configuration file, ``/etc/netplan/50-clearpath-bridge.yaml``, is below:

.. code-block:: yaml

    # /etc/netplan/50-clearpath-bridge.yaml
    network:
    version: 2
    renderer: networkd
    ethernets:
      # bridge all wired interfaces together on 192.168.131.x
      bridge_eth:
        dhcp4: no
        dhcp6: no
        match:
          name: eth*
      bridge_en:
        dhcp4: no
        dhcp6: no
        match:
          name: en*

    bridges:
      br0:
        dhcp4: yes
        dhcp6: no
        interfaces: [bridge_eth, bridge_en]
        addresses:
          - 192.168.131.1/24

To enable network configuration using netplan you must install the ``netplan.io`` package:

.. code-block:: bash

    sudo apt-get install netplan.io


**Ifupdown & Interfaces**

Upon release, Jackal was configured to use the same networking tools on Ubuntu 16.04 running ROS Kinetic.  This was done
to ensure compatibility with Clearpath's other platforms, and to ease the transition to 18.04 and ROS Melodic.  As-of
December 2021, configuration using ``/etc/network/interfaces`` on Ubuntu 18.04 should be considered deprecated; the
configuration using ``netplan`` described above is the preferred method of configuring the network.

For reference, the default ``/etc/network/interfaces`` file for Jackal is below:

.. code-block::

    auto lo br0 br0:0
    iface lo inet loopback

    # Bridge together physical ports on machine, assign standard Clearpath Robot IP.
    iface br0 inet static
      bridge_ports regex (eth.*)|(en.*)
      address 192.168.131.1
      netmask 255.255.255.0
      bridge_maxwait 0

    # Also seek out DHCP IP on those ports, for the sake of easily getting online,
    # maintenance, ethernet radio support, etc.
    iface br0:0 inet dhcp

To enable network configuration using ``/etc/network/interfaces`` you must install the ``ifupdown`` package:

.. code-block:: bash

    sudo apt-get install ifupdown
