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
the PC's ``STATIC`` port. The other end of this cable should be connected to your laptop, and you should give yourself
an IP address in the ``192.168.131.x`` space, such as ``192.168.131.100``. Then, make the connection to Jackal's default
static IP:

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

Jackal uses ``netplan`` for configuring its wired and wireless interfaces.  To connect Jackal to your wireless network,
create the file ``/etc/netplan/60-wireless.yaml`` and fill in the following:

.. code-block:: yaml

    network:
      wifis:
        # Fill in the SSID and PASSWORD fields as appropriate.  The password may be included as plain-text
        # or as a password hash.  To generate the hashed password, run
        #   echo -n 'WIFI_PASSWORD' | iconv -t UTF-16LE | openssl md4 -binary | xxd -p
        # If you have multiple wireless cards you may include a block for each device.
        # For more options, see https://netplan.io/reference/
        $wireless_interface:
          optional: true
          access-points:
            SSID_GOES_HERE:
              password: PASSWORD_GOES_HERE
          dhcp4: true
          dhcp4-overrides:
            send-hostname: true

Once configured, run

.. code-block:: bash

    sudo netplan apply

to bring up your wireless connection.  Running ``ip a`` will show all active connections and their IP addresses.


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

Reconfiguring the network bridge
-------------------------------------

In the unlikely event you must modify Jackal's ethernet bridge, you can do so by editing the Netplan configuration file
found at ``/etc/netplan/50-clearpath-bridge.yaml``:

.. code-block:: yaml

    # Configure the wired ports to form a single bridge
    # We assume wired ports are en* or eth*
    # This host will have address 192.168.131.1
    network:
    version: 2
    renderer: networkd
    ethernets:
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
      interfaces: [bridge_en, bridge_eth]
      addresses:
        - 192.168.131.1/24

This file will create a bridged interface called ``br0`` that will have a static address of 192.168.131.1, but will
also be able to accept a DHCP lease when connected to a wired router.  By default all network ports named ``en*`` and
``eth*`` are added to the bridge.  This includes all common wired port names, such as:

- ``eth0``
- ``eno1``
- ``enx0123456789ab``
- ``enp3s0``
- etc...

To include/exclude additional ports from the bridge, edit the ``match`` fields, or add additional ``bridge_*`` sections
with their own ``match`` fields, and add those interfaces to the ``interfaces: [bridge_en, bridge_eth]`` line near the
bottom of the file.

We do not recommend changing the static address of the bridge to be anything other than ``192.168.131.1``; changing
this may cause sensors that communicate over ethernet (e.g. lidars, cameras, GPS arrays) from working properly.
