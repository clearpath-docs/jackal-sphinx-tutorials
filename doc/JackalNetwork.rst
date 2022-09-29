Setting Up Jackal's Network
===========================

Jackal is normally equipped with a combination Wifi + Bluetooth module, such as the `Intel Centrino Advanced-N 6235`__. If this is your first unboxing, ensure that Jackal's wireless antennas are firmly screwed on to the chassis.

.. _Centrino: http://www.intel.com/content/www/us/en/wireless-products/centrino-advanced-n-6235.html
__ Centrino_

Some Jackal robots may only be equipped with a single antenna, depending on the exact model of PC installed in the robot.

First Connection
-----------------

In order to set Jackal up to connect to your own wireless network, you will first need to access the Jackal's computer from you computer over ``ssh``:

1. Configure your computer to have a static IP address on the ``192.168.131.x`` subnet, e.g. ``192.168.131.100``.

2. Connect an ethernet cable between Jackal's computer and your computer.

3. ``ssh`` into Jackal's computer from your computer. In terminal, run:

.. code-block:: bash

    ssh administrator@192.168.131.1

The default password is ``clearpath``. You should now be logged into Jackal as the administrator user.

Changing the Default Password
------------------------------

.. Note::

  All Clearpath robots ship from the factory with their login password set to ``clearpath``.  Upon receipt of your robot we recommend changing the password.

To change the password to log into your robot, you can use the ``passwd`` command. In terminal, run:

.. code-block:: bash

  passwd

This will prompt you to enter the current password, followed by the new password twice.  While typing the passwords in the ``passwd`` prompt there will be no visual feedback (e.g. "*" characters).

To further restrict access to your robot you can reconfigure the robot's ``ssh`` service to disallow logging in with a password and require ``ssh`` certificates to log in.  This_ tutorial covers how to configure ``ssh`` to disable password-based login.

.. _This: https://linuxize.com/post/how-to-setup-passwordless-ssh-login/

Connecting to Wifi Access Point
--------------------------------

Jackal uses ``netplan`` for configuring its wired and wireless interfaces. After accessing Jackal's computer from your computer, you can configure ``netplan`` so that Jackal can connect to your own wireless network:

1. Create the file ``/etc/netplan/60-wireless.yaml``.

2. Populate the file ``/etc/netplan/60-wireless.yaml`` with the following:

.. code-block:: yaml

    network:
      wifis:
        # Replace WIRELESS_INTERFACE with the name of the wireless network device, e.g. wlan0 or wlp3s0
        # Fill in the SSID and PASSWORD fields as appropriate.  The password may be included as plain-text
        # or as a password hash.  To generate the hashed password, run
        #   echo -n 'WIFI_PASSWORD' | iconv -t UTF-16LE | openssl md4 -binary | xxd -p
        # If you have multiple wireless cards you may include a block for each device.
        # For more options, see https://netplan.io/reference/
        WIRELESS_INTERFACE:
          optional: true
          access-points:
            SSID_GOES_HERE:
              password: PASSWORD_GOES_HERE
          dhcp4: true
          dhcp4-overrides:
            send-hostname: true

3. Save the file ``/etc/netplan/60-wireless.yaml``. You will then need to apply your new ``netplan`` configuration and bring up your wireless connection. In terminal, run:

.. code-block:: bash

    sudo netplan apply

4. Verify that Jackal successfully connected to your wireless network. In terminal, run:

.. code-block:: bash

    ip a

This will show all active connections and their IP addresses, including the connection to your wireless network, and the IP address assigned to Jackal's computer.

Remote ROS2 Connection
----------------------

It is useful to connect your computer to the same ROS2 network as the Jackal, particularly if you want to interface with the Jackal through ROS2 topics, services, and actions:

1. Ensure that your computer has ROS2 Foxy installed.

2. Connect Jackal's computer and your computer to the same (wired or wireless) network.

3. Verify that Jackal's computer can ``ping`` your computer. In terminal on Jackal's computer, run:

.. code-block:: bash

    ping <YOUR_COMPUTER_IP>

4. Verify that your computer can ``ping`` Jackal's computer. In terminal on your computer, run:

.. code-block:: bash

    ping <JACKAL_COMPUTER_IP>

5. You should be able to now be able to access Jackal's ROS2 data from your computer, such as viewing Jackal's ROS2 topics. In terminal on your computer, run:

.. code-block:: bash

    source /opt/ros/foxy/setup.bash
    ros2 topic list

6. If you are unable to access Jackal's ROS2 data from your computer, make sure to set the ``ROS_DOMAIN_ID`` environment variable on your computer to the same value as on Jackal's computer. By default, ``ROS_DOMAIN_ID`` is set to 0 (in ROS2 and on Jackal), so you will not need to do this step. However, if ``ROS_DOMAIN_ID`` is set to a different value on Jackal, make sure to do the same on your computer. In terminal on your computer, run:

.. code-block:: bash

    export ROS_DOMAIN_ID=<JACKAL_ROS_DOMAIN_ID>
    source /opt/ros/foxy/setup.bash

You should now be able to access Jackal's ROS2 data (e.g. topics, services, and actions) from your computer.
