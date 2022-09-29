Jackal UGV Tutorials
====================

.. image:: images/jackal_banner.png
    :alt: Jackal Robot

This package supplies Sphinx-based tutorial content to assist you with setting up and operating your Jackal_ mobile robot. The tutorials topics are listed in the left column, and presented in the suggested reading order.

.. _Jackal: https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/

.. Warning::
  These tutorials assume that you are comfortable working with `ROS2 <https://docs.ros.org/en/foxy/index.html>`_.  Also, the accessory support for ROS2 is not the same as ROS1.  Contact support if you are not confident prior to upgrading at support@clearpathrobotics.com.

:doc:`Simulation <JackalSimulation>` is a logical place for most users to start, as this is universally applicable; understanding how to effectively operate Jackal in simulation is valuable whether you are in the testing phase with software you intend to ultimately deploy on a real Jackal, or you do not have one and are simply exploring the platform's capabilities.

:doc:`Driving <JackalDriving>` covers how to teleoperate Jackal using the remote control, a well as safety procedures for operating the real robot. Anyone working with a physical robot should be familiar with this section.

The remainder of the subjects are more applicable to the real robot, and have to do with configuring, using, and maintaining the platform. If you are a lab administrator rather than direct platform user, you may wish to skip the introductory chapters and jump straight to these ones.

.. toctree::
    :maxdepth: 0
    :caption: Jackal Overview

    Introduction <self>

.. toctree::
    :maxdepth: 0
    :caption: Jackal Packages

    JackalPackages

.. toctree::
    :maxdepth: 0
    :caption: Setting Up Jackal

    ROSUpgrade
    JackalInstallRobotSoftware
    JackalInstallDesktopSoftware
    JackalNetwork
    JackalControllerPairing

.. toctree::
    :maxdepth: 0
    :caption: Using Jackal

    JackalDriving
    JackalSimulation
