Jackal UGV Tutorials
======================

.. image:: images/jackal_banner.png
    :alt: Jackal Robot

This package supplies Sphinx-based tutorial content to assist you with setting up and operating your Jackal_
mobile robot. The tutorials topics are listed in the left column, and presented in the suggested reading order.

.. _Jackal: http://www.clearpathrobotics.com/jackal/

.. Warning::
  These tutorials assume that you are comfortable working with ROS.  We recommend starting with our
  `ROS tutorial <./../ros>`_ if you are not familiar with ROS already.

:doc:`Simulation <simulation>` is a logical place for most users to start, as this is universally applicable;
understanding how to effectively operate Jackal in simulation is valuable whether you are in the testing
phase with software you intend to ultimately deploy on a real Jackal, or you do not have one and are
simply exploring the platform's capabilities.

:doc:`Navigation <navigation>` is a follow-on to what is learned in the simulation tutorial, as navigation and
map-making may be run in the simulated environment. However, this content is applicable to both the simulator
and the real platform, if equipped with a laser scanner.

The remainder of the subjects are more applicable to the real robot, and have to do with configuring, using,
and maintaining the platform. If you are a lab administrator rather than direct platform user, you may wish to
skip the introductory chapters and jump straight to these ones.


.. toctree::
    :titlesonly:
    :hidden:
    :caption: Using Clearpath Jackal

    Overview <self>

.. toctree::
    :titlesonly:
    :hidden:
    :caption: Getting Started with Linux

    simulation
    network
    navigation
    startup
    calibration
    update
    cartographer

.. toctree::
    :maxdepth: 0
    :caption: NVIDIA Jetson

    jetson_tx2
    jetson_nano
    jetson_xavier

.. toctree::
    :titlesonly:
    :hidden:
    :caption: Getting Started with Windows

    winsetup
    winstartup
    wincalibration

.. toctree::
    :titlesonly:
    :hidden:
    :caption: Jackal Packages

    description
    additional_sim_worlds
