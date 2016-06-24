jackal_description Package
===========================

The jackal_description package is the URDF robot description for Jackal UGV. 

.. _Source: https://github.com/jackal/jackal 


Overview
---------

This package provides a `URDF <http://wiki.ros.org/urdf>`_ model of Jackal.  For an example launchfile to use in visualizing this model, see `jacakl_viz <http://wiki.ros.org/jackal_viz>`_.

.. image:: jackal-urdf.png


Accessories 
------------

Jackal has a suite of optional payloads called accessories. These payloads can be enabled and placed on Jackal using environment variables specified at the time the `xacro <http://wiki.ros.org/xacro>`_ is rendered to URDF. Available accessory vars are:

.. raw:: html

    <table><tbody><tr>  <td><p><strong>Variable</strong> </p></td>
      <td><p><strong>Default</strong> </p></td>
      <td><p><strong>Description</strong> </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>JACKAL_LASER</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Enable primary laser scanner. </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-12"></span><p><tt>JACKAL_LASER_MOUNT</tt> </p></td>
      <td><p><tt>front</tt> </p></td>
      <td><p>Where to attach primary laser scanner on Jackal. </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-13"></span><p><tt>JACKAL_LASER_OFFSET</tt> </p></td>
      <td><p><tt>"0&nbsp;0&nbsp;0"</tt> </p></td>
      <td><p>XYZ offset from the mount. </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-14"></span><p><tt>JACKAL_LASER_RPY</tt> </p></td>
      <td><p><tt>"0&nbsp;0&nbsp;0"</tt> </p></td>
      <td><p>RPY offset from the mount (eg, to face it backward). </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-15"></span><p><tt>JACKAL_LASER_HOST</tt> </p></td>
      <td><p><tt>192.168.1.14</tt> </p></td>
      <td><p>IP address of real scanner (used by <a href="http://wiki.ros.org/jackal_bringup">jackal_bringup</a>) </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-16"></span><p><tt>JACKAL_NAVSAT</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Enable upgraded NovAtel satellite navigation receiver. </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-17"></span><p><tt>JACKAL_NAVSAT_MOUNT</tt> </p></td>
      <td><p><tt>rear</tt> </p></td>
      <td><p>Where to attach upgraded GNSS to Jackal. </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-18"></span><p><tt>JACKAL_NAVSAT_HEIGHT</tt> </p></td>
      <td><p><tt>0.1</tt> </p></td>
      <td><p>Height of GNSS receiver from mount point (metres). </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-19"></span><p><tt>JACKAL_NAVSAT</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Enable upgraded NovAtel satellite navigation receiver. </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-20"></span><p><tt>JACKAL_NAVSAT_MOUNT</tt> </p></td>
      <td><p><tt>rear</tt> </p></td>
      <td><p>Where to attach upgraded GNSS to Jackal. </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-21"></span><p><tt>JACKAL_NAVSAT_HEIGHT</tt> </p></td>
      <td><p><tt>0.1</tt> </p></td>
      <td><p>Height of GNSS receiver from mount point (metres). </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-22"></span><p><tt>JACKAL_FLEA3</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Enable a Pointgrey Flea3 camera. </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-23"></span><p><tt>JACKAL_FLEA3_MOUNT</tt> </p></td>
      <td><p><tt>front</tt> </p></td>
      <td><p>Where to attach the camera on Jackal. </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-24"></span><p><tt>JACKAL_FLEA3_OFFSET</tt> </p></td>
      <td><p><tt>"0&nbsp;0&nbsp;0"</tt> </p></td>
      <td><p>XYZ offset from the mount. </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-25"></span><p><tt>JACKAL_FLEA3_RPY</tt> </p></td>
      <td><p><tt>"0&nbsp;0&nbsp;0"</tt> </p></td>
      <td><p>RPY offset from the mount (eg, to face it backward). </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-26"></span><p><tt>JACKAL_FLEA3_TILT</tt> </p></td>
      <td><p><tt>"0.5236"</tt> </p></td>
      <td><p>The angle in radians of the camera where positive is down. </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-27"></span><p><tt>JACKAL_BB2</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Enable a Pointgrey Bumblebee2 camera. </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-28"></span><p><tt>JACKAL_BB2_MOUNT</tt> </p></td>
      <td><p><tt>front</tt> </p></td>
      <td><p>Where to attach the camera on Jackal. </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-29"></span><p><tt>JACKAL_BB2_OFFSET</tt> </p></td>
      <td><p><tt>"0&nbsp;0&nbsp;0"</tt> </p></td>
      <td><p>XYZ offset from the mount. </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-30"></span><p><tt>JACKAL_BB2_RPY</tt> </p></td>
      <td><p><tt>"0&nbsp;0&nbsp;0"</tt> </p></td>
      <td><p>RPY offset from the mount (eg, to face it backward). </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-31"></span><p><tt>JACKAL_BB2_TILT</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>The angle in radians of the camera where positive is down. </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-32"></span><p><tt>JACKAL_BB2_CALIBRATION</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>If the camera has a calibration. </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-33"></span><p><tt>JACKAL_BB2_SERIAL</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>The serial of the camera which is used for determining the calibration file name. </p></td>
    </tr>
    </tbody></table>

Configurations
----------------

As an alternative to individually specifying each accessory, some fixed configurations are provided in the package. These can be specified using the ``config arg to description.launch``, and are intended especially as a convenience for simulation launch.

====================================  ====================================================
Config:                               Description:
====================================  ====================================================
base                                  Base Jackal, includes IMU and GPS
front_laser                           Include front-facing LMS1xx LIDAR. 
front_bumblebee2                      Includes front-facing Pointgrey Bumblebee2
front_flea3                           Includes front-facing Pointgrey Flea3
====================================  ====================================================

Please see `jackal_simulator <http://wiki.ros.org/jackal_simulator>`_ for more information on simulating Jackal. 
