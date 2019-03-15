<!-- 
 * Copyright (c) 2018 Kinova inc. All rights reserved.
 *
 * This software may be modified and distributed under the 
 * terms of the BSD 3-Clause license. 
 *
 * Refer to the LICENSE file for details.
 *
 * -->
# Kortex Examples

<!-- MarkdownTOC -->

1. [Get control loop parameters](#get-control-loop-parameters)
1. [Set control loop parameters](#set-control-loop-parameters)
1. [Play cartesian](#play-cartesian)
1. [Play cartesian position](#play-cartesian-position)
1. [Get sensor settings](#get-sensor-settings)
1. [Read all devices](#read-all-devices)

<!-- /MarkdownTOC -->


<a id="get-control-loop-parameters"></a>
## Get control loop parameters
<p>
Gets the control loop parameters from an actuator that is part of a Gen3 robot. 
</p>

To run this example, those node need to be running:
> - kortex\_device\_manager (**rosrun kortex\_device\_manager kortex\_device\_manager 192.168.1.10**)
> - kortex\_actuator\_driver (**rosrun kortex\_actuator\_driver kortex\_actuator\_driver 192.168.1.10 100**)

\* Note here that the address **192.168.1.10** is the default IP address of a robot but you can put any IP address that suits your need.

To run the example:

<code>rosrun kortex_examples GetControlLoopParameters</code>

<a id="set-control-loop-parameters"></a>
## Set control loop parameters
<p>
Sets the control loop parameters from an actuator that is part of a Gen3 robot. 
</p>

This example needs those node to be running:
> - kortex\_device\_manager (**rosrun kortex\_device\_manager kortex\_device\_manager 192.168.1.10**)
> - kortex\_actuator\_driver (**rosrun kortex\_actuator\_driver kortex\_actuator\_driver 192.168.1.10 100**)

\* Note here that the address **192.168.1.10** is the default IP address of a robot but you can put any IP address that suits your need.

<code>rosrun kortex_examples SetControlLoopParameters</code>

<a id="play-cartesian"></a>
## Play cartesian
<p>
Move the end effector of a Gen3 robot in the Z axix by +0,1 meter and rotate around the THETA Z by +60°. 
</p>

To run this example, those node need to be running:
> - kortex\_device\_manager (**rosrun kortex\_device\_manager kortex\_device\_manager 192.168.1.10**)
> - kortex\_driver (**rosrun kortex\_driver kortex\_driver 192.168.1.10 100**)

\* Note here that the address **192.168.1.10** is the default IP address of a robot but you can put any IP address that suits your need.

To run the example:

<code>rosrun kortex_examples PlayCartesian</code>

<a id="play-cartesian-position"></a>
## Play cartesian position

<p>
Move the end effector of a Gen3 robot in the Z axix by +0,1. 
</p>

To run this example, those node need to be running:
> - kortex\_device\_manager (**rosrun kortex\_device\_manager kortex\_device\_manager 192.168.1.10**)
> - kortex\_driver (**rosrun kortex\_driver kortex\_driver 192.168.1.10 100**)

\* Note here that the address **192.168.1.10** is the default IP address of a robot but you can put any IP address that suits your need.

To run the example:

<code>rosrun kortex_examples PlayCartesianPosition</code>

<a id="get-sensor-settings"></a>
## Get sensor settings
<p>
	Gets the settings of all the sensors from a vision module. In this example, we assume that the targeted vision module is part of a robot and to you want to communicate with through the robot's base using the device routing feature.
</p>

To run this example, those node need to be running:
> - kortex\_device\_manager (**rosrun kortex\_device\_manager kortex\_device\_manager 192.168.1.10**)
> - kortex\_vision\_config\_driver (**rosrun kortex\_vision\_config\_driver kortex\_vision\_config\_driver 192.168.1.10**)

Then to run it:

<code>rosrun kortex_examples GetSensorSettings</code>

<a id="read-all-devices"></a>
## Read all devices
<p>
	Get a list of all the devices available on a Gen3 robot's base. This example is useful when you want to communicate with a device that is a part of a robot using the device routin feature.
</p>

To run this example, those node need to be running:
> - kortex\_device\_manager (**rosrun kortex\_device\_manager kortex\_device\_manager 192.168.1.10**)

Then to run it:

<code>rosrun kortex_examples ReadAllDevices</code>


