TODO: combine this with the top level README

This is the ros2 state_monitor (intended to be created by each worker, if there's more than one), based on:
- Project Description: Proximity Based Speed Control with Emergency Stop;
- the response document: high-level design sketch, in the top level README.

NOTE: Heavy reliance on ROS2 specifics here, in particular:
- use (TRANSIENT_LOCAL, RELIABLE) for latched topics;
- use dealine as a heartbeat mechanism.
- with the understanding that ROS2 optimizes for intra-process communication, IFF (below):

TODO: ensure high-cricial workers are launched using ROS2 Component Container mechanism.
