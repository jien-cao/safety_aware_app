TODO: combine this with the top level README

This is the ros2 adapter for state_manager, based on:
- Project Description: Proximity Based Speed Control with Emergency Stop;
- the response document: high-level design sketch, in the top level README.

NOTE: Heavy reliance on ROS2 specifics here, in particular:
- use (TRANSIENT_LOCAL, RELIABLE) for latched topics;
- use Liveliness as a heartbeat mechanism.
- with the understanding that ROS2 optimizes for intra-process communication, IFF (below):

TODO: ensure high-cricial workers are launched using ROS2 Component Container mechanism.

MISSING FILES:
- likely use AI tools to generate boiler plate ROS2 setup.py, package.xml
- create pyproject.toml with care (leave python dependencies as an empty list, to fill in as needed)
- IDL interface declaration is needed.
