TODO: combine this with the top level README

This is the pure python implementation for worker (i.e. the "business logic"), based on:
- Project Description: Proximity Based Speed Control with Emergency Stop;
- the response document: high-level design sketch, in the top level README.

Notables:
- Build / Dependency management: use pyproject.toml (minimum cost, modern tool) + ROS2 package.xml/setup.py (simplicity);
- Individual files may still have TODO's.

MISSING FILES:
- likely use AI tools to generate boiler plate colcon setup.py, package.xml
- create pyproject.toml with care (leave python dependencies as an empty list, to fill in as needed)