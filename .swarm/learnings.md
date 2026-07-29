# Durable Repository Learnings

- The host runs Ubuntu 26.04 x86_64 with ROS 2 Lyrical installed at `/opt/ros/lyrical`.
- RViz works under OpenGL 4.6 after sanitizing VS Code Snap GUI environment variables in the user's `.bashrc`.
- No physical CAN interface or USB CAN adapter was present during the 2026-07-28 inventory.
- The canonical current OpenArm v1.0 xacro has a 0.1025 m link7-to-hand transform plus a 0.0835 m hand-to-`hand_tcp` transform. Its checked-in flattened example is stale at zero TCP offset.
- OpenArm v1.0 physical arm coordinates cannot be inferred safely from CAN discovery: each unit requires a commissioned side/joint/sign/scale/zero/firmware/timeout manifest before arming.
