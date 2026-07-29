# Durable Repository Learnings

- The host runs Ubuntu 26.04 x86_64 with ROS 2 Lyrical installed at `/opt/ros/lyrical`.
- RViz works under OpenGL 4.6 after sanitizing VS Code Snap GUI environment variables in the user's `.bashrc`.
- No physical CAN interface or USB CAN adapter was present during the 2026-07-28 inventory.
- The canonical current OpenArm v1.0 xacro has a 0.1025 m link7-to-hand transform plus a 0.0835 m hand-to-`hand_tcp` transform. Its checked-in flattened example is stale at zero TCP offset.
- OpenArm v1.0 physical arm coordinates cannot be inferred safely from CAN discovery: each unit requires a commissioned side/joint/sign/scale/zero/firmware/timeout manifest before arming.
- On this GNOME Wayland/HiDPI host, RViz/Ogre requires XWayland/GLX. Hardware GLX flickers during live resize; Mesa software OpenGL is stable enough for the OpenArm scene and is the preferred Wayland default.
- RViz should be closed through `WM_DELETE_WINDOW` before ROS receives SIGINT on this host; this avoids the observed Ogre teardown crash and lets both authored ROS nodes exit cleanly.
- DaMiao/OpenArm feedback is output-shaft encoder position in radians. The integrated motor reduction is verified configuration metadata and must not be applied again in host joint-angle conversion.
- DaMiao absolute output encoders do not reveal assembled arm side/joint/URDF zero. First-time calibration needs a known reference pose or a separately qualified, supervised mechanical-stop recipe.
