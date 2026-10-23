# Repository scope

- Work and commit on `develop`, never `main`, after initial repository setup.
- Keep this repository independent of Fortuz repositories; do not modify or push them.
- Target Ubuntu 22.04, ROS 2 Humble, C++17, and ament_cmake.
- The current task is scaffolding only: no full A*, robot control, or web interface.
- Build and test using the Ubuntu-22.04 WSL distribution, with `/opt/ros/humble/setup.bash` sourced.
- Never commit colcon build/, install/, or log/ output.
