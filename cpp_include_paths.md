## Include paths to be added to Microsoft C/C++ Extension IntelliSense Configurations

- ${workspaceFolder}/**
- /opt/ros/humble/include/**
- ${workspaceFolder}/install/**
- /usr/include/eigen3/**
- /usr/include/pcl-1.12/**
- /usr/include/opencv4/**
- /usr/include/x86_64-linux-gnu/qt5/**

### Use this build command instead
`colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`