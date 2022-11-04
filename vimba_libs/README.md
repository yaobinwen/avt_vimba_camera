# `vimba_libs`: Building `libVimbaCPP.so`

I hacked the `CMakeLists.txt` to build `libVimbaCPP.so` so I could debug deeper into the driver's code.

Building the code requires `/opt/ros/melodic/lib/libVimbaC.so` which you can get by installing `ros-<distro>-avt-vimba-camera`, but this should be the same file under `lib/<architecture>/libVimbaC.so`. Ideally, I should detect the system architecture and then choose the appropriate `libVimbaC.so` under `lib`.

How to build:
- `mkdir build`
- `cd build`
- `cmake ..`
- `cmake --build .`
