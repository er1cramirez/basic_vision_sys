### Setup gazebo 8
```
# Nvidia OpenGL
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```
### Run Gazebo Sim
```

```
### Fix simulator Attitude vibrations:
In the mavproxy console run:
```
param set ATC_RAT_RLL_D 0.001
```
```
param set ATC_RAT_PIT_D 0.001
```
