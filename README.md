# ros2 package to receive imu sendouts over wifi networks 
coverted from https://github.com/jhacsonmeza/NGIMU-OSC-Cpp

- First, go to OSC folder to build/compile the liboscpack to get the os-system appropriate libraty liboscpack.a
```sh
  mkdir build
cd build && cmake .. && make 
```
- copy the library to appropriate location in this package. 
- Use the NGIMU GUI & Network Synchorinization to configure the local network
- Configure the port to be the ones in the code. If assuming 4 IMUs, the ports in the code are 8001, 8002, 8003, 8004.
  
