# Time-Of-Flight-Based-Fill-Level-Detection-System
## Aims and Objectives
The aim of this thesis is to design a robust, low-powered, embedded system capable of measuring the fill-level of linen containers (i.e. towel usage), to aid in measuring the effectiveness of towel reuse interventions, as well as to provide hotels a measure of their towel usage. The main objectives of this thesis are:
- Develop a robust, low-power, embedded system to measure towel usage.
- Design an accurate, and precise fill-level measurement algorithm to meet an accuracy of $\pm3$ towels.
- Employ hardware such that it is adaptable and robust in new environments.

![image](https://github.com/ryanlederhose/Time-Of-Flight-Based-Fill-Level-Detection-System/assets/112144274/f2c23e5c-d32d-4a5a-9327-909abe5d1ca8)

## Hardware
This system is designed using the CS20 ToF camera provided by ![DFRobot](https://www.dfrobot.com/product-2670.html).

![image](https://github.com/ryanlederhose/Time-Of-Flight-Based-Fill-Level-Detection-System/assets/112144274/283b39e7-2653-44a7-9fbb-65906034ec8b)

## Dependencies
- Ubuntu 22.04
- Open3D @ 0.18.0
- PCL @ 1.14.0

## Build Instructions
```bash
mkdir build && cd build
cmake ..
make -j
./Volume_App
```
