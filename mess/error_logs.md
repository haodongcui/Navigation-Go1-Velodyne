# !/bin/bash

# 1 

```
CMake Error at unitree_guide/unitree_guide/CMakeLists.txt:1 (cmake_minimum_required):
    CMake 3.14 or higher is required.  You are running version 3.10.2
```

这个错误非常清晰，是一个版本不匹配问题。

```bash
nano ~/catkin_ws/src/unitree_guide/unitree_guide/CMakeLists.txt
```

第一行, 3.14 -> 3.10

```txt
cmake_minimum_required(VERSION 3.14)
```

ctrl+o
ctrl+x

```bash
cd ~/catkin_ws
catkin_make
```
