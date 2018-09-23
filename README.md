# Trajectory Optimization Library

Contents:

1 Introduction

2 Setup
- 2.1 Third-Party Dependencies
- 2.2 Installation Instructions
  - 2.2.1 IPOPT Installation
  - 2.2.2 CMake Installation
  - 2.2.3 GNUPlot Installation
  - 2.2.4 MuJoCo Installation
  - 2.2.5 A `setup.sh` script for Mac-based system
  
3 Building and Running Samples
- 3.1 Building Library
- 3.2 Running Tests
- 3.3 Running Samples

4 Common Installation Issues
- 4.1 IPOPT-Related Issues
- 4.2 MuJoCo-Related Issues (when using cmake)
- 4.3 mujoco-py Issues (optional)

5 Profiling Instructions

## 1 Introduction

Trajectory optimization is a software library for robotic motion planning. The core libraries are implemented in C++. The library has been tested on Ubuntu 16.04.

## 2 Setup

### 2.1 Third-Party dependencies

1) [IPOPT](https://projects.coin-or.org/Ipopt)
2) [CMake](https://cmake.org/)
3) [GNUPlot](http://www.gnuplot.info/) (Optional, used for trajectory visualization)
4) [MuJoCo](http://www.mujoco.org)


### 2.2 Installation Instructions

#### 2.2.1 IPOPT Installation

1) Download the source code from https://www.coin-or.org/download/source/Ipopt/ , and then upack IPOPT in the home directory.
```
wget http://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.4.tgz
tar xvzf Ipopt-3.12.4.tgz
```

2) Get IPOPT third-party packages:
```
cd ~/Ipopt-3.12.4/ThirdParty/Blas
./get.Blas
cd ../Lapack
./get.Lapack
cd ../Mumps
./get.Mumps
cd ../Metis
./get.Metis
cd ../../
```
3) Compile IPOPT:
```
cd ~/Ipopt-3.12.4/
mkdir build
cd build
../configure
make -j 4 #Compile using 4 cores (if you have them) 
make install
```

#### 2.2.2 CMake Installation

1) Download the source code from https://cmake.org/download/ , choose "cmake-3.12.2.tar.gz".

2) Unpack the file in your preferred location, and then configure CMake.

```
tar -zxvf xx.tar.gz 
./bootstrap 
make 
make install
```

#### 2.2.3 GNUPlot Installation
```
sudo apt-get install gnuplot
```

#### 2.2.4 MuJoCo Installation
1) If you do not have a license, get your license key following the website https://www.roboti.us/license.html .

2) Download MuJoCo Pro 150 from MuJoCo website https://www.roboti.us/index.html , choose "mjpro150 linux".

3) Unpack the file in your preferred location (home as an example), and **copy the license to the bin folder** (~/mjpro150/bin).

4) Add the environment variable to ~/.bashrc. 

```
nano ~/.bashrc
```

```
export LD_LIBRARY_PATH=~/mjpro150/bin #your own location
export PATH="$LD_LIBRARY_PATH:$PATH"
```

```
source ~/.bashrc
```

5) Give it a try! 

```
cd ~/mjpro150/bin
simualte ../model/humanoid.xml
```

You should be able to see a humanoid model falling down.

(it is noted that the executable "simulate" was coded to search the license key file to the relative simulate path, so the key should be put into the bin folder.)


#### 2.2.5 A `setup.sh` script for Mac-based system
(A script `setup.sh` has been provided to install the first three dependencies on a Mac-based system. It will install Homebrew and use it to install the first three dependencies. To execute it, run `chmod +x setup.sh && ./setup.sh`.)

Linux support has not been added to the script yet.


## 3 Building and Running Samples

### 3.1 Building Library

TrajectoryOptimization can be easily used as a git submodule in any other CMake-based project. Let's walk through integrating TrajectoryOptimization into an existing source project.

1) cd into your project directory, `git init` if it isn't already a git repository.
2) `mkdir -p lib && git submodule add [insert this project's clone URL (https/ssh)] lib/trajectoryOptimization`
3) If you don't already use CMake, run `touch CMakeLists.txt`. Insert `cmake_minimum_required(VERSION 3.8)` as the first line.
5) Insert these into your `CMakeLists.txt` to import and link the `TrajectoryOptimization::TrajectoryOptimizationLib` target:
```
add_subdirectory(lib/trajectoryOptimization)
add_executable([yourExecutableName] [yourSourceFiles])
target_link_libraries([yourExecutableName]
  PUBLIC
    TrajectoryOptimization::TrajectoryOptimizationLib
)
```
6) `mkdir build && cd build && cmake ..`
7) `make`
8) `./[yourExecutableName]`

Now you can build software using TrajectoryOptimization!

To ever recompile and rerun, just cd into `build/` and run `make && ./[yourExecutableName]`.

### 3.2 Running Tests

To build tests, run cmake like this: `cmake -Dtraj_opt_build_tests=ON ..`. Then cd into `lib/trajectoryOptimization` and run `ctest`.

### 3.3 Running Samples

To build samples, run cmake like this: `cmake -Dtraj_opt_build_samples=ON ..`. Then cd into `lib/trajectoryOptimization` and run `./trajectoryOptimizationSample`. The sample currently optimizes a 3D trajectory. Inspect [the source](src/trajectoryOptimizationMain.cpp) for more information and to learn about usage.


## 4 Common Installation Issues

When installing or builidng executables using CMake, you may encounter one of the following issues. Please try the suggestion below.

### 4.1 IPOPT-Related Issues

1) Could not find Blas: Solution:
```
sudo apt-get install gfortran
```
2) Could not find -lgfortran: You may need to update your gcc-5 to gcc-7 instructed by the following link https://gist.github.com/jlblancoc/99521194aba975286c80f93e47966dc5 . After that, When you make executable, you may run into “could not find -lgfortran”. You can solve this issue by copying all the files that named as "\*libgfortran"  in gcc 5 folder (4 files probably) into 7.3.0 folder (or your current gcc version).

### 4.2 MuJoCo-Related Issues (when using cmake)

1) Missing libgl: If you don't have libGL.so in usr/lib/x86_64-linux-gnu/ but have it in usr/lib/x86_64-linux-gnu/mesa , you may try the following solution.
```
ln -s /usr/lib/x86_64-linux-gnu/mesa/libGL.so /usr/lib/x86_64-linux-gnu/libGL.so
```

2) Missing libglfw: Create a simbolic link libglfw.so  to libglfw.so.3 (in ~/mjpro150/bin folder), so that the missing glfw problem can be solved.

```
ln -s libglfw.so.3 libglfw.so
```
### 4.3 mujoco-py Issues (optional)

If you want to test mujoco-py from OpenAI https://github.com/openai/mujoco-py , here are a few problems you might encounter.

1) If you accidentaly upgraded pip to pip 10 or above, you may run into this error “python3-pip installed but pip3 command not found” . Solution:
```
sudo apt-get remove python3-pip
sudo apt-get install python3-pip</pre
```
2) Fatal error: GL/osmesa.h: No such file or directory #96. Solution(https://github.com/openai/mujoco-py/issues/96):
```
sudo apt-get install libosmesa6-dev
```

3) ImportError: No module named 'OpenGL'. Solution:
```
sudo apt-get install python3-opengl
```

4) ERROR: GLEW initalization error: Missing GL version. Solution: add the environment variable to ~/.bashrc. 
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia-390
```

## 5 Profiling Instructions
Tool: valgrind
