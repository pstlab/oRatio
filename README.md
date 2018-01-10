# oRatio

[![Build Travis-CI Status](https://travis-ci.org/riccardodebenedictis/oRatio.svg?branch=master)](https://travis-ci.org/riccardodebenedictis/oRatio)

> Dum loquimur, fugerit invida aetas: carpe diem, quam minimum credula postero. (Orazio, Odi, I, 11, 7-8)

oRatio is an Integrated Logic and Constraint based solver which takes inspiration from both Logic Programming (LP) and Constraint Programming (CP).

## Building oRatio

The basic requirements for building oRatio are:

- [Git](https://git-scm.com/)
- [CMake](https://cmake.org) v3.x
- A C++ compiler

### Building on Linux

The easiest way to install the building requirements on Ubuntu is as follows

```
sudo apt-get install build-essential
sudo apt-get install cmake
```

once the building requirements are installed, move to a desired folder and clone the oRatio repository

```
git clone https://github.com/riccardodebenedictis/oRatio
```

finally, build oRatio

```
mkdir build
cd build
cmake ..
make
```

### Building on Windows

The easiest way to compile oRatio on Windows is through [Visual Studio](https://www.visualstudio.com/). Download and install Visual Studio, download Git and CMake. Start a Visual Studio Command prompt, move to a desired folder and clone the oRatio repository

```
git clone https://github.com/riccardodebenedictis/oRatio
```

finally, build oRatio

```
mkdir build
cd build
cmake -G "NMake Makefiles" ..
nmake
```
