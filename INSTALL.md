# FIRASim - INSTALL

## Overview

We developed FIRASim on Ubuntu OS. (Ubuntu 18.04+ tested and is recommended). It is  important that the graphics card driver is installed properly (the official Ubuntu packages for nVidia and AMD(ATI) graphics cards are available). FIRASim will compile and run in 32 and 64 bits Linux, Mac OS and Windows.

FIRASim is written in C++, in order to compile it, you will need a working toolchain and a c++ compiler.

## Dependencies

FIRASim depends on:

- [CMake](https://cmake.org/) version 3.5+ 
- [OpenGL](https://www.opengl.org)
- [Qt5 Development Libraries](https://www.qt.io)
- [Open Dynamics Engine (ODE)](http://www.ode.org)
- [VarTypes Library](https://github.com/jpfeltracco/vartypes) forked from [Szi's Vartypes](https://github.com/szi/vartypes)
- [Google Protobuf 3](https://github.com/google/protobuf)
- [Boost development libraries](http://www.boost.org/) (needed by VarTypes)

**Note:** It's necessary to compile ODE in double precision. This is default when installing the ODE binaries in Ubuntu. However, if you are compiling ODE from source (e.g on Mac OS), please make sure to enable the double precision during the configuration step: `./configure --enable-double-precision`.

### Linux/Unix Installation

If you run a Debian system, or derivative, first ensure that these dependencies are there:

```bash
$ sudo apt-get install git build-essential cmake qt5-default libqt5opengl5-dev libgl1-mesa-dev libglu1-mesa-dev libprotobuf-dev protobuf-compiler libode-dev libboost-dev
```

Next compile and install VarTypes from source. In the following we install VarTypes from source using `git`.

```bash
$ cd /tmp
$ git clone https://github.com/jpfeltracco/vartypes.git
$ cd vartypes
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
```

Next, clone FIRASim into your preferred location.

```bash
$ cd /path/to/firasim_ws
$ git clone https://github.com/robocin/FIRASim.git
$ cd FIRASim
```

Create a build directory within the project (this is ignored by .gitignore):

```bash
$ mkdir build
$ cd build
```

Run CMake to generate the makefiles:

```bash
$ cmake ..
```

Then compile the program:

```bash
$ make
```

The binary is copied to the `../bin` folder after a successful compilation.

### Mac OS X Installation

Pre-requirements:

- [Xcode](https://developer.apple.com/xcode/) or Xcode Command Line Tools 8.0 or newer
- [Homebrew](http://brew.sh/) package manager.

First ensure the dependencies are there:

```bash
brew install cmake
brew tap robotology/formulae         
brew install robotology/formulae/ode 
brew install qt
brew install protobuf
```

If you run into build issues, you may need to run this first:

```bash
brew update
brew doctor
```

Next we need to install VarTypes manually. Please refer to the documentation above for the procedure. 

The steps to compile FIRASim on Mac OS is similar to the steps outlines above for Linux:


```bash
$ cd /path/to/firasim_ws
$ git clone https://github.com/robocin/FIRASim.git
$ cd ./grSim
$ mkdir build
$ cd build
$ cmake ..
$ make
```

The binary files (grSim and the sample client) will be placed in `../bin`. 


## Notes on Protobuf on older Linux 

You have to install protobuf3 in order to compile and run the FIRASim.
If the defualt option for your linux disterbution is protobuf2 please follow the below instruction to install the correct version.

### Remove old versions
```bash
  sudo rm -rf /usr/include/google/protobuf
  sudo rm -rf /usr/lib/libprotobuf*
  sudo rm -rf /usr/bin/proto*
  sudo rm -rf /usr/local/include/google/protobuf
  sudo rm -rf /usr/local/lib/libprotobuf*
  sudo rm -rf /usr/local/bin/proto*
```

### Prerequesites
```bash
sudo apt-get install autoconf automake libtool curl make g++ unzip
```


### Install protoc3
``` bash
#! /bin/bash
# Make sure you grab the latest version
curl -OL https://github.com/google/protobuf/releases/download/v3.6.1/protoc-3.6.1-linux-x86_64.zip

# Unzip
unzip protoc-3.6.1-linux-x86_64.zip -d protoc3

# Move protoc to /usr/local/bin/
sudo cp -r protoc3/bin/* /usr/local/bin/

# Move protoc3/include to /usr/local/include/
sudo cp -r protoc3/include/* /usr/local/include/

# Optional: change owner
sudo chown $USER /usr/local/bin/protoc
sudo chown -R $USER /usr/local/include/google

sudo ldconfig
```

### Install libprotobuf
```bash
# Make sure you grab the latest version
curl -OL https://github.com/protocolbuffers/protobuf/releases/download/v3.6.1/protobuf-all-3.6.1.zip

# Unzip
unzip protobuf-all-3.6.1.zip -d protobuf-all
cd protobuf-all/protobuf-3.6.1

# Installation
./configure
make -j 4
make check
sudo make install
sudo ldconfig

```


## Notes on the performance

When running FIRASim, check the FPS in the status bar. If it is running at **60 FPS** or higher, everything is ok. Otherwise check the graphics card's driver installation and OpenGL settings.

## Notes on Linux firewall

If you have some communication issues, try to allow the UDP connection to through the firewall.
Example using UFW (Uncomplicated Firewall)
```bash
sudo ufw allow 10002/udp
sudo ufw allow from 224.0.0.1
```
