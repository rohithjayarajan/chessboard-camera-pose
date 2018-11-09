<h1 align="center"> Camera Pose Estimation </h1>
<p align="center">
<a href='https://github.com/rohithjayarajan/chessboard-camera-extrinsics/blob/master/LICENSE'><img src='https://img.shields.io/badge/License-BSD%203--Clause-blue.svg'/></a>
</p>

---

## Overview

A system for camera pose estimation relative to a chessboard in C++ which includes:

- cmake
- OpenCV

## License
```
BSD 3-Clause License

Copyright (c) 2018, Rohith Jayarajan
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

## Dependencies

- [OpenCV][reference-id-for-OpenCV]: An Open Source Computer Vision Library released under a BSD license.
A complete installation guide for OpenCV can be found [here][reference-id-for-here].

[reference-id-for-OpenCV]: https://opencv.org/
[reference-id-for-here]: https://docs.opencv.org/3.3.1/d7/d9f/tutorial_linux_install.html

## Standard install via command-line
```
git clone --recursive https://github.com/rohithjayarajan/chessboard-camera-extrinsics.git
cd <path to repository>
mkdir build
cd build
cmake ..
make
```

Run program for a given image as input: 
```
./app/pose-app <path_to_image> <chessboard_edge_length_in_meter>
```
OR

Run program for using camera data stream as input: 
```
./app/pose-app <chessboard_edge_length_in_meter>
```

Further prompts are provided to enter chessboard inner corner information.

(camera instrinsics have to be edited in CameraPose.hpp for now)

## Building for code coverage
```
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```
This generates a index.html page in the build/coverage sub-directory that can be viewed locally in a web browser.

## Working with Eclipse IDE ##

## Installation

In your Eclipse workspace directory (or create a new one), checkout the repo (and submodules)
```
mkdir -p ~/workspace
cd ~/workspace
git clone --recursive https://github.com/rohithjayarajan/chessboard-camera-extrinsics.git
```

In your work directory, use cmake to create an Eclipse project for an [out-of-source build] of chessboard-camera-extrinsics

```
cd ~/workspace
mkdir -p chessboard-camera-extrinsics-eclipse
cd chessboard-camera-extrinsics-eclipse
cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug -D CMAKE_ECLIPSE_VERSION=4.7.0 -D CMAKE_CXX_COMPILER_ARG1=-std=c++14 ../chessboard-camera-extrinsics/
```

## Import

Open Eclipse, go to File -> Import -> General -> Existing Projects into Workspace -> 
Select "chessboard-camera-extrinsics-eclipse" directory created previously as root directory -> Finish

## Edit

Source files may be edited under the "[Source Directory]" label in the Project Explorer.


## Build

To build the project, in Eclipse, unfold chessboard-camera-extrinsics-eclipse project in Project Explorer,unfold Build Targets, double click on "all" to build all projects.

## Run

1. In Eclipse, right click on the chessboard-camera-extrinsics-eclipse in Project Explorer,
select Run As -> Local C/C++ Application

2. Choose the binaries to run (e.g. psoe-app, cpp-test for unit testing)


## Debug


1. Set breakpoint in source file (i.e. double click in the left margin on the line you want 
the program to break).

2. In Eclipse, right click on the chessboard-camera-extrinsics-eclipse in Project Explorer, select Debug As -> 
Local C/C++ Application, choose the binaries to run (e.g. pose-app).

3. If prompt to "Confirm Perspective Switch", select yes.

4. Program will break at the breakpoint you set.

5. Press Step Into (F5), Step Over (F6), Step Return (F7) to step/debug your program.

6. Right click on the variable in editor to add watch expression to watch the variable in 
debugger window.

7. Press Terminate icon to terminate debugging and press C/C++ icon to switch back to C/C++ 
perspetive view (or Windows->Perspective->Open Perspective->C/C++).


## Plugins

- CppChEclipse

    To install and run cppcheck in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> cppcheclipse.
    Set cppcheck binary path to "/usr/bin/cppcheck".

    2. To run CPPCheck on a project, right click on the project name in the Project Explorer 
    and choose cppcheck -> Run cppcheck.


- Google C++ Sytle

    To include and use Google C++ Style formatter in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> Code Style -> Formatter. 
    Import [eclipse-cpp-google-style][reference-id-for-eclipse-cpp-google-style] and apply.

    2. To use Google C++ style formatter, right click on the source code or folder in 
    Project Explorer and choose Source -> Format

[reference-id-for-eclipse-cpp-google-style]: https://raw.githubusercontent.com/google/styleguide/gh-pages/eclipse-cpp-google-style.xml

- Git

    It is possible to manage version control through Eclipse and the git plugin, but it typically requires creating another project.
    
## Doxygen Documentation

To generate Doxygen Documentation,
```
cd <path to repository>
mkdir <documentation_folder_name>
cd <documentation_folder_name>
doxygen -g <config_file_name>

```
Update PROJECT_NAME and INPUT fields in the configuration file.

Then run the following command to generate the documentations,
```
doxygen <config_file_name>
```
