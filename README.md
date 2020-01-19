AdTree implements the tree reconstruction method described in the following [paper](https://3d.bk.tudelft.nl/liangliang/publications/2019/adtree/AdTree_RS-2019.pdf):
```
Shenglan Du, Roderik Lindenbergh, Hugo Ledoux, Jantien Stoter, and Liangliang Nan.
AdTree: Accurate, Detailed, and Automatic Modelling of Laser-Scanned Trees.
Remote Sensing. 2019, 11(18), 2074.
```
Please consider citing our paper if you use the code/program (or part of it).

<img src="./resource/images/AdTree.jpg" width="800"/>
<p align="center">3D Trees reconstructed from point clouds</p>

### Textured rendering (both leaves and branches) coming soon ...

### Build and Run AdTree
AdTree depends on some third-party libraries and most dependencies are included in the distribution except 
[Boost](https://www.boost.org/). So you will need to have Boost install first. 

Note: AdTree uses a stripped earlier version of [Easy3D](https://github.com/LiangliangNan/Easy3D), which is not compitable with the recent version.

You need [CMake](https://cmake.org/download/) and of course a compiler to build AdTree:

- CMake >= 3.1
- a compiler that supports `>= C++11`

AdTree has been tested on macOS (Xcode >= 8), Windows (MSVC >=2015), and Linux (GCC >= 4.8, Clang >= 3.3). Machines nowadays typically provide higher [supports](https://en.cppreference.com/w/cpp/compiler_support), so you should be able to build AdTree on almost all platforms.

There are many options to build AdTree. Choose one of the following (or whatever you prefer):

- Option 1: Use any IDE that can directly handle CMakeLists files to open the `CMakeLists.txt` in the root directory of AdTree. Then you should have obtained a usable project and just build. I recommend using [CLion](https://www.jetbrains.com/clion/) or [QtCreator](https://www.qt.io/product).
- Option 2: Use CMake to generate project files for you IDE. Then load the project to your IDE and build.
- Option 3: Use CMake to generate Makefiles and then `make` (on Linux/macOS) or `nmake`(on Windows with Microsoft Visual Studio).

This demo version provides a user interface with menus. Just clicking on the menus will do all the magic :-)
<img src="./resource/images/ui.jpg" width="600"/>

---

### Data
Some test tree point clouds are provided in the 'data' folder.

**Note:** When testing on your point clouds, please make sure that:
 - you point cloud represents a single tree (i.e., the tree is segmented out from the background; no ground, no fence...);
 - the tree has an upright orientation (i.e., with Z-axis pointing up).

---

### License
This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License or (at your option) any later version. The full text of the license can be found in the accompanying LICENSE file.

---

Should you have any questions, comments, or suggestions, please contact us at liangliang.nan@tudelft.nl

3D Geoinformation Research Group, TU Delft,

https://3d.bk.tudelft.nl,

Dec. 1, 2019
