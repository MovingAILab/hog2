# HOG2

## About the Project

HOG2 (Hierarchical Open Graph 2) is a collection of classes and a tile-based simulator which are designed as a simple model of RTS and other clocked simulation environments. 

Documentation (much of it older, but starting to be updated) [is available here](https://movingai.com/hog2/).

## Getting Started

To get started with the HOG2 applications run the following commands. Note that SFML and OpenGL are not required for the headless version of HOG2 (see build instructions below.)

### Prerequisites

#### Linux
On Ubuntu run the following command:

```sh
# apt install build-essential libglu1-mesa-dev freeglut3-dev libsfml-dev
```

On Debian run:

```sh
# apt install git libglu1-mesa-dev freeglut3-dev libsfml-dev
```

On Arch run:

```sh
# pacman -S git base-devel mesa glu freeglut libsfml-dev
```

On CentOs and Fedora run:

```sh
# yum install git make gcc-c++ mesa-libGL-devel mesa-libGLU-devel freeglut-devel libsfml-dev
```

#### MacOS

Download and install XCode from the App store.

#### Windows 10

TODO

### Building

To build the project on the command-line, you must first download the source code:

```sh
git clone https://github.com/nathansttt/hog2.git
```

Then traverse to the build directory with:

```sh
cd hog2/build/SFML # cd hog2/build/web for the web version
```

Finally, build the project with make:

```sh
make
```

Alternately, you can build a headless version of HOG2 (which does not require SFML or OpenGL) using:

```sh
make OPENGL=STUB
```

Note that when switching between the headless and GUI versions of HOG2 you must do a clean rebuild.

After this completes the binaries can be found under `../../bin/release/`.

To build using XCode you can open one of the projects in `build/XCode`. `HOG2 ObjC` contains a full sample application; many demos are available inside the `hog2 mac native demos` project.

### Installation

Typical research usage of HOG2 would not involve installing applications from HOG2.

To fully install the programs to /usr/local/bin, run `sudo make install` under the `hog2/build/gmake/` directory; to uninstall run `sudo make uninstall` in the same directory. The installation location can be changed with `make install prefix=</path/to/dir>`.

## Licence

HOG2 is open source software licensed under the [MIT license](LICENSE.txt)
