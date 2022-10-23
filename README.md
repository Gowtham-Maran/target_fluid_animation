# Time Reversal and Simulation Merging for Target-Driven Fluid Animation

[![License](http://img.shields.io/:license-mit-blue.svg)](LICENSE.md)

We present an approach to control the animation of liquids. The user influences the simulation by providing a target surface which will be matched by a portion of the liquid at a specific frame of the animation; our approach is also effective for multiple target surfaces forming an animated sequence. A source simulation provides the context liquid animation with which we integrate the controlled target elements. From each target frame, we compute a target simulation in two parts, one forward and one backward, which are then joined together. The particles for the two simulations are initially placed on the target shape, with velocities sampled from the source simulation. The backward particles use velocities in the opposite direction as the forward simulation, so that the two halves join seamlessly. When there are multiple target frames, each target frame simulation is computed independently, and the particles from these multiple target simulations are later combined. In turn, the target simulation is joined to the source simulation. Appropriate steps are taken to select which particles to keep when joining the forward, backward, and source simulations. This results in an approach where only a small fraction of the computation time is devoted to the target simulation, allowing faster computation times as well as good turnaround times when designing the full animation. Source and target simulations are computed using an off-the-shelf Lagrangian simulator, making it easy to integrate our approach with many existing animation pipelines. We present test scenarios demonstrating the effectiveness of the approach in achieving a well-formed target shape, while still depicting a convincing liquid look and feel.

The code is built on C++11 and can be compiled with most of the commonly available compilers such as g++, clang++, or Microsoft Visual Studio. It currently supports macOS (10.10 or later), Ubuntu (14.04 or later), and Windows (Visual Studio 2015 or later). Other untested platforms that support C++11 also should be able to build it.

## Quick Start

You will need CMake to build the code. If you're using Windows, you need Visual Studio 2015 or 2017 in addition to CMake.

First, clone the code:

```
git clone https://github.com/Gowtham-Maran/target_fluid_animation.git --recursive
cd fluid-engine-dev
```

### C++ API

For macOS or Linux:

```
mkdir build && cd build && cmake .. && make
```

For Windows:

```
mkdir build
cd build
cmake .. -G"Visual Studio 14 2015 Win64"
MSBuild jet.sln /p:Configuration=Release
```

Now run the project:

```
bin/target_fluid_animation
```

## Examples

Here are some of the example simulations generated using Jet framework. Corresponding example codes can be found under src/examples. All images are rendered using [Mitsuba renderer](https://www.mitsuba-renderer.org/) and the Mitsuba scene files can be found from [the demo repository](https://github.com/doyubkim/fluid-engine-dev-demo/). Find out more demos from [the project website](http://fluidenginedevelopment.org/examples/).

### Source: Water Tank, Target: Stars

![FLIP Example](https://github.com/Gowtham-Maran/target_fluid_animation/blob/main/doc/img/tank_star.png "FLIP Example")

### PIC Simulation Example

![PIC Example](https://github.com/doyubkim/fluid-engine-dev/raw/main/doc/img/pic_dam_breaking.png "PIC Example")

### Level Set Example with Different Viscosity

![Level Set Example](https://github.com/doyubkim/fluid-engine-dev/raw/main/doc/img/ls_bunny_drop.png "Level Set Example ")

### Smoke Simulation with Different Advection Methods

![Cubic-smoke Example](https://github.com/doyubkim/fluid-engine-dev/raw/main/doc/img/smoke_cubic.png "Cubic-smoke Example")
![Linear-smoke Example](https://github.com/doyubkim/fluid-engine-dev/raw/main/doc/img/smoke_linear.png "Linear-smoke Example")

### Point-to-Surface Examples

![Point-to-Surface Example](https://github.com/doyubkim/fluid-engine-dev/raw/main/doc/img/point_to_surface.png "Point-to-Surface Example")

> Top-left: spherical, top-right: SPH blobby, bottom-left: Zhu and Bridson's method, and bottom-right: Anisotropic kernel

## Acknowledgement
I would like to thank my Master of Computer Science thesis supervisor Professor David Mould, for his direction and support. The inspiration for this thesis originated in a conversation between Dr Mould and Dr. Eric Paquette. I would like to thank Dr. Eric Paquette for that valuable contribution. I would like to thank Doyub Kim for his book ‘Fluid Engine Development’ and his Jet framework C++ libraries which was of great help in code implementation of this system.
