# Trajectory Visualization

The visualization of particle trajectories is an important tool for analyzing particle movements and finding movement patterns. The trajectory of a particle describes its path covered over time. This project is based on my diploma thesis on *Visualization of trajectories of ellipsoid-shaped particles* (see [project](https://franzi.pages.square-src.de/ellipsoid-trajectory-vis/))). It is written as a plugin for the [CGV framework](https://github.com/sgumhold/cgv).


## Compilation

This is a short instruction describing how to build the application from source.

### Dependencies

```bash
sudo apt-get install \
        build-essential \
        git \
        cmake \
        chrpath \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        libglew-dev \
        libxi-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        libz-dev
```
(CMake Version >= 3.8 required)


### Build Source Code

This project depends on the cgv-framework which is only developed for usage with Windows. Therefore compiling with Visual Studio is recommended.

Install the cgv-framework according to the documentation: [CGV framework on github](https://github.com/sgumhold/cgv).

After the cgv-framework is set up clone this repository and use the provided cgv-framework project file (trajectory-vis-cgv.pj) to generate the Visual Studio solution using the generator of the cgv-framework:

        - drag the trajectory-vis.pj onto <cgv>/bin/generate-makefile.bat
        - the VS-solution should open automatically otherwise it can be found under <cgv-build-dir>/vs141/trajectory-vis (VS-version may vary)
        - either build as Debug/Relaase Dll or Exe
        - for details see documentation of cgv-framework



## Data Sets

### Start Screen

The start screen show some example trajectories including particle movement without rotation in the direction of the three coordinate system axes and particle movements with rotation around one axis of the particle.


### Random Generator

Trajectory data of moving particles can be generated with the random generator. The number of trajectories and the number of time steps can be chosen using the corresponding slider in the UI. The resulting data slightly changes its velocity vector each time step and its orientation every 50 timesteps. The orientation change is split to each time step resulting in smooth orientation changes in between. (the angular velocity visualization clearly shows the cuts when a new orientation is chosen)


### Loading Data

The project provides read-functionality for the dataset of the project "Simulation of hydraulic transport of particles over rough surfaces". (Other datasets can be added by providing a suitable read-funtion.)
