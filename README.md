# Trajectory Visualization

The visualization of particle trajectories is an important tool for analyzing particle movements and finding movement patterns. The trajectory of a particle describes its path covered over time. This project is based on my diploma thesis on *Visualization of trajectories of ellipsoid-shaped particles* (see [project](https://git.square-src.de/franzi/ellipsoid-trajectory-vis) and [diploma thesis pdf](https://franzi.square-src.de/diploma-thesis.pdf)). It is written as a plugin for the [CGV framework](https://github.com/sgumhold/cgv) (for CMake support the [cgv fork](https://github.com/f3anaro/cgv) is used).


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

The cgv framework is automatically build by using a git submodule that uses the correct version of the cgv framework in combination with a subdirectory in cmake to build the framework alongside with the trajectory visualization plugin.

```bash
git clone https://git.square-src.de/franzi/ellipsoid-trajectory-vis.git
cd ellipsoid-trajectory-vis/
git submodule update --init --recursive

cd <project/root/path>
mkdir build/
cd build/

cmake ..
make -j

# use the launch script which handles configuration of shader paths and required plugins
../launch.sh
```

#### Optional: Create Release Application

There is a Makefile for building a `release` application with all its dependencies. The application is bundled in directory `dist/ellipsoid-trajectory-vis/`.

```bash
make dist

# Execute launch script
dist/ellipsoid-trajectory-vis/ellipsoid-trajectory-vis
```


## Data Sets

### Start Screen

The start screen show some example trajectories including particle movement without rotation in the direction of the three coordinate system axes and particle movements with rotation around one axis of the particle.


### Random Generator

Trajectory data of moving particles can be generated with the random generator. The number of trajectories and the number of time steps can be chosen using the corresponding slider in the UI. The resulting data slightly changes its velocity vector each time step and its orientation every 50 timesteps. The orientation change is split to each time step resulting in smooth orientation changes in between. (the angular velocity visualization clearly shows the cuts when a new orientation is chosen)


### Loading Data

The project provides read-functionality for the dataset of the project "Simulation of hydraulic transport of particles over rough surfaces". (Other datasets can be added by providing a suitable read-funtion.)
