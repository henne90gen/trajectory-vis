#!/bin/bash
# 
# Ellipsoid Trajectory Visualization
#
# Author: Franziska Krüger
# Date: Nov 2018
#

# Get path of this script
__dir__="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

bindir="$__dir__/bin"
libdir="$__dir__/lib"
shaderdir="$__dir__/shader"

export CGV_SHADER_PATH=$shaderdir
export LD_LIBRARY_PATH=$libdir

exec $bindir/cgv_viewer \
    plugin:$libdir/libcg_fltk.so \
    plugin:$libdir/libcrg_stereo_view.so \
    plugin:$libdir/libellips_trajectory.so
