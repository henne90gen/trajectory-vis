#!/bin/bash

# gets path of this script
project_path="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

export CGV_SHADER_PATH="$project_path/build/shader;$project_path/shader"

exec $project_path/build/bin/cgv_viewer \
    plugin:$project_path/build/lib/libcg_fltk.so \
    plugin:$project_path/build/lib/libcrg_stereo_view.so \
    plugin:$project_path/build/lib/libellips_trajectory.so
