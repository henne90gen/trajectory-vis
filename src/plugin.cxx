#include <queue>
#include <fstream>

#include <cgv/base/register.h>
#include <cgv/utils/ostream_printf.h>
#include <cgv/gui/file_dialog.h>
#include <cgv_gl/gl/gl.h>
#include <cgv/gui/key_event.h>
#include <cgv/render/drawable.h>    // find_action needs this
#include <cgv/base/find_action.h>
#include <cgv/utils/file.h>
#include <cgv/gui/trigger.h>
#include "plugin.h"
#include "math_utils.h"
#include "metatube.h"

using namespace cgv::base;
using namespace cgv::gui;
using namespace cgv::signal;
using namespace cgv::render;
using namespace cgv::utils;

namespace ellipsoid_trajectory {

plugin::plugin(const char* name) : group(name) 
{
    // create new data storage
    ellips_data = new data();
    data_name = "nothing";
    stationaries_available = false;
    nr_particles = 0;
    time_steps = 0;
    time_per_step = 0.0f;

    // loading options
    scanned_time_steps = 0;
    start_load_time_step = 1;
    end_load_time_step = 1;
    time_step_resolution = 1;
    cut_trajs = true;
    split_tolerance = 0.9;
    create_equidistant = true;
    same_start = false;
    unchanged_view = false;

    // generator settings
    generator_number_trajectories = 10000;
    generator_number_time_steps = 1000;
    generator_start_velocity = 0.0f;
    generator_seed = 0;

    // visualization options
    mode = TRAJ_3D_RIBBON_GPU;
    hide_trajs = false;
    hide_b_box = false;
    hide_coord = false;
    hide_stationaries = false;

    // view and light
    scene_light = new lighting();
    view_ptr = 0;
    set_light_to_eye_pos = false;

    // ellipsoids on ticks
    display_ellipsoids = true;
    textured_ellipsoids = true;
    setup_ellipsoids = false;
    ellipsoid_tick_sample = 1;

    // velocity glyphs
    glyph_mode = LINEAR_VELOCITY;
    display_glyphs = false;
    glyph_sample = 10;
    glyph_scale_rate = 50;
    glyph_value_color = false;
    
    // time selection
    start_time = 1;
    end_time = 1;
    
    // animation
    current_time = -1;
    fps = 30;
    animate = false;
    paused = false;
    animation_speed = 5;
    time_last_frame = std::chrono::steady_clock::now();

    // trajectory selection
    single_traj_id = 0;
    display_single_traj = false;

	// trajectory export
	exact_metatube = false;

    // handling of index vectors
    out_of_date = true;
    tick_marks_sample = 10;
    restart_id = std::numeric_limits<unsigned int>::max();
   
    // length filter
    nr_visible_traj = 0;
    filter_length_active = false;
    length_filter_data.thresh_very_small = 1;
    length_filter_data.thresh_small = 10;
    length_filter_data.thresh_medium = 45;
    length_filter_data.x_very_small_traj = false;
    length_filter_data.x_small_traj = false;
    length_filter_data.x_medium_traj = true;
    length_filter_data.x_large_traj = true;
    length_filter_data.y_very_small_traj = false;
    length_filter_data.y_small_traj = false;
    length_filter_data.y_medium_traj = true;
    length_filter_data.y_large_traj = true;
    length_filter_data.z_small_traj = true;
    length_filter_data.z_very_small_traj = true;
    length_filter_data.z_medium_traj = true;
    length_filter_data.z_large_traj = true;

    // region of interest
    roi_active = false;
    roi_with_time_interval = false;
    roi_exact = true;
    roi_data.length_x_percent = 10;
    roi_data.length_y_percent = 20;
    roi_data.length_z_percent = 30;
    roi_data.pos_x_percent = 50;
    roi_data.pos_y_percent = 50;
    roi_data.pos_z_percent = 50;

    // search for points of interest
    poi_searched = false;
    poi_id = 0;
    max_pois = 100;

    // performance statistics
    perf_stats = false;
    max_indices_time = 0.;
    min_indices_time = std::numeric_limits<double>::max();
    avg_indices_time = 0.0;
    indices_time_sum = 0.0;
    indices_time_ticks = 0;
    max_gpu_time = 0.;
    min_gpu_time = std::numeric_limits<double>::max();
    avg_gpu_time = 0.0;
    gpu_time_sum = 0.0;
    gpu_time_ticks = 0;
    measured = false;

    // TODO: use framework materials
    // dynamic particle material
    ellipsoid_material.ambient = clr_type(0.8f, 0.8f, 0.8f);
    ellipsoid_material.diffuse = clr_type(0.95f, 0.95f, 1.0f);
    ellipsoid_material.specular = clr_type(0.0f, 0.0f, 0.0f);
    ellipsoid_material.shininess = 2.0f;

    // stationary particle material
    stationary_material.ambient = clr_type(0.878f, 0.878f, 0.878f);
    stationary_material.diffuse = clr_type(1.0f, 1.0f, 1.0f);
    stationary_material.specular = clr_type(0.0f, 0.0f, 0.0f);
    stationary_material.shininess = 2.0f;

    // default material for GPU ribbons
    ribbon_height = 0.1;
    ribbon_material.ambient = clr_type(0.773f, 0.773f, 0.773f);
    ribbon_material.diffuse = clr_type(1.0f, 1.0f, 1.0f);
    ribbon_material.specular = clr_type(0.0f, 0.0f, 0.0f);
    ribbon_material.shininess = 2.0f;
}

plugin::~plugin()
{
    delete ellips_data;
    delete scene_light;

    for (size_t e = 0; e < traj_renderer_ellipsoids.size(); e++) {
        delete traj_renderer_ellipsoids[e];
    }
}

void plugin::create_gui()
{
    add_decorator("","separator");

    connect_copy(add_button("Scan for Data", "color=0xffe6cc;tooltip='Scans a directory to be able to determine which files (time steps) should be loaded afterwards';")->click,rebind(this, &plugin::scan_data));

    bool load_options = false;
    if (begin_tree_node("Load Options", load_options, load_options)) {
        align("\a");

        connect_copy(
            add_control("Start File", start_load_time_step, "value_slider", 
            "min=1;max="+ std::to_string(scanned_time_steps) +";")->value_change,
            rebind(this, &plugin::changed_setting)
        );
        connect_copy(
            add_control("End File", end_load_time_step, "value_slider", 
            "min=1;max="+ std::to_string(scanned_time_steps) +";")->value_change,
            rebind(this, &plugin::changed_setting)
        );
        connect_copy(
            add_control("Step Resolution", time_step_resolution, "value_slider", 
            "min=1;max=20;tooltip='Only reads every ith file (time step). Wrong Trajectories can be caused by chossing a very low resolution since out of bound trajectories cannot be detected anymore.'")->value_change,
            rebind(this, &plugin::changed_setting)
        );

        align("\b");
        end_tree_node(load_options);
    }

    connect_copy(add_button("Load Data", "color=0xffe6cc;tooltip='Loads all files of scanned folder';")->click,rebind(this, &plugin::load_data, false));

    bool generator = false;
    if (begin_tree_node("Generator Settings", generator, generator)) {
        align("\a");
        connect_copy(
            add_control("Trajectories", generator_number_trajectories, "value_slider", 
            "min=0;max=30000;")->value_change,
            rebind(this, &plugin::changed_setting)
        );
        connect_copy(
            add_control("Time Steps", generator_number_time_steps, "value_slider", 
            "min=1;max=5000")->value_change,
            rebind(this, &plugin::changed_setting)
        );
        connect_copy(
            add_control("Start Velocity", generator_start_velocity, "value_slider", 
            "min=0;max=0.5;step=0.01;tooltip='Lets particles initially move in x direction with given amount'")->value_change,
            rebind(this, &plugin::changed_setting)
        );
        connect_copy(
            add_control("Seed", generator_seed, "value_input", 
            "tooltip='Seed used for mersenne twister. 0 is random'")->value_change,
            rebind(this, &plugin::changed_setting)
        );
        align("\b");
        end_tree_node(generator);
    }

    connect_copy(add_button("Generate Random Data", "color=0xffe6cc;tooltip='Randomly generates specified number of trajectories with given time steps in a cubic starting area (no collision detection)';")->click,rebind(this, &plugin::load_data, true));

    bool preprocess_options = true;
    if (begin_tree_node("Preprocessing Options", preprocess_options, preprocess_options)) {
        align("\a");
        connect_copy(
            add_control("split trajectories", cut_trajs, "check", 
            "value=true;tooltip='Inserts new trajectory if it went out of bounding box'")->value_change,
            rebind(this, &plugin::changed_setting)
        );
        connect_copy(
            add_control("tolerance", split_tolerance, "value_slider", 
            "min=0;max=1;step=0.01;tooltip='Set tolerance for splitting or continuing particle trajectories when particle enters bounding box again. If the particle traveled more than the given percentage of the bounding measure in one time step the trajectory will be split or continued. To high value can lead to artifacts due to missing continuation or splitting of trajectories.';")->value_change,
            rebind(this, &plugin::changed_setting)
        );
        connect_copy(
            add_control("start at origin", same_start, "check", 
            "value=true;tooltip='All trajectories start at origin. Only possible if trajectories are not split.';")->value_change,
            rebind(this, &plugin::changed_setting)
        );
        connect_copy(
            add_control("create time equidistant ticks", create_equidistant, "check", 
            "value=true;tooltip='Interpolates original loaded data to create data points that are equidistant in time';")->value_change,
            rebind(this, &plugin::changed_setting)
        );
        connect_copy(
            add_control("view unchanged", unchanged_view, "check", 
            "value=true;tooltip='Do not automatically fit view for dataset';")->value_change,
            rebind(this, &plugin::changed_setting)
        );
        align("\b");
        end_tree_node(preprocess_options);
    }

    add_decorator("","separator");
    add_decorator("Visualization Options","heading");

    connect_copy(
        add_control("Mode", mode, "dropdown",
            "enums='Line, Ribbon, 3D Ribbon, 3D Ribbon (GPU), Tube'")->value_change,
        rebind(this, &plugin::set_traj_indices_out_of_date)
    );

    bool hide_options = false;
    if (begin_tree_node("Hide Options", hide_options, hide_options)) {
        align("\a");
        connect_copy(
            add_control("Hide Trajectories", hide_trajs, "check", 
            "value=false")->value_change,
            rebind(this, &plugin::set_traj_indices_out_of_date)
        );
        connect_copy(
            add_control("Hide Bounding Box", hide_b_box, "check", 
            "value=false")->value_change,
            rebind(this, &plugin::changed_setting)
        );
        connect_copy(
            add_control("Hide Coordinate System", hide_coord, "check", 
            "value=false")->value_change,
            rebind(this, &plugin::changed_setting)
        );
        if (stationaries_available) {
            connect_copy(
                add_control("Hide Stationary Particle", hide_stationaries, "check", 
                "value=false")->value_change,
                rebind(this, &plugin::changed_setting)
            );
        }
        align("\b");
        end_tree_node(hide_options);
    }

    bool show_ellipsoid = true;
    bool ellipsoid_node = begin_tree_node("Ellipsoids", show_ellipsoid, show_ellipsoid, "level=3;options='w=140';align=' '");
    connect_copy(
        add_control("show", display_ellipsoids, "check", 
        "value=true")->value_change,
        rebind(this, &plugin::set_traj_indices_out_of_date)
    );
    if (ellipsoid_node) {
        align("\a");
        connect_copy(
            add_control("Sample Rate", ellipsoid_tick_sample, "value_slider", 
            "min=1;max=" + std::to_string(time_steps) + ";ticks=true;tooltip='Display ellipsoid at i-th time step. Always including start position except for max value.'")->value_change,
            rebind(this, &plugin::set_traj_indices_out_of_date)
        );
        connect_copy(
        add_control("textured", textured_ellipsoids, "check", 
        "value=true")->value_change,
        rebind(this, &plugin::changed_setting)
    );
        align("\b");
        end_tree_node(ellipsoid_node);
    }

    bool velocity = false;
    bool velocity_node = begin_tree_node("Velocity Visualization", velocity, velocity, "level=3;options='w=140';align=' '");
    connect_copy(
        add_control("show", display_glyphs, "check", 
        "value=true")->value_change,
        rebind(this, &plugin::set_traj_indices_out_of_date)
    );
    if (velocity_node) {
        align("\a");
        connect_copy(
            add_control("Mode", glyph_mode, "dropdown",
                "enums='Linear Velocity, Angular Velocity, Normals'")->value_change,
            rebind(this, &plugin::set_traj_indices_out_of_date)
        );
        connect_copy(
            add_control("Sample Rate", glyph_sample, "value_slider", 
            "min=1;max=" + std::to_string(time_steps) + ";ticks=true")->value_change,
            rebind(this, &plugin::set_traj_indices_out_of_date)
        );
        connect_copy(
            add_control("Scale Rate", glyph_scale_rate, "value_slider", 
            "min=0;max=100;ticks=true")->value_change,
            rebind(this, &plugin::set_traj_indices_out_of_date)
        );
        connect_copy(
            add_control("Value as Color", glyph_value_color, "toggle", 
            "tooltip='Toggles color encoding between value of velocity and direction of linear velocity vector or direction of rotation axis of angular velocity. Default encoding: direction';value=false;")->value_change,
            rebind(this, &plugin::set_velocity_color)
        );
        align("\b");
        end_tree_node(velocity_node);
    }

    add_decorator("","separator");
    add_decorator("Filter Options","heading");

    bool select_time = true;

    if (begin_tree_node("Time Selection", select_time, select_time)) {
        align("\a");
        connect_copy(
            add_control("Start", start_time, "value_slider", 
            "step=1;min=1;max=" + std::to_string(time_steps) + ";ticks=true")->value_change,
            rebind(this, &plugin::set_traj_indices_out_of_date)
        );
        connect_copy(
            add_control("End", end_time, "value_slider", 
            "step=1;min=1;max=" + std::to_string(time_steps) + ";ticks=true")->value_change,
            rebind(this, &plugin::set_traj_indices_out_of_date)
        );

        bool animation_node = begin_tree_node("Animation", animation_node, false, "level=3;options='w=120';align=' '");
        connect_copy(
            add_control("animate", animate, "check", "tooltip='Animates the current time selection.'")->value_change,
            rebind(this, &plugin::changed_setting)
        );
        if (animation_node) {
            connect_copy(
                add_control("fps", fps, "value_slider", 
                "min=1;max=60;tooltip='Set frames per second of animation. Caution: rendering takes long for many trajectories leading to missmatch of given fps number.'")->value_change,
                rebind(this, &plugin::changed_setting)
            );
            connect_copy(
                add_control("speed", animation_speed, "value_slider", 
                "min=1;max=100;tooltip='Set speed of animation (skips given number of time steps).'")->value_change,
                rebind(this, &plugin::changed_setting)
            );
            
        }
        connect_copy(
            add_control("Pause", paused, "toggle", 
            "tooltip='Pauses the animation';value=false;")->value_change,
            rebind(this, &plugin::changed_setting)
        );
        align("\b");
        end_tree_node(select_time);
    }

    if (begin_tree_node("Trajectory Selection", display_single_traj, false)) {
        align("\a");
        connect_copy(
            add_control("Show single Traj with Id:", display_single_traj, "check", 
            "value=false")->value_change,
            rebind(this, &plugin::set_traj_indices_out_of_date)
        );
        connect_copy(
            add_control("Traj Id", single_traj_id, "value_slider", 
            "min=0;max=" + std::to_string(nr_particles - 1))->value_change,
            rebind(this, &plugin::set_traj_indices_out_of_date)
        );

		// Eport
		add_control(
			"Extract exact metatube shape (very slow)", exact_metatube, "check",
			"tooltip='When disabled, a simple spherical tube with radius equal to the mean of the ellipsoid axes will be generated. When"
			" enabled, the exact shape of the metatube formed by the union of all oriented hyper-ellipsoids along the trajectory will be extracted.';"
		);
		connect_copy(add_button(
			"Export selected as tube mesh",
			"tooltip='Writes a .ply triangle mesh representing the metatube formed by the ellipsoid samples from the selected trajectory';"
		)->click,
			rebind(this, &plugin::export_metatube)
		);
		connect_copy(add_button(
			"Export all as tube mesh",
			"tooltip='Writes the metatubes formed by all the trajectories in the data set to individual .ply files.';"
		)->click,
			rebind(this, &plugin::export_all)
		);
		connect_copy(add_button(
			"Export all as .csv",
			"tooltip='Writes the trajectory samples to a .csv file containing one line per sample. The first value of each line is be the trajectory"
			" ID, followed by sample xyz position, followed by attributes (if any).';"
		)->click,
			rebind(this, &plugin::export_csv)
		);
		connect_copy(add_button(
			"Export all as .bezdat",
			"tooltip='Writes all trajectoreis to a single .bezdat file, where they are being represented as hermite splines.';"
		)->click,
			rebind(this, &plugin::export_bezdat)
		);

		align("\b");
        end_tree_node(display_single_traj);
    }

    bool filter_x = true;
    bool filter_y = true;
    bool filter_z = false;
    bool filter_thres = false;
    bool filter = begin_tree_node("Length Filter", filter, false, "level=3;options='w=140';align=' '");
    connect_copy(
        add_control("active", filter_length_active, "check", 
        "value=true")->value_change,
        rebind(this, &plugin::set_traj_indices_out_of_date)
    );
    if (filter) {
        align("\a");
        if (begin_tree_node("x direction", filter_x, filter_x)) {
            align("\a");
            connect_copy(
                add_control("very small", length_filter_data.x_very_small_traj, "check", 
                "value=false")->value_change,
                rebind(this, &plugin::set_traj_indices_out_of_date)
            );
            connect_copy(
                add_control("small", length_filter_data.x_small_traj, "check", 
                "value=false")->value_change,
                rebind(this, &plugin::set_traj_indices_out_of_date)
            );
            connect_copy(
                add_control("medium", length_filter_data.x_medium_traj, "check", 
                "value=false")->value_change,
                rebind(this, &plugin::set_traj_indices_out_of_date)
            );
            connect_copy(
                add_control("large", length_filter_data.x_large_traj, "check", 
                "value=false")->value_change,
                rebind(this, &plugin::set_traj_indices_out_of_date)
            );
            align("\b");
            end_tree_node(filter_x);
        }
        if (begin_tree_node("y direction", filter_y, filter_y)) {
            align("\a");
            connect_copy(
                add_control("very small", length_filter_data.y_very_small_traj, "check", 
                "value=false")->value_change,
                rebind(this, &plugin::set_traj_indices_out_of_date)
            );
            connect_copy(
                add_control("small", length_filter_data.y_small_traj, "check", 
                "value=false")->value_change,
                rebind(this, &plugin::set_traj_indices_out_of_date)
            );
            connect_copy(
                add_control("medium", length_filter_data.y_medium_traj, "check", 
                "value=false")->value_change,
                rebind(this, &plugin::set_traj_indices_out_of_date)
            );
            connect_copy(
                add_control("large", length_filter_data.y_large_traj, "check", 
                "value=false")->value_change,
                rebind(this, &plugin::set_traj_indices_out_of_date)
            );
            align("\b");
            end_tree_node(filter_y);
        }
        if (begin_tree_node("z direction", filter_z, filter_z)) {
            align("\a");
            connect_copy(
                add_control("very small", length_filter_data.z_very_small_traj, "check", 
                "value=false")->value_change,
                rebind(this, &plugin::set_traj_indices_out_of_date)
            );
            connect_copy(
                add_control("small", length_filter_data.z_small_traj, "check", 
                "value=false")->value_change,
                rebind(this, &plugin::set_traj_indices_out_of_date)
            );
            connect_copy(
                add_control("medium", length_filter_data.z_medium_traj, "check", 
                "value=false")->value_change,
                rebind(this, &plugin::set_traj_indices_out_of_date)
            );
            connect_copy(
                add_control("large", length_filter_data.z_large_traj, "check", 
                "value=false")->value_change,
                rebind(this, &plugin::set_traj_indices_out_of_date)
            );
            align("\b");
            end_tree_node(filter_z);
        }
        if (begin_tree_node("thresholds", filter_thres, filter_thres)) {
            align("\a");
            connect_copy(
                add_control("very small thres", length_filter_data.thresh_very_small, "value_slider", 
                "min=0;max=100;ticks=true")->value_change,
                rebind(this, &plugin::set_traj_indices_out_of_date)
            );
            connect_copy(
                add_control("small thres", length_filter_data.thresh_small, "value_slider", 
                "min=0;max=100;ticks=true")->value_change,
                rebind(this, &plugin::set_traj_indices_out_of_date)
            );
            connect_copy(
                add_control("large thres", length_filter_data.thresh_medium, "value_slider", 
                "min=0;max=100;ticks=true")->value_change,
                rebind(this, &plugin::set_traj_indices_out_of_date)
            );
            align("\b");
            end_tree_node(filter_thres);
        }
        align("\b");
        end_tree_node(filter);
    }

    bool roi_box = true;
    bool roi_node = begin_tree_node("Region of Interest", roi_box, roi_box, "level=3;options='w=140';align=' '");
    connect_copy(
        add_control("active", roi_active, "check", 
        "value=true")->value_change,
        rebind(this, &plugin::set_traj_indices_out_of_date)
    );
    if (roi_node) {
        align("\a");
        connect_copy(
            add_control("length x", roi_data.length_x_percent, "value_slider", 
            "min=0;max=100;ticks=true")->value_change,
            rebind(this, &plugin::set_traj_indices_out_of_date)
        );
        connect_copy(
            add_control("length y", roi_data.length_y_percent, "value_slider", 
            "min=0;max=100;ticks=true")->value_change,
            rebind(this, &plugin::set_traj_indices_out_of_date)
        );
        connect_copy(
            add_control("length z", roi_data.length_z_percent, "value_slider", 
            "min=0;max=100;ticks=true")->value_change,
            rebind(this, &plugin::set_traj_indices_out_of_date)
        );

        connect_copy(
            add_control("coordinate x", roi_data.pos_x_percent, "value_slider", 
            "min=0;max=100;ticks=true")->value_change,
            rebind(this, &plugin::set_traj_indices_out_of_date)
        );
        connect_copy(
            add_control("coordinate y", roi_data.pos_y_percent, "value_slider", 
            "min=0;max=100;ticks=true")->value_change,
            rebind(this, &plugin::set_traj_indices_out_of_date)
        );
        connect_copy(
            add_control("coordinate z", roi_data.pos_z_percent, "value_slider", 
            "min=0;max=100;ticks=true")->value_change,
            rebind(this, &plugin::set_traj_indices_out_of_date)
        );

        connect_copy(
            add_control("only selected time interval", roi_with_time_interval, "check", 
            "value=false")->value_change,
            rebind(this, &plugin::set_traj_indices_out_of_date)
        );

        connect_copy(
            add_control("exact", roi_exact, "check", 
            "value=false")->value_change,
            rebind(this, &plugin::set_traj_indices_out_of_date)
        );

        align("\b");
        end_tree_node(roi_box);
    }

    bool poi_box = true;
    bool poi_node = begin_tree_node("Automatic ROI", poi_box, poi_box, "level=3;options='w=140';align=' '");
    connect_copy(
        add_control("show", show_pois, "check", 
        "value=true")->value_change,
        rebind(this, &plugin::show_points_of_interest)
    );
    if (poi_node) {
        align("\a");
        connect_copy(add_button("Search", "tooltip='Searches for regions with most trajectories with current specified filter active (including ROI size, length filter and time selection)'")->click,rebind(this, &plugin::search_points_of_interest));

        connect_copy(add_control("Results", poi_id, "value_slider", 
                    "min=0;max=" + std::to_string(max_pois - 1) + ";ticks=true")->value_change,
                    rebind(this, &plugin::show_points_of_interest)
                );

        align("\b");
        end_tree_node(poi_box);
    }

    add_decorator("","separator");
    add_decorator("Additional Functionality","heading");

    connect_copy(add_button("Performance Statistics", "tooltip='Enables or resets performance statistics'")->click,rebind(this, &plugin::reset_perf_stats));

    connect_copy(
        add_control("set light source on camera", set_light_to_eye_pos, "check", 
        "value=false")->value_change,
        rebind(this, &plugin::changed_setting)
    );

    bool show_materials = false;
    bool show_ribbon_material = true;
    bool show_ellips_material = true;
    bool show_stationary_material = true;

    if (begin_tree_node("Materials", show_materials, show_materials)) {
        align("\a");
        if (begin_tree_node("3D Ribbons", show_ribbon_material, show_ribbon_material)) {
            align("\a");
            connect_copy(add_control("height", ribbon_height, "value_slider", 
                "min=0.001;max=0.3;step=0.001ticks=true")->value_change,
                rebind(this, &plugin::update_material)
            );

            connect_copy(add_control("ambient", ribbon_material.ambient, "color<float,RGB>")->value_change,
                rebind(this, &plugin::update_material));
            connect_copy(add_control("diffuse", ribbon_material.diffuse, "color<float,RGB>")->value_change,
                rebind(this, &plugin::update_material));
            connect_copy(add_control("specular", ribbon_material.specular, "color<float,RGB>")->value_change,
                rebind(this, &plugin::update_material));

            connect_copy(add_control("shiniess", ribbon_material.shininess, "value_slider", 
                "min=1;max=128;ticks=true")->value_change,
                rebind(this, &plugin::update_material)
            );

            align("\b");
            end_tree_node(show_ribbon_material);
        }

        if (begin_tree_node("Ellipsoids", show_ellips_material, show_ellips_material)) {
            align("\a");
            connect_copy(add_control("ambient", ellipsoid_material.ambient, "color<float,RGB>")->value_change,
                rebind(this, &plugin::update_material));
            connect_copy(add_control("diffuse", ellipsoid_material.diffuse, "color<float,RGB>")->value_change,
                rebind(this, &plugin::update_material));
            connect_copy(add_control("specular", ellipsoid_material.specular, "color<float,RGB>")->value_change,
                rebind(this, &plugin::update_material));

            connect_copy(add_control("shiniess", ellipsoid_material.shininess, "value_slider", 
                "min=1;max=128;ticks=true")->value_change,
                rebind(this, &plugin::update_material)
            );

            align("\b");
            end_tree_node(show_ellips_material);
        }

        if (stationaries_available) {
            if (begin_tree_node("Stationary Particles", show_stationary_material, show_stationary_material)) {
                align("\a");
                connect_copy(add_control("ambient", stationary_material.ambient, "color<float,RGB>")->value_change,
                    rebind(this, &plugin::update_material));
                connect_copy(add_control("diffuse", stationary_material.diffuse, "color<float,RGB>")->value_change,
                    rebind(this, &plugin::update_material));
                connect_copy(add_control("specular", stationary_material.specular, "color<float,RGB>")->value_change,
                    rebind(this, &plugin::update_material));

                connect_copy(add_control("shiniess", stationary_material.shininess, "value_slider", 
                    "min=1;max=128;ticks=true")->value_change,
                    rebind(this, &plugin::update_material)
                );

                align("\b");
                end_tree_node(show_stationary_material);
            }
        }

        align("\b");
        end_tree_node(show_materials);
    }
}

bool plugin::handle(cgv::gui::event& e)
{
    if (e.get_kind() == cgv::gui::EID_KEY) {
        cgv::gui::key_event& ke = static_cast<cgv::gui::key_event&>(e);
        if (ke.get_action() != cgv::gui::KA_RELEASE) {
            switch (ke.get_char()) {
            case '1':
                if (ke.get_modifiers() == 0) {
                    // front view
                    view_ptr->set_view_dir(vec3(0.0f, 0.0f, -1.0f));
                    view_ptr->set_view_up_dir(vec3(0.0f, 1.0f, 0.0f));
                    post_redraw();
                    return true;
                }
                break;
            case '2':
                if (ke.get_modifiers() == 0) {
                    // top view
                    view_ptr->set_view_dir(vec3(0.0f, -1.0f, 0.0f));
                    view_ptr->set_view_up_dir(vec3(0.0f, 0.0f, -1.0f));
                    post_redraw();
                    return true;
                }
                break;
            case '3':
                if (ke.get_modifiers() == 0) {
                    // left view
                    view_ptr->set_view_dir(vec3(1.0f, 0.0f, 0.0f));
                    view_ptr->set_view_up_dir(vec3(0.0f, 1.0f, 0.0f));
                    post_redraw();
                    return true;
                }
                break;
            case '4':
                if (ke.get_modifiers() == 0) {
                    // center focus on bounding box
                    view_ptr->set_focus(ellips_data->b_box.center);
                    post_redraw();
                    return true;
                }
                break;
            case '5':
                // TODO: use key modifiers (but not working)
                if (ke.get_modifiers() == 0) {
                    // back view
                    view_ptr->set_view_dir(vec3(0.0f, 0.0f, 1.0f));
                    view_ptr->set_view_up_dir(vec3(0.0f, 1.0f, 0.0f));
                    post_redraw();
                    return true;
                }
                break;
            case '6':
                if (ke.get_modifiers() == 0) {
                    // bottom view
                    view_ptr->set_view_dir(vec3(0.0f, 1.0f, 0.0f));
                    view_ptr->set_view_up_dir(vec3(0.0f, 0.0f, 1.0f));
                    post_redraw();
                    return true;
                }
                break;
            case '7':
                if (ke.get_modifiers() == 0) {
                    // right view
                    view_ptr->set_view_dir(vec3(-1.0f, 0.0f, 0.0f));
                    view_ptr->set_view_up_dir(vec3(0.0f, 1.0f, 0.0f));
                    post_redraw();
                    return true;
                }
                break;
            }
        }
    }
    return false;
}

void plugin::timer_event(double, double dt)
{
    if (animate && !paused) {
        auto time = std::chrono::steady_clock::now();

        // take care of specified fps number (with tolerance of 0.9)
        if ((std::chrono::duration_cast<std::chrono::duration<double>>(time - time_last_frame).count()) > 0.9 * (1.0 / (double)fps))
        {
            // set up new animation
            if (current_time < 0) {
                animate_start = start_time;
                animate_end = end_time;
                current_time = start_time;
            }

            // set next animation step or begin at start
            if (current_time < animate_end) {
                end_time = current_time;
            } else {
                current_time = start_time;
            }

            update_member(&end_time);
            set_traj_indices_out_of_date();
            post_redraw();

            current_time += animation_speed;
            time_last_frame = time;
        }

    }
}

bool plugin::init(cgv::render::context& ctx)
{
    // get view
    if (view_ptr = find_view_as_node()) {
        // initialize view
        view_ptr->set_focus(0.5, 0.5, 0.5);
        view_ptr->set_y_extent_at_focus(45.0);
        view_ptr->set_y_view_angle(45.0f);
        view_ptr->set_view_dir(vec3(0.0f, -0.6f, -0.8f));
        view_ptr->set_view_up_dir(vec3(0.0f, 0.0f, -1.0f));
    }
    
    // use white as background color
    ctx.set_bg_color(1.0f, 1.0f, 1.0f, 1.0f);

    // init renderer (except for ellipsoids and tubes which are handled in draw method)
    b_box_renderer.init(ctx);
    roi_box_renderer.init(ctx);
    coord_renderer.init(ctx);
    traj_renderer_line.init(ctx);
    traj_renderer_ribbon.init(ctx, scene_light, tick_marks_sample);
    traj_renderer_3D_ribbon.init(ctx, scene_light, ribbon_material, tick_marks_sample);
    traj_renderer_3D_ribbon_gpu.init(ctx, scene_light, ribbon_material, tick_marks_sample);
    sphere_renderer.init(ctx, scene_light, stationary_material, false);
    normal_renderer_line.init(ctx);
    velocity_renderer_line.init(ctx);
    angular_velocity_renderer_line.init(ctx);

    // enable and set restart id used in element buffer
    glPrimitiveRestartIndex(restart_id);
    glEnable(GL_PRIMITIVE_RESTART);

    // generate start screen trajectories
    ellips_data->generate_sample();
    // set up view, light ... 
    set_up_data();
    // overwrite some data from set up to better fit the start screen
    ellipsoid_tick_sample = time_steps / 5;
    view_ptr->set_y_extent_at_focus(25.0f);

    // set up openGL queries for performance stats
    // query count: 2 - GPU computation time and generated primitives
    // query buffer: for storing the results (two buffers used for swapping)
    glGenQueries(2, queryID[queryBackBuffer]);
    glGenQueries(2, queryID[queryFrontBuffer]);

    // the method timer_event will be triggered at 60hz
    connect(cgv::gui::get_animation_trigger().shoot, this, &plugin::timer_event);

    return true;
}

void plugin::init_frame(cgv::render::context& ctx)
{
}

void plugin::clear(cgv::render::context& ctx)
{
}

void plugin::scan_data()
{
    std::cout << "scans directory ... ";

    // delete previous entries
    files.resize(0);

    // get directory name
    directory_name = cgv::gui::directory_open_dialog("Data directory", "../..");
    if (!directory_name.empty())
        std::cout << directory_name << std::endl;
    else
        return;

    // filename pattern: ell_trn_<####>.bin
    std::string filter = directory_name + "/*.bin";
    std::string prefix = "ell_trn_";
    std::cout << "  .. expected file name: " << prefix << "<timestep>.bin" << std::endl;

    // search for all binary files in given folder
    void* file_handle = cgv::utils::file::find_first(filter);

    if (!file_handle)
        return;

    // store all filenames
    while (file_handle != NULL) {
        std::string file_name = cgv::utils::file::find_name(file_handle);
        file_handle = cgv::utils::file::find_next(file_handle);

        // open file and get physical time for sorting files afterwards
        uint32_t record_length;
        uint32_t s_number_particles;
        double time;
        std::ifstream file(file_name, std::ios::in | std::ios::binary);
        file.seekg(0, std::ios::beg);
        binary_read(file, record_length);
        binary_read(file, s_number_particles);
        binary_read(file, record_length);
        binary_read(file, record_length);
        binary_read(file, time);
        file.close();

        files.push_back(std::make_pair(time, directory_name + "/" + file_name));
    }

    // sort files based on time
    sort(files.begin(), files.end());

    std::cout << "  .. found " << files.size() << " files" << std::endl;

    start_load_time_step = 1;
    scanned_time_steps = files.size();
    end_load_time_step = scanned_time_steps;

    // update gui elements
    remove_all_elements();
    create_gui();
}

void plugin::set_up_data()
{
    // not every data set contains stationary variables (like the random generated data)
    if (ellips_data->stationaries.positions.size() > 0)
        stationaries_available = true;

    if (!unchanged_view) {
        // set new focus
        if (view_ptr) {
            view_ptr->set_focus(ellips_data->b_box.center);
            view_ptr->set_y_extent_at_focus(ellips_data->b_box.max[2] * 10);
        }

        // light position
        scene_light->set_light_position(vec3(ellips_data->b_box.center[0],
                                             ellips_data->b_box.max[1] + ellips_data->b_box.max[0],
                                             ellips_data->b_box.center[2]));
    }

    // 3D ribbon geometry cannot be computed for last time step
    // therefore remove it from displayable data for consistent view onto data
    time_steps = ellips_data->max_time_steps - 1;
    start_time = 1;
    end_time = time_steps;
    nr_particles = ellips_data->dynamics.axis_ids.size();

    // compute color based on time step
    time_colors.clear();
    time_colors.resize(0);

    for (size_t t = 0; t <= time_steps; t++) {
        // percental time of whole time interval
        // intensity = current_time / time_diff
        float intensity = t / (float)time_steps;

        // color transition yellow -> green -> blue
        // (1.0, 1.0, 0.0) -> (0.0, 1.0, 0.0) -> (0.0, 0.0, 1.0)
        float red = (intensity > 0.5f) ? 0.0f : (1.0f - 2 * intensity);
        float green = (intensity > 0.5f) ? (1.0f - 2 * (intensity - 0.5)) : 1.0f;
        float blue = (intensity > 0.5f) ? 2 * (intensity - 0.5) : 0.0f;

        vec3 tmp_color = vec3(0.95f, 0.95f, 0.95f) * 0.35f + vec3(red, green, blue) * 0.65f;
        vec4 color = vec4(tmp_color[0], tmp_color[1], tmp_color[2], (float)t);

        time_colors.push_back(color);
    }

    // ellipsoids and tubes needs to set up in the next draw call
    setup_ellipsoids = true;

    // reset some ui elements
    poi_searched = false;
    show_pois = false;
    animate = false;
    paused = false;
    ellipsoid_tick_sample = time_steps;
    time_per_step = (ellips_data->dynamics.times[time_steps - 1] - ellips_data->dynamics.times[0]) / (time_steps - 1);

    if (perf_stats)
        reset_perf_stats();
}

void plugin::load_data(bool generated)
{
    bool success = false;

    if (generated) {
        std::cout << "generate data" << std::endl;
        data_name = "generated data";

        // delete previously created data and load new one
        delete ellips_data;
        ellips_data = new data();

        // randomly generate data and fill ellips data structure
        success = ellips_data->generate_random(generator_number_trajectories, generator_number_time_steps, generator_start_velocity, generator_seed, cut_trajs, same_start);
    } else {
        // at least two time steps need to be loaded to generate trajectories from simulation data
        if (start_load_time_step < end_load_time_step) {
            // delete previously created data and load new one
            delete ellips_data;
            ellips_data = new data();
    
            // load data for visualization
            success = ellips_data->load(files, start_load_time_step - 1, end_load_time_step - 1, time_step_resolution, cut_trajs, same_start, create_equidistant, split_tolerance);
        } else {
            std::cerr << "Data loading failed: start time < end time expected" << std::endl;
        }
    }

    if (success) {
        // reset all renderer
        // except for ellipsoids and tubes (their are handle in the draw call)
        coord_renderer.reset();
        sphere_renderer.reset();
        traj_renderer_line.reset();
        traj_renderer_ribbon.reset();
        traj_renderer_3D_ribbon.reset();
        traj_renderer_3D_ribbon.reserve_memory(ellips_data->dynamics.axis_ids.size(),
                                               ellips_data->max_time_steps);
        traj_renderer_3D_ribbon_gpu.reset();
        b_box_renderer.reset();
        roi_box_renderer.reset();
        normal_renderer_line.reset();
        velocity_renderer_line.reset();
        angular_velocity_renderer_line.reset();

        // sets light, view point and time encoding as color
        set_up_data();

        // update gui elements
        remove_all_elements();
        create_gui();

        // indices of ebos have changed too
        compute_traj_indices();

        // draw
        post_redraw();

        // store name of current data set
        size_t index = directory_name.rfind("/") + 1;
        if (index == directory_name.length()) {
            // string "/data-name/" (ended with /)
            index = directory_name.rfind("/", directory_name.length() - 2) + 1;
            data_name = directory_name.substr(index, directory_name.length() - index);
        } else {
            // string "/data-name"
            data_name = directory_name.substr(index, directory_name.length() - index);  
        }
    }
}

void plugin::draw(context& ctx) {
    // set light position to camera position if enabled
    if (set_light_to_eye_pos) {
        dvec3 _eye = view_ptr->get_eye();
        scene_light->set_light_position(vec3((float)_eye[0],(float)_eye[1],(float)_eye[2]));
    }

    // query statistics from GPU if enabled
    if (perf_stats) {
        glBeginQuery(GL_TIME_ELAPSED, queryID[queryBackBuffer][0]);
        glBeginQuery(GL_PRIMITIVES_GENERATED, queryID[queryBackBuffer][1]);
    }

    // reset and initialize new ellipsoid and tube renderer if necessary (after loading)
    if (setup_ellipsoids) {
        // delete old ellipsoids renderer
        for (size_t e = 0; e < traj_renderer_ellipsoids.size(); e++) {
            delete traj_renderer_ellipsoids[e];
        }

        // initialize new ellipsoids
        traj_renderer_ellipsoids.resize(ellips_data->axes.size());
        for (size_t e = 0; e < ellips_data->axes.size(); e++) {
            traj_renderer_ellipsoids[e] = new ellipsoid_instanced_renderer();
            traj_renderer_ellipsoids[e]->init(ctx, scene_light, ellipsoid_material, true);
        }

        // delete old tube renderer
        for (size_t e = 0; e < traj_renderer_tubes.size(); e++) {
            delete traj_renderer_tubes[e];
        }

        // initialize new tubes
        traj_renderer_tubes.resize(ellips_data->axes.size());
        for (size_t e = 0; e < ellips_data->axes.size(); e++) {
            traj_renderer_tubes[e] = new traj_tube_renderer();
            traj_renderer_tubes[e]->init(ctx, scene_light, ellipsoid_material);
        }

        setup_ellipsoids = false;
    }
   
    // update index vectors if necessary (if filter are applied etc)
    if (out_of_date) {
        auto time_measure_start = std::chrono::system_clock::now();

        // compute new visible data
        compute_traj_indices();

        auto time_measure_end = std::chrono::system_clock::now();
        indices_time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1,1000>>>(time_measure_end - time_measure_start);

        if (indices_time.count() < min_indices_time)
            min_indices_time = indices_time.count();
        if (indices_time.count() > max_indices_time)
            max_indices_time = indices_time.count();

        indices_time_sum += indices_time.count();
        indices_time_ticks++;
        avg_indices_time = indices_time_sum / indices_time_ticks;
    }

    glEnable(GL_CULL_FACE);

    if (!hide_coord)
        render_coordinate_system(ctx);

    if (!hide_b_box)
        render_bounding_box(ctx);

    if (stationaries_available && !same_start && !hide_stationaries)
        render_stationary_particles(ctx);

    if (roi_active)
        render_roi_box(ctx);

    if (!hide_trajs) {
        if (mode == TRAJ_LINE)
            render_trajectory_lines(ctx);

        if (mode == TRAJ_RIBBON)
            render_trajectory_ribbons(ctx);

        if (mode == TRAJ_3D_RIBBON)
            render_trajectory_3D_ribbons(ctx);

        if (mode == TRAJ_3D_RIBBON_GPU)
            render_trajectory_3D_ribbons_gpu(ctx);

        if (mode == TRAJ_TUBE)
            render_trajectory_tubes(ctx);
    }
    
    if (display_glyphs) {
        if (glyph_mode == LINEAR_VELOCITY)
            render_velocities(ctx);

        if (glyph_mode == ANGULAR_VELOCITY)
            render_angular_velocities(ctx);

        if (glyph_mode == NORMALS)
            render_normals(ctx);
    }

    if (display_ellipsoids)
        render_ellipsoids(ctx);


    // clear index vectors to release unneeded memory
    if (out_of_date) {
        clear_traj_indices();
        out_of_date = false;
    }
    
    if (perf_stats) {
        glEndQuery(GL_TIME_ELAPSED);
        glEndQuery(GL_PRIMITIVES_GENERATED);
        glGetQueryObjectui64v(queryID[queryFrontBuffer][0], GL_QUERY_RESULT, &elapsed_time);
        glGetQueryObjectui64v(queryID[queryFrontBuffer][1], GL_QUERY_RESULT, &gen_prims);
        swapQueryBuffers();

        // gpu from nanoseconds to milliseconds
        gpu_time = (double)elapsed_time / 10.0e6;

        if (gpu_time < min_gpu_time)
            min_gpu_time = gpu_time;
        if (gpu_time > max_gpu_time)
            max_gpu_time = gpu_time;

        gpu_time_sum += gpu_time;
        gpu_time_ticks++;
        avg_gpu_time = gpu_time_sum / gpu_time_ticks;
    }

    glDisable(GL_CULL_FACE);
}

void plugin::export_metatube (unsigned traj_id, bool implicit_progress)
{
	// Convenience shortcut to selected trajectory and corresponding ellipsoid axes
	auto traj = ellips_data->dynamics.trajs[traj_id];
	const vec3 &axes = ellips_data->axes[ellips_data->dynamics.axis_ids[traj_id]];


	////
	// Extract mesh

	// Setup surface extraction
	// - setup metatube implicit function with trajectory data
	metatube<float> tube(
		traj->positions, traj->orientations, ellips_data->axes[ellips_data->dynamics.axis_ids[traj_id]]
	);
	// - setup extraction handler (to output vertices as points on the fly - for now)
	surface_extraction_handler<float> handler;
	// - setup mesh extraction module
	cgv::media::mesh::dual_contouring<float, float> extractor(tube, &handler, 0.01f, 8, 0);
	handler.mesh = &extractor;

	// Determine sampling grid
	// - resolution
	#ifdef _DEBUG
		#define DC_SAMPLING_RES_TARGET 8
	#else
		#define DC_SAMPLING_RES_TARGET 8
	#endif
	const unsigned min_extend_id = get_ellipsoid_min_axis_id(axes),
	               mid_extend_id = get_ellipsoid_mid_axis_id(axes),
	               max_extend_id = get_ellipsoid_max_axis_id(axes);
	handler.cellsize = axes[min_extend_id]*2.0f / float(DC_SAMPLING_RES_TARGET);
	// - sampling domain (do we even need this?)
	cgv::media::axis_aligned_box<float, 3> domain = discretize(traj->b_box, handler.cellsize);

	// Estimate if tube is too flat for 1st-order MLS-approximant
	bool use_mls = exact_metatube && ((axes[max_extend_id] < axes[min_extend_id]*2.5f) ? true : false);

	// Scan-convert (kind of) the tube
	if (use_mls) for (unsigned i=0; i<traj->positions.size(); /* no implicit increment */)
	{
		// Setup current sampling area
		i = tube.set_window(i, handler.cellsize);

		// Adapt resolution to current sampling area so that the grid cell size remains
		// the same
		unsigned resx = unsigned(tube.bbox_sampling.get_extent().x() / handler.cellsize),
		         resy = unsigned(tube.bbox_sampling.get_extent().y() / handler.cellsize),
		         resz = unsigned(tube.bbox_sampling.get_extent().z() / handler.cellsize);

		// Extract metatube iso surface in current sampling area
		extractor.extract(0, tube.bbox_sampling, resx, resy, resz, implicit_progress);
	}

	//// [DEBUG] ////
	/*if (use_mls)
	{
		std::stringstream fn;
		fn << "traj_" << std::setfill('0') << std::setw(6) << traj_id << "_pcloud.obj";
		std::ofstream file(fn.str());
		handler.write_obj(file);
	}*/
	//// [/DEBUG] ///

	// Tesselate by moving along trajectory and projecting onto MLS surface defined by
	// extracted iso surface samples
	// - init
	point_set_surface mls(handler.points, 5);
	std::vector<vec3> verts, nrmls;
	verts.reserve(unsigned(float(handler.points.size())*0.75f));
	nrmls.reserve(verts.size());
	// - main loop: move along trajectory
	float extrusion = (axes[max_extend_id] + axes[mid_extend_id] + axes[min_extend_id]) / 3.0f;
	unsigned i_last = 0, seg_counts = 0;
	for (unsigned i=1; i<traj->positions.size(); i++)
	{
		// Convencience shortcuts
		unsigned &c = seg_counts;
		const vec3 &pos = traj->positions[i];

		// Move forward
		vec3 travel_dir = pos - traj->positions[i_last];
		if (travel_dir.length() >= extrusion/2.0f)
		{
			// Increase segment counter
			c++;

			// Create points of a circle on current cross-section plane at current sample
			// - normalize travel_dir to double as plane normal
			travel_dir.normalize();
			// - check if sample 0 was already handled
			#define DC_TESS_SECTIONS 12
			if (i_last == 0)
			{
				const vec3 &pos0 = traj->positions[0];
				// - establish coordinate frame on cross-section plane
				vec3 tmp = plane_project(vec3(0,0,0), travel_dir, pos0),
				     refx = cgv::math::normalize(tmp - pos0),
				     refy = cgv::math::cross(travel_dir, refx);
				// - circle around the travel direction vector
				for (unsigned j=0; j<DC_TESS_SECTIONS; j++)
				{
					const float
						param = float(M_PI)*float(2*j)/float(DC_TESS_SECTIONS), // parametrization
						u = std::cos(param), v = std::sin(param);               // of 2D circle
					vec3 extrusion_dir = u*refx + v*refy,
					     extruded_vert = pos0 + extrusion*extrusion_dir;
					if (use_mls)
						verts.push_back(mls.projectPoint(extruded_vert, &extrusion_dir));
					else
						verts.push_back(extruded_vert);
					nrmls.push_back(extrusion_dir);
				}
			}
			// - establish coordinate frame on cross-section plane
			vec3 ref  = verts[(c-1)*DC_TESS_SECTIONS],
			     tmp  = plane_project(ref, travel_dir, pos),
			     refx = cgv::math::normalize(tmp-pos),
			     refy = cgv::math::cross(travel_dir, refx);
			// - circle around the travel direction vector
			for (unsigned j=0; j<DC_TESS_SECTIONS; j++)
			{
				const float
					param = float(M_PI)*float(2*j)/float(DC_TESS_SECTIONS), // parametrization
					u = std::cos(param), v = std::sin(param);               // of 2D circle
				vec3 extrusion_dir = u*refx + v*refy,
				     extruded_vert = pos + extrusion*extrusion_dir;
				if (use_mls)
					verts.push_back(mls.projectPoint(extruded_vert, &extrusion_dir));
				else
					verts.push_back(extruded_vert);
				nrmls.push_back(extrusion_dir);
			}
			// - triangulate
			for (unsigned j=1; j<DC_TESS_SECTIONS+1; j++)
			{
				handler.triangles.push_back(triangle(
					(c-1)*DC_TESS_SECTIONS + (j-1),
					(c-1)*DC_TESS_SECTIONS +  j%DC_TESS_SECTIONS,
					  c  *DC_TESS_SECTIONS + (j-1)
				));
				handler.triangles.push_back(triangle(
					  c  *DC_TESS_SECTIONS + (j-1),
					(c-1)*DC_TESS_SECTIONS +  j%DC_TESS_SECTIONS,
					  c  *DC_TESS_SECTIONS +  j%DC_TESS_SECTIONS
				));
			}
			// - update state
			i_last = i;
		}
	}
	// - end cap, trailing
	#define DC_TESS_ENDCAPS_VCOUNT 1
	vec3 endcap_extrusion(std::move(
		cgv::math::normalize(std::move(
			traj->positions[0] - traj->positions[1]
		)))
	);
	unsigned last_idx = (unsigned)verts.size();
	verts.push_back(std::move(traj->positions[0] + endcap_extrusion*extrusion));
	nrmls.push_back(std::move(endcap_extrusion));
	for (unsigned j=1; j<=DC_TESS_SECTIONS; j++)
		handler.triangles.push_back(triangle(
			last_idx, j%DC_TESS_SECTIONS, j-1
		));
	// - end cap, leading
	endcap_extrusion = std::move(
		cgv::math::normalize(std::move(
			traj->positions.back() - traj->positions[traj->positions.size()-2]
		))
	);
	last_idx = (unsigned)verts.size();
	verts.push_back(std::move(traj->positions.back() + endcap_extrusion*extrusion));
	nrmls.push_back(std::move(endcap_extrusion));
	for (unsigned j=1; j<=DC_TESS_SECTIONS; j++)
		handler.triangles.push_back(triangle(
			last_idx,
			seg_counts*DC_TESS_SECTIONS + j-1,
			seg_counts*DC_TESS_SECTIONS + j%DC_TESS_SECTIONS
		));
	// - replace with new tesselation
	handler.points = std::move(verts); handler.normals = std::move(nrmls);


	////
	// Write .obj and .ply files

	// Filename template
	std::stringstream filename;
	filename << "traj_" << std::setfill('0') << std::setw(6) << traj_id;
	std::ofstream
		//objfile(filename.str() + ".obj"),
		plyfile(filename.str() + ".ply");

	// Write mesh
	//handler.write_obj(objfile);
	handler.write_ply(plyfile);
}

void plugin::export_metatube (void)
{
	export_metatube(single_traj_id, true);
}

void plugin::export_all(void)
{
	std::cout << std::endl << std::endl << std::endl << "[BEGIN] METATUBE EXTRACTION" << "===========================";

	//#pragma omp parallel for schedule(dynamic)
	for (signed i=0; unsigned(i)<ellips_data->dynamics.trajs.size(); i++)
	{
		std::cout << std::endl << std::endl << "Traj #" << i << std::endl;
		#ifdef _DEBUG
			export_metatube(i, true);
		#else
			export_metatube(i, false);
		#endif
	}

	std::cout << std::endl << std::endl << "[END] METATUBE EXTRACTION" << "========================="
	          << std::endl << std::endl << std::endl;
}

void plugin::export_csv(void)
{
	// Convenience shorthands
	auto &trajs = ellips_data->dynamics.trajs;

	// Generate filename
	std::stringstream filename;
	filename << "trajectories_" << generator_seed << trajs.size() << trajs[0]->positions.size() << ".csv";
	std::cout << std::endl << "Exporting trajectories to file '" << filename.str() << "'...";

	// Write data
	std::ofstream csvfile(filename.str());
	// - header
	csvfile << "traj_id,pos_x,pos_y,pos_z,radius" << std::endl;
	// - samples
	for (unsigned t=0; t<trajs.size(); t++)
	{
		const auto &traj = trajs[t];
		const auto &axes = ellips_data->axes[ellips_data->dynamics.axis_ids[t]];
		const auto radius = (
			  axes[get_ellipsoid_min_axis_id(axes)]
			+ axes[get_ellipsoid_mid_axis_id(axes)]
			+ axes[get_ellipsoid_max_axis_id(axes)]
		) / 3.0f;

		for (const auto &pos : traj->positions)
			csvfile << t << "," << pos.x() << "," << pos.y() << "," << pos.z() << "," << radius << std::endl;
	}

	// Done!
	std::cout << " Done!" << std::endl << std::endl;
}

void plugin::export_bezdat(void)
{
	// Hermite->Bezier basis transform
	constexpr auto _1o3(mat4::value_type(1.0/3.0));
	struct hermite_interpolation_helper
	{
		const mat4::value_type data[16] = {
			1, 0,     0,    0,   // transposed because we use column
			1, _1o3,  0,    0,   // vectors and cgv::math::fmat is
			0, 0,    -_1o3, 1,   // column major
			0, 0,     0,    1    //
		};
		const mat4& h2b(void) const	{ return *((mat4*)data); }
	};
	static const hermite_interpolation_helper helper;

	// Convenience shorthands
	auto &trajs = ellips_data->dynamics.trajs;

	// Generate filename
	std::stringstream filename;
	filename << "trajectories_" << generator_seed << trajs.size() << trajs[0]->positions.size() << ".bezdat";
	std::cout << std::endl << "Exporting trajectories to file '" << filename.str() << "'...";

	// Write data
	std::ofstream bzdfile(filename.str());
	// - header
	bzdfile << "BezDatA 1.0" << std::endl;
	// - samples
	unsigned long long points_written = 0;
	for (unsigned t=0; t<trajs.size(); t++)
	{
		const auto &traj = trajs[t];
		const auto &axes = ellips_data->axes[ellips_data->dynamics.axis_ids[t]];
		const auto radius = (
			  axes[get_ellipsoid_min_axis_id(axes)]
			+ axes[get_ellipsoid_mid_axis_id(axes)]
			+ axes[get_ellipsoid_max_axis_id(axes)]
		) / 3.0f;

		// Control tangent data
		unsigned N = traj->positions.size();
		std::vector<vec3> m(N); std::vector<vec3::value_type> len(N);

		// First control point and tangent
		m[0] = std::move(traj->positions[1] - traj->positions[0]);
		len[0] = m[0].length();
		m[0] /= 2;

		// 2nd to (N-1)-th control points and tangents
		for (unsigned p=1; p<N-1; p++)
		{
			m[p] = std::move(
				  std::move(traj->positions[p]   - traj->positions[p-1])
				+ std::move(traj->positions[p+1] - traj->positions[p])
			);
			len[p] = m[p].length();
			m[p] /= 2;
		}

		// Last control point and tangent
		m[N-1] = std::move(traj->positions.back() - traj->positions[N-2]);
		len[N-1] = m[N-1].length();
		m[N-1] /= 2;

		// Hermite interpolation
		for (unsigned p=0; p<N-1; p++)
		{
			// Setup as hermite control matrix
			cgv::math::fmat<mat4::value_type, 3, 4> M;
			M.set_col(0, traj->positions[p]);
			M.set_col(1, m[p]);
			M.set_col(2, m[p+1]);
			M.set_col(3, traj->positions[p+1]);

			// Convert to bezier patch
			M = std::move( M * helper.h2b() );

			// Write patch control points to file
			for (unsigned i=0; i<3; i++)
				bzdfile
					// item type
					<< "PT "
					// position
					<< M.col(i).x() <<" "<< M.col(i).y() <<" "<< M.col(i).z() <<" "
					// attributes
					<< radius <<" "<< "128 128 128"
					// newline
					<< std::endl;
		}
		// Last point
		bzdfile
			// item type
			<< "PT "
			// position
			<< traj->positions.back().x() <<" "<< traj->positions.back().y() <<" "<<
			   traj->positions.back().z() <<" "
			// attributes
			<< radius << " " << "128 128 128"
			// newline
			<< std::endl;

		// Write control point connectivity to file
		for (unsigned p=0; p<N-1; p++)
			bzdfile
				// item type
				<< "BC "
				// 1st control point index
				<< points_written + p*3 <<" "
				// 2nd control point index
				<< points_written + p*3 + 1 <<" "
				// 3rd control point index
				<< points_written + p*3 + 2 <<" "
				// 4th control point index
				<< points_written + p*3 + 3 <<" "
				// newline
				<< std::endl;

		// Update start index for next trajectory
		points_written += (N-1)*3 + 1;
	}

	// Done!
	std::cout << " Done!" << std::endl << std::endl;
}

void plugin::render_coordinate_system(cgv::render::context& ctx)
{
    if (coord_renderer.initial) {
        std::cout << "Set up coordinate system ... ";
        std::vector<vec3> vertices;
        vertices.push_back(vec3(0.0f, 0.0f, 0.0f));
        vertices.push_back(vec3(5.0f, 0.0f, 0.0f));
        vertices.push_back(vec3(0.0f, 5.0f, 0.0f));
        vertices.push_back(vec3(0.0f, 0.0f, 5.0f));

        std::vector<vec4> colors;
        colors.push_back(vec4(1.0f, 1.0f, 1.0f, 1.0f));     // white
        colors.push_back(vec4(1.0f, 0.0f, 0.0f, 1.0f));     // red
        colors.push_back(vec4(0.0f, 1.0f, 0.0f, 1.0f));     // green
        colors.push_back(vec4(0.0f, 0.0f, 1.0f, 1.0f));     // blue

        std::vector<unsigned int> indices;
        indices.push_back(0);
        indices.push_back(1);
        indices.push_back(restart_id);
        indices.push_back(0);
        indices.push_back(2);
        indices.push_back(restart_id);
        indices.push_back(0);
        indices.push_back(3);

        // set all buffers needed for rendering lines
        coord_renderer.set_buffers(ctx, vertices, colors, indices);

        coord_renderer.initial = false;
        std::cout << "finished" << std::endl;
    }

    // all data already transfered to GPU
    // draw with current view
    coord_renderer.draw(ctx);
}

void plugin::render_trajectory_lines(cgv::render::context& ctx)
{
    // set all vertex data once
    if (traj_renderer_line.initial) {
        std::cout << "Set up trajectory lines ... ";
        std::vector<vec3> positions;
        std::vector<vec4> colors;

        // set traj data
        for (size_t p = 0; p < ellips_data->dynamics.trajs.size(); p++) {
            positions.insert(positions.end(),
                             ellips_data->dynamics.trajs[p]->positions.begin(),
                             ellips_data->dynamics.trajs[p]->positions.end());
            colors.insert(colors.end(),
                          time_colors.begin(),
                          time_colors.end());
        }

        traj_renderer_line.set_buffers(ctx, positions, colors, *traj_indices_strip);

        traj_renderer_line.initial = false;
        std::cout << " finished" << std::endl;
    } else if (out_of_date) {
        traj_renderer_line.update_element_buffer(*traj_indices_strip);
    }

    // all data already transfered to GPU
    // draw with current view
    traj_renderer_line.draw(ctx);
}

void plugin::render_trajectory_ribbons(cgv::render::context& ctx)
{
    // set all vertex data once
    if (traj_renderer_ribbon.initial) {
        std::cout << "Set up trajectory ribbons ... ";

        // compute vertices
        std::vector<vec3> vertices;
        std::vector<vec4> new_colors;
        for (size_t p = 0; p < ellips_data->dynamics.trajs.size(); p++) {
            traj_renderer_ribbon.create_vertices(vertices, new_colors,
                                                 ellips_data->dynamics.trajs[p]->positions,
                                                 ellips_data->axes[ellips_data->dynamics.axis_ids[p]],
                                                 ellips_data->dynamics.trajs[p]->orientations,
                                                 time_colors);
        }

        // need to be called here too since the indices-vector was just filled yet
        compute_traj_indices();
        traj_renderer_ribbon.set_buffers(ctx, vertices, new_colors, *traj_ribbon_indices);

        traj_renderer_ribbon.initial = false;
        std::cout << " finished" << std::endl;
    } else if (out_of_date) {
        traj_renderer_ribbon.update_element_buffer(*traj_ribbon_indices);
    }

    // all data already transfered to GPU
    // draw with current view
    traj_renderer_ribbon.draw(ctx);
}

void plugin::render_trajectory_3D_ribbons(cgv::render::context& ctx)
{
    // set all vertex data once
    if (traj_renderer_3D_ribbon.initial) {
        std::cout << "Set up trajectory 3D ribbons (GPU)... ";
        std::vector<vec4> colors;

        // compute color
        for (size_t p = 0; p < ellips_data->dynamics.trajs.size(); p++) {
            colors.insert(colors.end(),
                          time_colors.begin(),
                          time_colors.end());
        }

        // compute vertices
        std::vector<vec3> vertices;
        std::vector<vec4> new_colors;
        std::vector<vec3> normals;

        vertices.reserve(ellips_data->dynamics.axis_ids.size() * ellips_data->max_time_steps * 2);
        new_colors.reserve(ellips_data->dynamics.axis_ids.size() * ellips_data->max_time_steps * 2);
        normals.reserve(ellips_data->dynamics.axis_ids.size() * ellips_data->max_time_steps * 2);


        for (size_t p = 0; p < ellips_data->dynamics.trajs.size(); p++) {
            traj_renderer_3D_ribbon.create_vertices(vertices, new_colors, normals,
                                                    ellips_data->dynamics.trajs[p]->positions,
                                                    vec3(ellips_data->axes[ellips_data->dynamics.axis_ids[p]][0], 0.0f, 0.0f),
                                                    ellips_data->dynamics.trajs[p]->main_axis_normals,
                                                    ellips_data->dynamics.trajs[p]->orientations,
                                                    colors);
        }

        // need to be called here too since the indices-vector was just filled yet
        compute_traj_indices();

        traj_renderer_3D_ribbon.set_buffers(ctx, vertices, new_colors, normals, *traj_3D_ribbon_indices);

        traj_renderer_3D_ribbon.initial = false;
        std::cout << " finished" << std::endl;
    } else if (out_of_date) {
        traj_renderer_3D_ribbon.update_element_buffer(*traj_3D_ribbon_indices);
    }

    // all data already transfered to GPU
    // draw with current view
    traj_renderer_3D_ribbon.draw(ctx);
}

void plugin::render_trajectory_3D_ribbons_gpu(cgv::render::context& ctx)
{
    // set all vertex data once
    if (traj_renderer_3D_ribbon_gpu.initial) {
        std::cout << "Set up trajectory 3D ribbons (GPU)... ";
        std::vector<vec3> positions;
        std::vector<vec4> orientations;
        std::vector<vec4> colors;
        std::vector<vec3> axes;
        std::vector<vec3> normals;

        // set traj data
        for (size_t p = 0; p < ellips_data->dynamics.trajs.size(); p++) {
            positions.insert(positions.end(),
                             ellips_data->dynamics.trajs[p]->positions.begin(),
                             ellips_data->dynamics.trajs[p]->positions.end());
            orientations.insert(orientations.end(),
                                ellips_data->dynamics.trajs[p]->orientations.begin(),
                                ellips_data->dynamics.trajs[p]->orientations.end());
            normals.insert(normals.end(),
                           ellips_data->dynamics.trajs[p]->main_axis_normals.begin(),
                           ellips_data->dynamics.trajs[p]->main_axis_normals.end());
            colors.insert(colors.end(),
                          time_colors.begin(),
                          time_colors.end());

            // find largest axis
            float axis_max = 0.0f;
            vec3 main_axis = vec3(ellips_data->axes[ellips_data->dynamics.axis_ids[p]][0], 0.0f, 0.0f);
            if (ellips_data->axes[ellips_data->dynamics.axis_ids[p]][0] > axis_max){
                axis_max = ellips_data->axes[ellips_data->dynamics.axis_ids[p]][0];
                main_axis = vec3(ellips_data->axes[ellips_data->dynamics.axis_ids[p]][0], 0.0f, 0.0f);
            }
            if (ellips_data->axes[ellips_data->dynamics.axis_ids[p]][1] > axis_max){
                axis_max = ellips_data->axes[ellips_data->dynamics.axis_ids[p]][1];
                main_axis = vec3(0.0f, ellips_data->axes[ellips_data->dynamics.axis_ids[p]][1], 0.0f);
            }
            if (ellips_data->axes[ellips_data->dynamics.axis_ids[p]][2] > axis_max){
                axis_max = ellips_data->axes[ellips_data->dynamics.axis_ids[p]][2];
                main_axis = vec3(0.0f, 0.0f, ellips_data->axes[ellips_data->dynamics.axis_ids[p]][2]);
            }

            std::vector<vec3> axis;
            axis.resize(ellips_data->dynamics.trajs[p]->positions.size());
            // same axis along trajectory
            std::fill(axis.begin(), axis.end(), main_axis);
            axes.insert(axes.end(),
                        axis.begin(),
                        axis.end());

        }

        traj_renderer_3D_ribbon_gpu.set_buffers(ctx, positions, colors, axes, orientations, normals, *traj_indices);

        traj_renderer_3D_ribbon_gpu.initial = false;
        std::cout << " finished" << std::endl;
    } else if (out_of_date) {
        traj_renderer_3D_ribbon_gpu.update_element_buffer(*traj_indices);
    }

    traj_renderer_3D_ribbon_gpu.height = ribbon_height;

    // all data already transfered to GPU
    // draw with current view
    traj_renderer_3D_ribbon_gpu.draw(ctx);
}

void plugin::render_trajectory_tubes(cgv::render::context& ctx)
{
    for (size_t e = 0; e < traj_renderer_tubes.size(); e++) {
        if (traj_renderer_tubes[e]->initial) {
            std::cout << "Set up trajectory tubes ... ";
            std::vector<vec4> vertices;
            std::vector<vec4> normals;
            std::vector<vec2> texture_coord;

            // compute vertices for one ellipsoid
            create_ellipsoid_vertices(vertices, normals, texture_coord, ellips_data->axes[e], 6, 6);
            traj_renderer_tubes[e]->set_buffers(ctx, vertices, normals, *tubes_positions[e], *tubes_orientations[e], *tubes_colors[e]);

            traj_renderer_tubes[e]->initial = false; 
            std::cout << " finished" << std::endl;
        } else if (out_of_date) {
            traj_renderer_tubes[e]->update_translation_buffer(*tubes_positions[e]);
            traj_renderer_tubes[e]->update_orientation_buffer(*tubes_orientations[e]);
            traj_renderer_tubes[e]->update_color_buffer(*tubes_colors[e]);
        }

        traj_renderer_tubes[e]->draw(ctx);
    }
}

void plugin::render_normals(cgv::render::context& ctx)
{
    // set all vertex data once
    if (velocity_renderer_line.initial) {
        std::cout << "Set up normals ... ";

        velocity_renderer_line.set_buffers(ctx, *glyph_positions, *normals_vis, glyph_value_color);

        velocity_renderer_line.initial = false;
        std::cout << "finished" << std::endl;
    } else if (out_of_date) {
        velocity_renderer_line.update_position_buffer(*glyph_positions);
        velocity_renderer_line.update_velocity_buffer(*normals_vis);
    }

    velocity_renderer_line.draw(ctx);
}

void plugin::render_velocities(cgv::render::context& ctx)
{
    // set all vertex data once
    if (velocity_renderer_line.initial) {
        std::cout << "Set up velocities ... ";

        velocity_renderer_line.set_buffers(ctx, *glyph_positions, *velocities, glyph_value_color);

        velocity_renderer_line.initial = false;
        std::cout << "finished" << std::endl;
    } else if (out_of_date) {
        velocity_renderer_line.update_position_buffer(*glyph_positions);
        velocity_renderer_line.update_velocity_buffer(*velocities);
    }

    velocity_renderer_line.draw(ctx);
}

void plugin::render_angular_velocities(cgv::render::context& ctx)
{
    // set all vertex data once
    if (angular_velocity_renderer_line.initial) {
        std::cout << "Set up angular velocities ... ";

        angular_velocity_renderer_line.set_buffers(ctx, *glyph_positions, *angular_velocities, glyph_value_color);

        angular_velocity_renderer_line.initial = false;
        std::cout << "finished" << std::endl;
    } else if (out_of_date) {
        angular_velocity_renderer_line.update_position_buffer(*glyph_positions);
        angular_velocity_renderer_line.update_velocity_buffer(*angular_velocities);
    }

    angular_velocity_renderer_line.draw(ctx);
}

void plugin::render_ellipsoids(cgv::render::context& ctx)
{
    for (size_t e = 0; e < traj_renderer_ellipsoids.size(); e++) {
        if (traj_renderer_ellipsoids[e]->initial) {
            std::cout << "Set up trajectory ellipsoids ... ";
            std::vector<vec4> vertices;
            std::vector<vec4> normals;

            // compute texture
            Texture2D tex;
            tex.width = 128;
            tex.height = 128;
            tex.texture.resize(tex.width * tex.height);

            size_t index = 0;
            for (int y = 1; y <= tex.height; y++) {
                for (int x = 1; x <= tex.width; x++) {
                        // paint one half red and white and the other blue and white
                        if (x <= (0.5 * tex.width) && y > 0.5 * tex.height){
                            tex.texture[index] = vec3(1.0f, 0.85f, 0.85f);
                        } else if (x >= tex.width - (0.5 * tex.width) && y < 0.5 * tex.height){
                            tex.texture[index] = vec3(0.85f, 0.85f, 1.0f);
                        } else {
                            tex.texture[index] = vec3(1.0f, 1.0f, 1.0f);
                        }
                    
                    index++;
                }
            }

            // compute vertices for one ellipsoid
            create_ellipsoid_vertices(vertices, normals, tex.coord, ellips_data->axes[e]);

            traj_renderer_ellipsoids[e]->set_buffers(ctx, vertices, normals, tex, *ellipsoid_positions[e], *ellipsoid_orientations[e]);

            traj_renderer_ellipsoids[e]->initial = false; 
            std::cout << "finished" << std::endl;
        } else if (out_of_date) {
            traj_renderer_ellipsoids[e]->update_translation_buffer(*ellipsoid_positions[e]);
            traj_renderer_ellipsoids[e]->update_orientation_buffer(*ellipsoid_orientations[e]);
        }

        traj_renderer_ellipsoids[e]->textured = textured_ellipsoids;

        traj_renderer_ellipsoids[e]->draw(ctx);
    }
}

void plugin::render_stationary_particles(cgv::render::context& ctx)
{
    if (sphere_renderer.initial) {
        std::cout << "Set up stationary particles ... ";
        std::vector<vec4> vertices;
        std::vector<vec4> normals;

        // compute texture
        Texture2D tex;
        tex.width = 128;
        tex.height = 128;
        tex.texture.resize(tex.width * tex.height);
        std::fill(tex.texture.begin(), tex.texture.end(), vec3(0.75f, 0.75f, 0.85f));

        // compute vertices for one sphere
        create_ellipsoid_vertices(vertices, normals, tex.coord, ellips_data->axes[ellips_data->stationaries.axis_ids[0]], 10, 10);

        // set vertices of one sphere at origin
        sphere_renderer.set_buffers(ctx, vertices, normals, tex, ellips_data->stationaries.positions, ellips_data->stationaries.orientations);

        sphere_renderer.initial = false; 
        std::cout << "finished" << std::endl;
    }

    sphere_renderer.draw(ctx);
}

void plugin::render_bounding_box(cgv::render::context& ctx)
{
    if (b_box_renderer.initial) {
        std::cout << "Set up bounding box ... ";
        vec3 min = ellips_data->b_box.min;
        vec3 max = ellips_data->b_box.max;

        std::vector<vec3> vertices;
        std::vector<unsigned int> indices;

        create_box_vertices(vertices, indices, min, max, restart_id);

        std::vector<vec4> colors;
        colors.resize(vertices.size());
        std::fill(colors.begin(), colors.end(), vec4(0.75f, 0.75f, 0.85f, 1.0f));


        // set all buffers needed for rendering lines
        b_box_renderer.set_buffers(ctx, vertices, colors, indices);

        b_box_renderer.initial = false;
        std::cout << "finished" << std::endl;
    }

    // all data already transfered to GPU
    // draw with current view
    b_box_renderer.draw(ctx);
}

void plugin::render_roi_box(cgv::render::context& ctx)
{
    if (roi_box_renderer.initial) {
        std::cout << "Set up bounding box ... ";
        vec3 min = roi.min;
        vec3 max = roi.max;

        std::vector<vec3> vertices;
        std::vector<unsigned int> indices;

        create_box_vertices(vertices, indices, min, max, restart_id);

        std::vector<vec4> colors;
        colors.resize(vertices.size());
        std::fill(colors.begin(), colors.end(), vec4(1.0f, 0.60f, 0.40f, 1.0f));

        // set all buffers needed for rendering lines
        roi_box_renderer.set_buffers(ctx, vertices, colors, indices);

        roi_box_renderer.initial = false;
        std::cout << "finished" << std::endl;
    } else if (out_of_date) {
        std::vector<vec3> vertices;
        std::vector<unsigned int> indices;
        vec3 min = roi.min;
        vec3 max = roi.max;

        create_box_vertices(vertices, indices, min, max, restart_id);

        roi_box_renderer.update_element_buffer(indices);
        roi_box_renderer.update_position_buffer(vertices);
    }
  
    roi_box_renderer.draw(ctx);
}

void plugin::set_traj_indices_out_of_date()
{
    if (start_time > end_time) {
        std::cerr << "Wrong value: start time < end time expected" << std::endl;
        return;
    }

    out_of_date = true;

    post_redraw();
}

bool plugin::filter_length(std::shared_ptr<trajectory_data> traj)
{
    float total_length_x = ellips_data->b_box.max[0] - ellips_data->b_box.min[0];
    float very_small_x = total_length_x * length_filter_data.thresh_very_small / 100;
    float small_x = total_length_x * length_filter_data.thresh_small / 100;
    float medium_x = total_length_x * length_filter_data.thresh_medium / 100;

    float length_x =  abs(traj->b_box.max[0] - traj->b_box.min[0]);


    // very small particle that should not be displayed
    if (length_x < very_small_x && !length_filter_data.x_very_small_traj){
        return true;
    }
    // small particle that should not be displayed
    else if (length_x > very_small_x && length_x < small_x && !length_filter_data.x_small_traj){
        return true;
    }
    // medium particle that should not be displayed
    else if (length_x > small_x && length_x < medium_x && !length_filter_data.x_medium_traj) {
        return true;
    }
    // large particle that should not be displayed
    else if (length_x > medium_x  && !length_filter_data.x_large_traj) {
        return true;
    }

    float total_length_y = ellips_data->b_box.max[1] - ellips_data->b_box.min[1];
    float very_small_y = total_length_y * length_filter_data.thresh_very_small / 100;
    float small_y = total_length_y * length_filter_data.thresh_small / 100;
    float medium_y = total_length_y * length_filter_data.thresh_medium / 100;

    float length_y =  abs(traj->b_box.max[1] - traj->b_box.min[1]);


    // very small particle that should not be displayed
    if (length_y < very_small_y && !length_filter_data.y_very_small_traj){
        return true;
    }
    // small particle that should not be displayed
    else if (length_y > very_small_y && length_y < small_y && !length_filter_data.y_small_traj){
        return true;
    }
    // medium particle that should not be displayed
    else if (length_y > small_y && length_y < medium_y && !length_filter_data.y_medium_traj) {
        return true;
    }
    // large particle that should not be displayed
    else if (length_y > medium_y  && !length_filter_data.y_large_traj) {
        return true;
    }

    float total_length_z = ellips_data->b_box.max[2] - ellips_data->b_box.min[2];
    float very_small_z = total_length_z * length_filter_data.thresh_very_small / 100;
    float small_z = total_length_z * length_filter_data.thresh_small / 100;
    float medium_z = total_length_z * length_filter_data.thresh_medium / 100;

    float length_z =  abs(traj->b_box.max[2] - traj->b_box.min[2]);


    // very small particle that should not be displayed
    if (length_z < very_small_z && !length_filter_data.z_very_small_traj){
        return true;
    }
    // small particle that should not be displayed
    else if (length_z > very_small_z && length_z < small_z && !length_filter_data.z_small_traj){
        return true;
    }
    // medium particle that should not be displayed
    else if (length_z > small_z && length_z < medium_z && !length_filter_data.z_medium_traj) {
        return true;
    }
    // large particle that should not be displayed
    else if (length_z > medium_z  && !length_filter_data.z_large_traj) {
        return true;
    }

    return false;
}

bool plugin::in_region_of_interest(std::shared_ptr<trajectory_data> traj)
{
    if (traj->b_box.min[0] <= roi.max[0] && traj->b_box.max[0] >= roi.min[0] &&
            traj->b_box.min[1] <= roi.max[1] && traj->b_box.max[1] >= roi.min[1] &&
            traj->b_box.min[2] <= roi.max[2] && traj->b_box.max[2] >= roi.min[2]){
        return true;
    }

    return false;
}

bool plugin::in_region_of_interest_exact(std::shared_ptr<trajectory_data> traj)
{
    bool result = false;

    for (int t = 0; t < (int)traj->positions.size(); t++) {
        if (traj->positions[t][0] >= roi.min[0] && traj->positions[t][0] <= roi.max[0] &&
                traj->positions[t][1] >= roi.min[1] && traj->positions[t][1] <= roi.max[1] &&
                traj->positions[t][2] >= roi.min[2] && traj->positions[t][2] <= roi.max[2]) {

            if (roi_with_time_interval) {
                // if automatically searched regions of interests are viewed use time interval
                // of their search for determining if they are displayed
                if (show_pois && poi_searched) {
                    if (t <= poi_end_time && t >= poi_start_time){
                        result = true;
                    }
                } else {
                    if (t <= end_time && t >= start_time){
                        result = true;
                    }
                }
            } else {
                result = true;
            }
        }
    }

    return result;
}

bool plugin::skip_traj(size_t p)
{
    bool check_length = !(length_filter_data.x_very_small_traj && length_filter_data.x_small_traj && length_filter_data.x_medium_traj && length_filter_data.x_large_traj
                        && length_filter_data.y_very_small_traj && length_filter_data.y_small_traj && length_filter_data.y_medium_traj && length_filter_data.y_large_traj
                        && length_filter_data.z_very_small_traj && length_filter_data.z_small_traj && length_filter_data.z_medium_traj && length_filter_data.z_large_traj)
                        && filter_length_active;

    if (check_length) {
        // skips trajectory if it doesn't fit the selected lengths
        if (filter_length(ellips_data->dynamics.trajs[p]))
            return true;
    }

    if (roi_active) {
        // compute new roi
        vec3 diff = ellips_data->b_box.max - ellips_data->b_box.min;
        vec3 length = vec3(roi_data.length_x_percent / 100.0f * diff[0],
                           roi_data.length_y_percent / 100.0f * diff[1],
                           roi_data.length_z_percent / 100.0f * diff[2]);
        roi.min = vec3(roi_data.pos_x_percent / 100.0f * diff[0],
                       roi_data.pos_y_percent / 100.0f * diff[1],
                       roi_data.pos_z_percent / 100.0f * diff[2])
                     + ellips_data->b_box.min;
        roi.max = roi.min + length;
        roi.center = roi.min + (length / 2);

        // first check if bounding box of trajectory intersects with roi, skip if not
        if (!in_region_of_interest(ellips_data->dynamics.trajs[p]))
                return true;

        // consideration of time interval is only possible for exact computation
        // therefore turn it on and update gui
        if (roi_with_time_interval && !roi_exact) {
            roi_exact = true;
            update_all_members();
        }

        // it is possible that the trajectory not really intersected with current roi
        if (roi_exact) {
            if (!in_region_of_interest_exact(ellips_data->dynamics.trajs[p]))
                return true;
        }
    }

    return false;
}

void plugin::setup_traj_indices()
{
    size_t vis_traj = ellips_data->dynamics.trajs.size();

    if (mode == TRAJ_LINE && !hide_trajs) {
        traj_indices_strip = new std::vector<unsigned int>();
        traj_indices_strip->reserve(vis_traj * (end_time - start_time));
    }

    if (mode == TRAJ_3D_RIBBON && !hide_trajs) {
        traj_3D_ribbon_indices = new std::vector<unsigned int>();
        traj_3D_ribbon_indices->reserve(vis_traj * (end_time - start_time) * 2);
    }

    if (mode == TRAJ_3D_RIBBON_GPU && !hide_trajs) {
        traj_indices = new std::vector<unsigned int>();
        traj_indices->reserve(vis_traj * (end_time - start_time) * 2);
    }

    if (mode == TRAJ_RIBBON && !hide_trajs) {
        traj_ribbon_indices = new std::vector<unsigned int>();
        traj_ribbon_indices->reserve(vis_traj * (end_time - start_time) * 2);
    }

    if (mode == TRAJ_TUBE && !hide_trajs) {
        tubes_positions.resize(ellips_data->axes.size());
        tubes_orientations.resize(ellips_data->axes.size());
        tubes_colors.resize(ellips_data->axes.size());

        for (size_t e = 0; e < ellips_data->axes.size(); e++) {
            tubes_positions[e] = new std::vector<vec3>();
            tubes_positions[e]->reserve(vis_traj);
            tubes_orientations[e] = new std::vector<vec4>();
            tubes_orientations[e]->reserve(vis_traj);
            tubes_colors[e] = new std::vector<vec4>();
            tubes_colors[e]->reserve(vis_traj);
        }
    }

    if (display_glyphs) {
        glyph_positions = new std::vector<vec3>();
        normals_vis = new std::vector<vec3>();
        velocities = new std::vector<vec3>();
        angular_velocities = new std::vector<vec3>();
        glyph_positions->reserve(vis_traj * (end_time - start_time));
        velocities->reserve(vis_traj * (end_time - start_time));
        angular_velocities->reserve(vis_traj * (end_time - start_time));
        normals_vis->reserve(vis_traj * (end_time - start_time));
    }

    if (display_ellipsoids) {
        ellipsoid_orientations.resize(ellips_data->axes.size());
        ellipsoid_positions.resize(ellips_data->axes.size());

        for (size_t e = 0; e < ellips_data->axes.size(); e++) {
            ellipsoid_positions[e] = new std::vector<vec3>();
            ellipsoid_positions[e]->reserve(vis_traj);
            ellipsoid_orientations[e] = new std::vector<vec4>();
            ellipsoid_orientations[e]->reserve(vis_traj);
        }
    }
}

void plugin::clear_traj_indices()
{
    if (mode == TRAJ_LINE && !hide_trajs)
        delete traj_indices_strip;

    if (mode == TRAJ_3D_RIBBON_GPU && !hide_trajs)
        delete traj_indices;

    if (mode == TRAJ_3D_RIBBON && !hide_trajs)
        delete traj_3D_ribbon_indices;

    if (mode == TRAJ_RIBBON && !hide_trajs)
        delete traj_ribbon_indices;

    if (mode == TRAJ_TUBE && !hide_trajs) {
        for (size_t e = 0; e < ellipsoid_positions.size(); e++) {
            delete tubes_positions[e];
            delete tubes_orientations[e];
            delete tubes_colors[e];
        }
    }
    
    if (display_ellipsoids) {
        for (size_t e = 0; e < ellipsoid_positions.size(); e++) {
            delete ellipsoid_positions[e];
            delete ellipsoid_orientations[e];
        }
    }

    if (display_glyphs) {
        delete glyph_positions;
        delete velocities;
        delete normals_vis;
        delete angular_velocities;
    }
}

void plugin::compute_traj_indices()
{
    // get number of trajectories that should be displayed
    size_t vis_traj = ellips_data->dynamics.trajs.size();
    size_t start_id = 0;

    setup_traj_indices();

    // display only one trajectory
    if (display_single_traj) {
        start_id = single_traj_id;
        vis_traj = start_id + 1;
    }

    // count number of visualized trajectory
    nr_visible_traj = 0;

    for (size_t p = start_id; p < vis_traj; p++) {
        if (skip_traj(p)) {
            continue;
        }

        // compute offsets for inserting vector elements
        //   - std::vector::end is the element past the last element of the vector
        //   - std::vector::insert inserts element between first and last position
        //     (including first but not including last)
        //   - start and end time is given in range of [1, timesteps]
        int start_offset = start_time - 1;
        int end_offset = end_time;

        if (mode == TRAJ_LINE && !hide_trajs) {
            traj_indices_strip->insert(traj_indices_strip->end(),
                    ellips_data->dynamics.trajs[p]->indices_strip.begin() + start_offset,
                    ellips_data->dynamics.trajs[p]->indices_strip.begin() + end_offset);
            // determine end of primitive
            traj_indices_strip->push_back(restart_id);
        }

        if (mode == TRAJ_3D_RIBBON_GPU && !hide_trajs) {
            traj_indices->insert(traj_indices->end(),
                    ellips_data->dynamics.trajs[p]->indices.begin() + (start_offset * 2),
                    ellips_data->dynamics.trajs[p]->indices.begin() + (end_offset * 2) - 1);
            // determine end of primitive
            traj_indices->push_back(restart_id);
        }

        if (mode == TRAJ_3D_RIBBON && !hide_trajs) {
            if (traj_renderer_3D_ribbon.indices_top.size() > 0){
                traj_3D_ribbon_indices->insert(traj_3D_ribbon_indices->end(),
                        traj_renderer_3D_ribbon.indices_top[p].begin() + (start_offset * 2),
                        traj_renderer_3D_ribbon.indices_top[p].begin() + (end_offset * 2));
                traj_3D_ribbon_indices->push_back(restart_id);

                traj_3D_ribbon_indices->insert(traj_3D_ribbon_indices->end(),
                        traj_renderer_3D_ribbon.indices_side1[p].begin() + (start_offset * 2),
                        traj_renderer_3D_ribbon.indices_side1[p].begin() + (end_offset * 2));
                traj_3D_ribbon_indices->push_back(restart_id);

                traj_3D_ribbon_indices->insert(traj_3D_ribbon_indices->end(),
                        traj_renderer_3D_ribbon.indices_bottom[p].begin() + (start_offset * 2),
                        traj_renderer_3D_ribbon.indices_bottom[p].begin() + (end_offset * 2));
                traj_3D_ribbon_indices->push_back(restart_id);

                traj_3D_ribbon_indices->insert(traj_3D_ribbon_indices->end(),
                        traj_renderer_3D_ribbon.indices_side2[p].begin() + (start_offset * 2),
                        traj_renderer_3D_ribbon.indices_side2[p].begin() + (end_offset * 2));
                traj_3D_ribbon_indices->push_back(restart_id);
            }
        }

        if (mode == TRAJ_RIBBON && !hide_trajs) {
            if (traj_renderer_ribbon.indices.size() > 0){
                traj_ribbon_indices->insert(traj_ribbon_indices->end(),
                        traj_renderer_ribbon.indices[p].begin() + (start_offset * 2),
                        traj_renderer_ribbon.indices[p].begin() + (end_offset * 2));
            }
            traj_ribbon_indices->push_back(restart_id);
        }

        if (mode == TRAJ_TUBE && !hide_trajs) {    
            // get ellipsoid id of current traj
            size_t id = ellips_data->dynamics.axis_ids[p];

            // TODO: this is very slow (because of vec3 and vec4 elements?)
            // update position and orientation vectors for tubes
            tubes_positions[id]->insert(tubes_positions[id]->end(),
                    ellips_data->dynamics.trajs[p]->positions.begin() + start_offset,
                    ellips_data->dynamics.trajs[p]->positions.begin() + end_offset);
            tubes_orientations[id]->insert(tubes_orientations[id]->end(),
                    ellips_data->dynamics.trajs[p]->orientations.begin() + start_offset,
                    ellips_data->dynamics.trajs[p]->orientations.begin() + end_offset);
            tubes_colors[id]->insert(tubes_colors[id]->end(),
                    time_colors.begin() + start_offset,
                    time_colors.begin() + end_offset);
        }

        if (display_ellipsoids) {
            // get ellipsoid id of current traj
            size_t id = ellips_data->dynamics.axis_ids[p];

            if (ellipsoid_tick_sample < (int)time_steps) {
                for (int t = start_time; t < end_time; t++) {
                    // do not display ellipsoid at every timestep
                    if (!(t % ellipsoid_tick_sample)) {
                        // update position and orientation vectors with last ellipsoid position
                        ellipsoid_positions[id]->push_back(ellips_data->dynamics.trajs[p]->positions[t]);
                        ellipsoid_orientations[id]->push_back(ellips_data->dynamics.trajs[p]->orientations[t]);
                    }
                }
            }

            // always display ellipsoid at end of traj
            int index = end_time - 1;
            // update position and orientation vectors with last ellipsoid position
            ellipsoid_positions[id]->push_back(ellips_data->dynamics.trajs[p]->positions[index]);
            ellipsoid_orientations[id]->push_back(ellips_data->dynamics.trajs[p]->orientations[index]);
        }

        if (display_glyphs) {
            for (int t = start_time; t < end_time; t++) {
                // do not display glyph at every timestep
                if (!(t % glyph_sample)) {
                    glyph_positions->push_back(ellips_data->dynamics.trajs[p]->positions[t]);
                    velocities->push_back(glyph_scale_rate * ellips_data->dynamics.trajs[p]->velocities[t]);
                    normals_vis->push_back(glyph_scale_rate * ellips_data->dynamics.trajs[p]->main_axis_normals[t]);
                    angular_velocities->push_back(glyph_scale_rate * ellips_data->dynamics.trajs[p]->angular_velocities[t]);
                }
            }
        }

        nr_visible_traj++;
    }   
}


class CompareROIData
{
public:
    bool operator() (std::pair<int, ROIData> a, std::pair<int, ROIData> b)
    {
        return a.first < b.first;
    }
};


void plugin::search_points_of_interest()
{
    std::cout << "start searching for interesting points ... ";

    poi_points.resize(0);

    // save current state of length and time filter
    poi_start_time = start_time;
    poi_end_time = end_time;
    poi_length_filter_data = length_filter_data;

    // activate length filter
    filter_length_active = true;

    // activate roi
    roi_active = true;
    roi_exact = true;
    roi_with_time_interval = true;

    // store results ordered by number of found trajectories
    std::priority_queue<std::pair<int, ROIData>, std::vector<std::pair<int, ROIData>>, CompareROIData> pq;

    // move roi through whole dataset
    for (int x = 0; x < 100; x = x + (roi_data.length_x_percent / 2)) {
        for (int y = 0; y < 100; y = y + (roi_data.length_x_percent / 2)) {
            for (int z = 0; z < 100; z = z + (roi_data.length_x_percent / 2)) {
                roi_data.pos_x_percent = x;
                roi_data.pos_y_percent = y;
                roi_data.pos_z_percent = z;

                compute_traj_indices();

                if (nr_visible_traj > 0)
                    pq.push(std::make_pair(nr_visible_traj, roi_data));

                clear_traj_indices();
            }
        }
    }

    while (poi_points.size() < max_pois && pq.size() > 0) {
        std::pair<int, ROIData> data = pq.top();
        poi_points.push_back(data.second);
        pq.pop();
        
    }

    std::cout << "finished with " << poi_points.size() << " points" << std::endl;
    poi_searched = true;

    show_pois = true;

    show_points_of_interest();
}

void plugin::show_points_of_interest()
{
    if (poi_searched) {
        // activate roi
        roi_active = true;
        roi_exact = true;
        roi_with_time_interval = true;

        // activate length filter
        filter_length_active = true;
        length_filter_data = poi_length_filter_data;

        if (poi_id <= (int)(poi_points.size() - 1)) {
            roi_data = poi_points[poi_id];

            set_traj_indices_out_of_date();
        }
    } else {
        show_pois = false;
    }
    update_all_members();
}

void plugin::update_material()
{
    traj_renderer_3D_ribbon.update_material(ribbon_material);
    traj_renderer_3D_ribbon_gpu.update_material(ribbon_material);
    sphere_renderer.update_material(stationary_material);

    for (size_t e = 0; e < traj_renderer_ellipsoids.size(); e++) {
        traj_renderer_ellipsoids[e]->update_material(ellipsoid_material);   
    }
    post_redraw();
}

void plugin::set_velocity_color()
{
    velocity_renderer_line.set_color(glyph_value_color);
    angular_velocity_renderer_line.set_color(glyph_value_color);
    post_redraw();
}

void plugin::reset_perf_stats()
{
    perf_stats = true;
    indices_time_ticks = 0;
    indices_time = std::chrono::duration<double, std::ratio<1,1000>>(0.0);
    indices_time_sum = 0.;
    max_indices_time = 0.;
    min_indices_time = std::numeric_limits<double>::max();
    avg_indices_time = 0.;

    gpu_time_ticks = 0;
    gpu_time_sum = 0.;
    max_gpu_time = 0.;
    min_gpu_time = std::numeric_limits<double>::max();
    avg_gpu_time = 0.;
    gen_prims = 0;
}

void plugin::swapQueryBuffers()
{
    if (queryBackBuffer) {
        queryBackBuffer = 0;
        queryFrontBuffer = 1;
    }
    else {
        queryBackBuffer = 1;
        queryFrontBuffer = 0;
    }
}

void plugin::changed_setting() {
    if (cut_trajs)
        same_start = false;

    if (!animate){
        current_time = -1;
        paused = false;
    }

    if (!set_light_to_eye_pos) {
        // light position
        scene_light->set_light_position(vec3(ellips_data->b_box.center[0],
                                             ellips_data->b_box.max[1] + ellips_data->b_box.max[0],
                                             ellips_data->b_box.center[2]));
    }

    update_all_members();
    post_redraw();
}

void plugin::on_set(void* member_ptr)
{
    // Redraw the scene every time a gui value was changed
    post_redraw();
}

void plugin::stream_help(std::ostream& os)
{
    // display help F1
    cgv::utils::oprintf(os, "%s short cuts: \n", get_name().c_str());
    cgv::utils::oprintf(os, "  fix views: 1 - front, 2 - top, 3 - left, 4 - center focus, 5 - back, 6 - bottom, 7 - right\n");
    cgv::utils::oprintf(os, "\n");
}

void plugin::stream_stats(std::ostream& os)
{
    // display information F8
    cgv::utils::oprintf(os, "%s \n", get_name().c_str());

    cgv::utils::oprintf(os, "  data set: %s \n", data_name.c_str());
    vec3 b_box_diff = ellips_data->b_box.max - ellips_data->b_box.min;
    cgv::utils::oprintf(os, "  bounding box: %.2f length - %.2f width - %.2f height\n", b_box_diff[0], b_box_diff[2], b_box_diff[1]);
    cgv::utils::oprintf(os, "  time interval: %s to %s of %s time steps\n", start_time, end_time, time_steps);

    cgv::utils::oprintf(os, "  physical time intervall: %.2f to %.2f with %.2f per time step\n", ellips_data->dynamics.times[start_time - 1], ellips_data->dynamics.times[end_time - 1], time_per_step);

    cgv::utils::oprintf(os, "  number of trajectories: %s visible - %s total \n", nr_visible_traj, nr_particles);

    if (perf_stats) {
        double _min_gpu_time = (min_gpu_time == std::numeric_limits<double>::max()) ? 0 : min_gpu_time;
        double _min_indices_time = (min_indices_time == std::numeric_limits<double>::max()) ? 0 : min_indices_time;

        cgv::utils::oprintf(os, "\nperformance stats:\n");
        cgv::utils::oprintf(os, "  gpu time: min %.2fms / cur %.2fms / max %.2fms / avg %.2fms (on %s frames) / primitives %s \n", _min_gpu_time, gpu_time, max_gpu_time, avg_gpu_time, gpu_time_ticks, gen_prims);

        cgv::utils::oprintf(os, "  update indices: %.2fms min / %.2fms current / %.2fms max / %.2fms avg (on %s updates) \n", _min_indices_time, indices_time.count(), max_indices_time, avg_indices_time, indices_time_ticks);       
    }
}

std::string plugin::get_menu_path() const
{
    return "trajectory/ellipsoid";
}

// register
static object_registration_1<plugin, const char*> ell_trajectory_reg("trajectory", "");
}
