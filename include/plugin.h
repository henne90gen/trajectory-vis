#pragma once

#include <chrono>

#include <cgv/base/node.h>
#include <cgv/render/drawable.h>
#include <cgv/gui/provider.h>
#include <cgv/render/context.h>
#include <cgv/gui/event_handler.h>
#include <cgv/render/view.h>

#include "traj_line_renderer.h"
#include "traj_ribbon_renderer.h"
#include "traj_tube_renderer.h"
#include "traj_ribbon_3d_renderer.h"
#include "traj_ribbon_3d_renderer_gpu.h"
#include "ellipsoid_instanced_renderer.h"
#include "traj_velocity_renderer.h"
#include "data.h"
#include "lighting.h"


#define GL_GPU_MEM_INFO_TOTAL_AVAILABLE_MEM_NVX 0x9048
#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049

namespace ellipsoid_trajectory {

enum RenderMode { TRAJ_LINE, TRAJ_RIBBON, TRAJ_3D_RIBBON, TRAJ_3D_RIBBON_GPU, TRAJ_TUBE };
enum GlyphMode { LINEAR_VELOCITY, ANGULAR_VELOCITY, NORMALS };

struct ROIData {
    int length_x_percent;
    int length_y_percent;
    int length_z_percent;
    int pos_x_percent;
    int pos_y_percent;
    int pos_z_percent;
};

struct LengthFilterData {
    bool filter_length_active;
    bool x_very_small_traj;
    bool x_small_traj;
    bool x_medium_traj;
    bool x_large_traj;
    bool y_very_small_traj;
    bool y_small_traj;
    bool y_medium_traj;
    bool y_large_traj;
    bool z_very_small_traj;
    bool z_small_traj;
    bool z_medium_traj;
    bool z_large_traj;
    int thresh_small;
    int thresh_very_small;
    int thresh_medium;
};

class plugin : 
    public cgv::base::group,         // obligatory base class to integrate into global tree structure and to store a name
    public cgv::render::drawable,    // enables 3d view capabilities for this class
    public cgv::gui::event_handler,  // handles key events
    public cgv::gui::provider        // enables 2d gui capabilities for this class
{
public:
    // construct from name which is necessary construction argument to node
    plugin(const char* name);
    ~plugin();

    // F8 text
    void stream_stats(std::ostream& os);
    // F1 text
    void stream_help(std::ostream& os);

    // return a path in the main menu to select the gui
    std::string get_menu_path() const;
    // set up view and renderer
    bool init(cgv::render::context& ctx);
    void init_frame(cgv::render::context& ctx);
    // creates gui elements as buttons and sliders
    void create_gui();
    // called whenever a gui element changes a bound value
    void on_set(void*);
    // takes care of consistency of values of ui elements
    void changed_setting();

    // handles key events used to set different views
    bool handle(cgv::gui::event& e);

    void clear(cgv::render::context& ctx);
    // calls different rendering methods
    void draw(cgv::render::context& c);

private:
    // ------------------------- data set -----------------------------------------------
    data* ellips_data;
    std::string data_name;
    bool stationaries_available;
    size_t nr_particles;
    size_t time_steps;
    float time_per_step;

    // ------------------------- load data ----------------------------------------------
    bool same_start;
    bool cut_trajs;
    bool create_equidistant;
    float split_tolerance;
    int scanned_time_steps;
    int start_load_time_step;       // range [1-N]
    int end_load_time_step;         // range [1-N]
    int time_step_resolution;
    std::string directory_name;

    // store all scanned files oredered by their id
    std::vector<std::pair<double, std::string>> files;

    // scans given directory for files with name "ell_tr_<timestep>.bin"
    void scan_data();
    // load files stored in files vector or call random generator
    void load_data(bool generated = false);
    // sets view, light direction, time colors ... depending on current ellips_data
    void set_up_data();


    // --------------------------- generator settings -----------------------------------
    int generator_number_trajectories;
    int generator_number_time_steps;
    float generator_start_velocity;
    int generator_seed;


    // ----------------------- trajectory rendering -------------------------------------
    // trajectory renderer
    RenderMode mode;
    bool hide_trajs;
    int tick_marks_sample;
    float ribbon_height;
    traj_line_renderer traj_renderer_line;
    traj_ribbon_renderer traj_renderer_ribbon;
    traj_ribbon_3d_renderer traj_renderer_3D_ribbon;
    traj_ribbon_3d_renderer_gpu traj_renderer_3D_ribbon_gpu;
    std::vector<traj_tube_renderer*> traj_renderer_tubes;

    void render_trajectory_lines(cgv::render::context& ctx);
    void render_trajectory_ribbons(cgv::render::context& ctx);
    void render_trajectory_3D_ribbons(cgv::render::context& ctx);
    void render_trajectory_3D_ribbons_gpu(cgv::render::context& ctx);
    void render_trajectory_tubes(cgv::render::context& ctx);

    // ellipsoid at end of trajectory
    bool display_ellipsoids;
    bool textured_ellipsoids;
    bool setup_ellipsoids;
    int ellipsoid_tick_sample;
    // ellipsoid_instanced_renderer traj_renderer_ellipsoid;
    std::vector<ellipsoid_instanced_renderer*> traj_renderer_ellipsoids;

    void render_ellipsoids(cgv::render::context& ctx);
    
    // glyph renderer for velocities and normals
    bool display_glyphs;
    GlyphMode glyph_mode;
    bool glyph_value_color;
    float glyph_scale_rate;
    int glyph_sample;
    traj_velocity_renderer normal_renderer_line;
    traj_velocity_renderer velocity_renderer_line;
    traj_velocity_renderer angular_velocity_renderer_line;

    // set currently selected velocity color (either value or direction encoding) 
    void set_velocity_color();
    void render_normals(cgv::render::context& ctx);
    void render_velocities(cgv::render::context& ctx);
    void render_angular_velocities(cgv::render::context& ctx);


    // ------------------------ environment rendering -----------------------------------
    bool hide_b_box;
    bool hide_coord;
    bool hide_stationaries;
    bool unchanged_view;
    

    // environment renderer (bounding box, stationary particles ...)
    traj_line_renderer b_box_renderer;
    traj_line_renderer roi_box_renderer;
    ellipsoid_instanced_renderer sphere_renderer;
    traj_line_renderer coord_renderer;
    
    void render_coordinate_system(cgv::render::context& ctx);
    void render_bounding_box(cgv::render::context& ctx);
    void render_roi_box(cgv::render::context& ctx);
    void render_stationary_particles(cgv::render::context& ctx);

    // material and light
    lighting* scene_light;
    bool set_light_to_eye_pos;
    Material ellipsoid_material;
    Material stationary_material;
    Material ribbon_material;
    // update material for all renderer
    void update_material();

    // get current view (position etc..)
    cgv::render::view* view_ptr;

    // ------------------------- handling of ebo indices --------------------------------
    bool out_of_date;
    unsigned int restart_id;

    // callback for changed ui elements which changes displayed trajectories
    void set_traj_indices_out_of_date();

    // current indices for element buffer of trajectory renderer for elements that
    // will be displayed
    std::vector<unsigned int>* traj_indices;            // indices for line: 1-2, 2-3, 3-4
    std::vector<unsigned int>* traj_indices_strip;      // indices for strip: 1-2-3-4
    std::vector<unsigned int>* traj_ribbon_indices;     // for precomputed vertices of ribbon
    std::vector<unsigned int>* traj_3D_ribbon_indices;  // for precomputed vertices of 3D ribbon
    std::vector<std::vector<vec3>*> ellipsoid_positions;
    std::vector<std::vector<vec4>*> ellipsoid_orientations;
    std::vector<std::vector<vec3>*> tubes_positions;
    std::vector<std::vector<vec4>*> tubes_orientations;
    std::vector<std::vector<vec4>*> tubes_colors;
    std::vector<vec3>* glyph_positions;
    std::vector<vec3>* velocities;
    std::vector<vec3>* normals_vis;
    std::vector<vec3>* angular_velocities;

    // computes indices for new EBO for rendering trajectories
    void compute_traj_indices();
    // creates new index vectors and reserves memory
    void setup_traj_indices();
    // deletes all index vectors to reduce memory
    void clear_traj_indices();

    // applies filter to current trajectory and returns true if filter doesn't match
    // thus the trajectory has to be skipped
    bool skip_traj(size_t p);


    // ---------------------- trajectory selection -------------------------------------
    int single_traj_id;
    bool display_single_traj;
    int nr_visible_traj;


    // --------------------------- time interval ----------------------------------------
    int start_time;
    int end_time;
    std::vector<vec4> time_colors;

    bool animate;
    bool paused;
    int current_time;
    int animate_start;
    int animate_end;
    // this method is called at a 60fps beat provided by cgv framework
    void timer_event(double, double dt);

    int fps;
    int animation_speed;
    std::chrono::steady_clock::time_point time_last_frame;


    // ----------------------- length filter --------------------------------------------
    LengthFilterData length_filter_data;
    bool filter_length_active;

    // checks if given trajectory fits current length filter
    bool filter_length(std::shared_ptr<trajectory_data> traj);


    // ----------------------- region of interest filter --------------------------------
    bool roi_active;
    bool roi_with_time_interval;
    bool roi_exact;
    ROIData roi_data;
    Bounding_Box roi;

    // checks if given trajectory crosses region of interest using bounding box of traj
    bool in_region_of_interest(std::shared_ptr<trajectory_data> traj);
    // checks if given trajectory crosses region of interest using its positions
    bool in_region_of_interest_exact(std::shared_ptr<trajectory_data> traj);


    // -------------------------- automatic search for interesting points ---------------
    bool poi_searched;
    size_t max_pois;
    bool show_pois;
    int poi_id;

    // save state of search
    int poi_start_time;
    int poi_end_time;
    LengthFilterData poi_length_filter_data;
    std::vector<ROIData> poi_points;

    // applies the current filter options while moving the ROI box through the bounding box
    void search_points_of_interest();
    // sets all ui variables to the corresponding result
    void show_points_of_interest();


    // --------------------------- performance stats ------------------------------------
    bool perf_stats;

    // store CPU time for indices computation
    std::chrono::duration<double, std::ratio<1,1000>> indices_time;
    double max_indices_time;
    double min_indices_time;
    double avg_indices_time;
    double indices_time_sum;
    int indices_time_ticks;

    // store GPU time measurements
    double gpu_time_sum;
    int gpu_time_ticks;
    double gpu_time;
    double max_gpu_time;
    double min_gpu_time;
    double avg_gpu_time;
    bool measured;

    // store information from GPU queries
    GLuint64 elapsed_time;
    GLuint64 gen_prims;
    // the array to store the two sets of queries.
    unsigned int queryID[2][2];
    unsigned int queryBackBuffer = 0;
    unsigned int queryFrontBuffer = 1;

    // restarts performance measure (necessary for averaging of the values)
    void reset_perf_stats();
    // swap GPU buffer to read result of last frame
    void swapQueryBuffers();
};

}
