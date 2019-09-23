
//////
//
// Dependencies
//

// C++ STL
#include <queue>
#include <unordered_map>

// ANN Library
#define ANN_USE_FLOAT
#include <ANN/ANN.h>

// CGV framework
#include <cgv/math/eig.h>
#include <cgv/math/point_operations.h>
#include <cgv/media/axis_aligned_box.h>
#include <cgv/media/mesh/dual_contouring.h>

// Local includes
#include "types.h"



//////
//
// (Sub-)Module namespace-ing [BEGIN]
//

// Top level
namespace ellipsoid_trajectory {



//////
//
// Types
//

typedef cgv::math::fvec<unsigned, 3> triangle;



//////
//
// Function declarations
//

template <class real>
inline static real r2d (real rads) { return real(180)*rads / real(M_PI); }

template <class real>
inline static real wendland (real d, real h)
{
	real norm = h*h*h*h*h, base = h - d;
	return d <= h ?
		(base*base*base*base * (h + real(4)*d)) / norm :
		0;
}

template <class T>
void estimate_normal_wls(unsigned nr_points, const T* _points, const T* _weights, T* _normal, T* _evals = 0, T* _mean = 0, T* _evecs = 0)
{
	cgv::math::mat<T> points;
	points.set_extern_data(3, nr_points, const_cast<T*>(_points));
	cgv::math::vec<T> weights;
	weights.set_extern_data(nr_points, const_cast<T*>(_weights));
	cgv::math::vec<T> normal;
	normal.set_extern_data(3, _normal);

	cgv::math::mat<T> covmat;
	cgv::math::vec<T> mean;

	cgv::math::weighted_covmat_and_mean(weights, points, covmat, mean);
	cgv::math::mat<double> dcovmat(covmat), v;
	cgv::math::diag_mat<double> d;
	cgv::math::eig_sym(dcovmat, v, d);

	normal = cgv::math::vec<T>(normalize(v.col(2)));

	if (_evals) {
		_evals[0] = (T)d(0);
		_evals[1] = (T)d(1);
		_evals[2] = (T)d(2);
	}
	if (_mean) {
		_mean[0] = (T)mean(0);
		_mean[1] = (T)mean(1);
		_mean[2] = (T)mean(2);
	}
	if (_evecs) {
		_evecs[0] = (T)v(0, 0);
		_evecs[1] = (T)v(1, 0);
		_evecs[2] = (T)v(2, 0);
		_evecs[3] = (T)v(0, 1);
		_evecs[4] = (T)v(1, 1);
		_evecs[5] = (T)v(2, 1);
		_evecs[6] = (T)v(0, 2);
		_evecs[7] = (T)v(1, 2);
		_evecs[8] = (T)v(2, 2);
	}
}

template <class real>
inline static cgv::math::fvec<real, 3> plane_project (
	const cgv::math::fvec<real, 3> &point,
	const cgv::math::fvec<real, 3> &plane_normal,
	const cgv::math::fvec<real, 3> &plane_refpoint
)
{
	vec3 vdir = point - plane_refpoint;
	float proj = cgv::math::dot(plane_normal, vdir);
	vec3 projvec = proj * plane_normal;
	return point - projvec;
}

template <class vec>
inline static vec vfloor(const vec &v)
{
	vec ret;
	for (int i=0; i<(int)v.size(); i++)
		ret[i] = std::floor(v[i]);
	return std::move(ret);
}
template <class vec>
inline static vec vceil(const vec &v)
{
	vec ret;
	for (int i=0; i<(int)v.size(); i++)
		ret[i] = std::ceil(v[i]);
	return std::move(ret);
}

template <class real, unsigned n>
inline static cgv::media::axis_aligned_box<real, n> discretize (const cgv::media::axis_aligned_box<real, n> &box, real res)
{
	return std::move(cgv::media::axis_aligned_box<real, n>(
		vfloor(box.get_min_pnt()/res) * res,
		vceil (box.get_max_pnt()/res) * res
	));
}
template <class real>
inline static cgv::media::axis_aligned_box<real, 3> discretize(const ellipsoid_trajectory::Bounding_Box &box, real res)
{
	return std::move(cgv::media::axis_aligned_box<real, 3>(
		vfloor(box.min/res) * res,
		vceil (box.max/res) * res
	));
}

template <class real>
inline static unsigned get_ellipsoid_max_axis_id(const cgv::math::fvec<real,3> &axes)
{
	return (
		axes[0] > axes[1] ?
			(axes[0] > axes[2] ? 0 : 2) :
			(axes[1] > axes[2] ? 1 : 2)
	);
}

template <class real>
inline static unsigned get_ellipsoid_min_axis_id(const cgv::math::fvec<real,3> &axes)
{
	return (
		axes[0] > axes[1] ?
			(axes[1] > axes[2] ? 2 : 1) :
			(axes[0] > axes[2] ? 2 : 0)
	);
}

template <class real>
inline static unsigned get_ellipsoid_mid_axis_id(const cgv::math::fvec<real,3> &axes)
{
	unsigned _min = get_ellipsoid_min_axis_id(axes), _max = get_ellipsoid_max_axis_id(axes);

	if ((_min == 2 && _max == 0) || (_min == 0 && _max == 2))
		return 1;
	else if ((_min == 1 && _max == 0) || (_min == 0 && _max == 1))
		return 2;
	else
		return 0;
}

template <class real>
inline static cgv::media::axis_aligned_box<real, 3> get_ellipsoid_bbox(const cgv::math::fvec<real,3> &axes)
{
	cgv::media::axis_aligned_box<real, 3> bbox;
	unsigned max_axis = get_ellipsoid_max_axis_id(axes);
	bbox.ref_max_pnt().set(axes[max_axis]*real(1.25), axes[max_axis]*real(1.25), axes[max_axis]*real(1.25));
	bbox.ref_min_pnt() = -bbox.get_max_pnt();
	return std::move(bbox);
}

template <class real>
inline static cgv::media::axis_aligned_box<real, 3> translate_bbox(
	const cgv::media::axis_aligned_box<real, 3> &bbox,
	const ellipsoid_trajectory::vec3 &v
)
{
	return std::move(cgv::media::axis_aligned_box<real, 3>(
		bbox.get_min_pnt() + v,
		bbox.get_max_pnt() + v
	));
}

template <class real>
bool boxes_overlap(
	const cgv::media::axis_aligned_box<real, 3> &box1, const cgv::media::axis_aligned_box<real, 3> &box2
)
{
	return
		box1.get_min_pnt().x() < box2.get_max_pnt().x() && box1.get_max_pnt().x() > box2.get_min_pnt().x() &&
		box1.get_min_pnt().y() < box2.get_max_pnt().y() && box1.get_max_pnt().y() > box2.get_min_pnt().y() &&
		box1.get_min_pnt().z() < box2.get_max_pnt().z() && box1.get_max_pnt().z() > box2.get_min_pnt().z();
}

template <class real, unsigned n>
bool point_in_box(
	const cgv::media::axis_aligned_box<real, n> &box, const cgv::math::fvec<real, n> &point
)
{
	bool gt_min = true, lt_max = true;
	for (unsigned i=0; i<n; i++)
	{
		gt_min = gt_min && point[i] >= box.get_min_pnt()[i];
		lt_max = lt_max && point[i] <= box.get_max_pnt()[i];
	}
	return gt_min && lt_max;
}

ellipsoid_trajectory::vec4 quat_to_aa (const ellipsoid_trajectory::vec4 &Q)
{
	register float tmp = std::sqrt(1.0f - Q.w()*Q.w());
	return std::move(ellipsoid_trajectory::vec4(
		Q.x() / tmp,
		Q.y() / tmp,
		Q.z() / tmp,
		std::acos(Q.w()) * 2.0f
	));
}



//////
//
// Class definitions
//

////
// Voxelization

template <class real>
struct voxel
{
	struct hash
	{
		unsigned long operator() (const voxel& key) const
		{
			// Uses the 3d vector hash function described in "Optimized Spatial
			// Hashing for Collision Detection of Deformable Objects" available here:
			// http://www.beosil.com/download/CollisionDetectionHashing_VMV03.pdf
			return unsigned long(
				key.x * 73856093LL ^ key.y * 19349663LL ^ key.z * 83492791LL
				/* mod N is ommitted since it would be (size_t_max)-1 here, which is
				   basically a no-op */
			);
		}
	};
	long long x, y, z;

	voxel() {}
	voxel(long long x, long long y, long long z) : x(x), y(y), z(z) {}
	voxel(const cgv::math::fvec<real,3> &position, real gridsize)
	{
		*this = get(position, gridsize);
	}

	inline bool operator == (const voxel &other) const
	{
		return x == other.x && y == other.y && z == other.z;
	}
	inline cgv::media::axis_aligned_box<real, 3> box(real gridsize)
	{
		return std::move(cgv::media::axis_aligned_box<real, 3>(
			vec3(
				real(x)*gridsize, real(y)*gridsize, real(z)*gridsize
			),
			vec3(
				real(x+1)*gridsize, real(y+1)*gridsize, real(z+2)*gridsize
			)
		));
	}
	inline vec3 midpoint(real gridsize)
	{
		return std::move(
			  (  vec3(real(x)*gridsize, real(y)*gridsize, real(z)*gridsize)
			   + vec3(real(x+1)*gridsize, real(y+1)*gridsize, real(z+2)*gridsize))
			* real(0.5)
		);
	}

	inline static voxel get (const cgv::math::fvec<real,3> &position, real gridsize)
	{
		bool nx = position.x() < 0, ny = position.y() < 0, nz = position.z() < 0;
		return std::move(voxel(
			(long)(position.x()/gridsize) - long(nx),
			(long)(position.y()/gridsize) - long(ny),
			(long)(position.z()/gridsize) - long(nz)
		));
	}
};


////
// MLS

class point_set_surface
{
protected:
	ANNpointArray pnt_adapter;
	ANNkd_tree kdtree;
	float avg_sample_dist, h;

	ANNpointArray adapt_points(const std::vector<vec3> &points)
	{
		for (unsigned i=0; i<points.size(); i++)
			pnt_adapter[i] = const_cast<ANNpoint>(&(points[i][0]));
		return pnt_adapter;
	}
	float calc_avg_sample_dist(const std::vector<vec3> &points)
	{
		#define MLS_AVG_MINDIST_SKIP 4
		float avg_sample_dist = 0;
		unsigned count = 0;
		for (unsigned i=0; i<points.size(); i+=MLS_AVG_MINDIST_SKIP, count++)
		{
			ANNidx idx[2]; ANNdist dists[2];
			kdtree.annkSearch(const_cast<ANNpoint>((const float*)(points[i])), 2, idx, dists);
			avg_sample_dist += std::sqrt(dists[1]);
		}
		avg_sample_dist /= float(count);
		return avg_sample_dist;
	}

public:
	point_set_surface (const std::vector<vec3> &points, float h)
		: pnt_adapter(new ANNpoint [points.size()]), kdtree(adapt_points(points), (int)points.size(), 3),
		  avg_sample_dist(calc_avg_sample_dist(points)), h(avg_sample_dist*h)
	{
	}
	~point_set_surface () { delete[] pnt_adapter; }

	vec3 projectPoint (const vec3 &point, vec3 *normal=nullptr)
	{
		#define MLS_MAX_NN 512
		ANNidx idx [MLS_MAX_NN];
		ANNdist dists [MLS_MAX_NN];

		vec3 normal_mls, pp = point;
		std::vector<vec3> nns;
		std::vector<float> weights;

		// Init at local area around closest point
		kdtree.annkSearch(const_cast<ANNpoint>((const float*)point), 1, idx, dists);
		pp = std::move(vec3(3, pnt_adapter[idx[0]]));

		// Iterative projection
		#define MLS_MAX_ITER 128
		const float epsilon = 8.0f*std::numeric_limits<float>::epsilon();
		unsigned iter = 0;
		for (; iter<MLS_MAX_ITER; iter++)
		{
			// Get nearest neighbors
			nns.reserve(32); weights.reserve(32);
			kdtree.annkFRSearch(pp, h*h, MLS_MAX_NN, idx, dists);
			for (unsigned i=0; idx[i]!=ANN_NULL_IDX; i++)
			{
				dists[i] = std::sqrt(dists[i]);
				nns.push_back(std::move(vec3(3, pnt_adapter[idx[i]])));
				weights.push_back(wendland(dists[i], h));
			}
			if (nns.size() < 4)
			{
				std::cout << "MLS projection failed!!!" << std::endl;
				return point;
			}

			// Find local best-fit plane
			float evals [3];
			vec3 evecs [3], mean;
			estimate_normal_wls<float>(nns.size(), nns[0], &(weights[0]), normal_mls, evals, mean, evecs[0]);
			normal_mls = cgv::math::normalize(normal_mls);

			// Project onto current local best-fit plane
			vec3 pp_new = plane_project(point, normal_mls, mean);

			// Convergence check
			float diff = (pp_new-pp).length();
			if (diff < epsilon)
				break;

			// Iterate
			pp = pp_new;
			nns.clear(); weights.clear();
		}

		// Optional return of normal
		if (normal)
			*normal = cgv::math::dot(normal_mls, *normal) >= 0 ? normal_mls : -normal_mls;

		// Done!
		return std::move(pp);
	}
};


////
// Implicits

template <class real>
struct quadric : public cgv::math::v3_func<real, real>
{
	real a_xx, a_xy, a_xz, a_xw,
	           a_yy, a_yz, a_yw,
	                 a_zz, a_zw,
	                       a_ww;

	vec_type axis;
	real     angle;

	vec_type transl;

	quadric()
		: axis(0, 1, 0)
	{
		a_xx = a_yy = a_zz = 1;
		angle = a_xy = a_xz = a_yz = a_xw = a_yw = a_zw = 0;
		a_ww = -1;
	}
	vec_type rotate(const vec_type& p, real ang) const
	{
		vec_type a = dot(p, axis)*axis;
		vec_type x = p - a;
		vec_type y = cross(axis, x);
		return a + std::cos(ang)*x + std::sin(ang)*y;
	}
	real evaluate(const pnt_type& p) const
	{
		return eval_quadric(rotate(p-transl, -angle));
	}
	real eval_quadric(const pnt_type& p) const
	{
		return   p(0)*(a_xx*p(0) + a_xy*p(1) + a_xz*p(2) + a_xw)
		       + p(1)*(            a_yy*p(1) + a_yz*p(2) + a_yw)
		       + p(2)*(                        a_zz*p(2) + a_zw)
		       + a_ww;
	}
};

template <class real>
struct metatube : public cgv::math::v3_func<real, real>
{
	// Types
	typedef ellipsoid_trajectory::vec3 vec3;
	typedef ellipsoid_trajectory::vec4 vec4;

	// External state
	const std::vector<vec3> *traj;
	const std::vector<vec4> *orientations;
	const vec3 *axes;

	// Internal state
	std::vector<vec3> positions_scaled;
	std::vector<cgv::media::axis_aligned_box<real, 3> > traj_bboxes;
	cgv::media::axis_aligned_box<real, 3> bbox_reference, bbox_sampling;
	quadric<real> quadric_template;
	std::deque<quadric<real> > ellipsoids;

private:
	void setup_state(void)
	{
		quadric_template.a_xx = axes->x();
		quadric_template.a_yy = axes->y();
		quadric_template.a_zz = axes->z();
		quadric_template.a_ww = real(-0.25);

		for (unsigned i=0; i<positions_scaled.size(); i++)
		{
			positions_scaled[i] = /*(*/traj->at(i)/* - origin) * scale*/;
			traj_bboxes[i] = std::move(translate_bbox(
				bbox_reference, positions_scaled[i]
			));
		}
	}
public:

	metatube(const std::vector<vec3> &traj, const std::vector<vec4> &orientations, const vec3 &axes)
		: traj(&traj), orientations(&orientations), axes(&axes), positions_scaled(traj.size()),
		  traj_bboxes(traj.size()), bbox_reference(get_ellipsoid_bbox<real>(axes))
	{
		setup_state();
	}

	void reset (const std::vector<vec3> &traj, const std::vector<vec4> &orientations, const vec3 &axes)
	{
		this->traj = &traj; this->orientations = &orientations; this->axes = &axes;
		positions_scaled.resize(traj.size());
		traj_bboxes.resize(traj.size());
		bbox_reference = get_ellipsoid_bbox(axes, scale);
		setup_state();
	}

	unsigned set_window(unsigned idx, real cellsize)
	{
		// ToDo: make it a parameter
		#ifdef _DEBUG
			#define SW_SKIP 4
		#else
			#define SW_SKIP 1
		#endif

		// Determine sampling box and reset window
		bbox_sampling = discretize(traj_bboxes[idx], cellsize);
		ellipsoids.clear();

		// Add trailing part
		unsigned next_idx;
		if (signed(idx)-SW_SKIP < 0)
			next_idx = 0;
		else if (idx%SW_SKIP == 0)
			next_idx = idx - SW_SKIP;
		else
			next_idx = idx - (idx%SW_SKIP);
		for (signed i=signed(idx)-SW_SKIP; i>=0; i-=SW_SKIP)
		{
			if (boxes_overlap(bbox_sampling, traj_bboxes[i]))
			{
				// Add ellipsoid to union
				ellipsoids.push_front(quadric_template);
				// - set translation
				ellipsoids.front().transl.set_extern_data(3, positions_scaled[i]);
				// - set rotation
				vec4 aa(std::move(quat_to_aa(orientations->at(i))));
				ellipsoids.front().axis.set(aa.x(), aa.y(), aa.z());
				ellipsoids.front().angle = aa.w();
			}
			else break;
		}

		// Current center
		ellipsoids.push_back(quadric_template);
		ellipsoids.back().transl.set_extern_data(3, positions_scaled[idx]);
		vec4 aa(std::move(quat_to_aa(orientations->at(idx))));
		ellipsoids.back().axis.set(aa.x(), aa.y(), aa.z());
		ellipsoids.back().angle = aa.w();

		// Add leading part
		if (idx+SW_SKIP < positions_scaled.size())
			next_idx = idx+SW_SKIP;
		else if (positions_scaled.size() > 1)
			next_idx = unsigned(positions_scaled.size());
		for (unsigned i=next_idx; i<positions_scaled.size(); i+=SW_SKIP)
		{
			// Keep track of last ellipsoid to be positioned in the current sampling area
			if (point_in_box(bbox_sampling, positions_scaled[i]))
				next_idx = i;
			// Add ellipsoid if it still contributes within current sampling area
			if (boxes_overlap(bbox_sampling, traj_bboxes[i]))
			{
				ellipsoids.push_back(quadric_template);
				ellipsoids.back().transl.set_extern_data(3, positions_scaled[i]);
				aa = std::move(quat_to_aa(orientations->at(i)));
				ellipsoids.back().axis.set(aa.x(), aa.y(), aa.z());
				ellipsoids.back().angle = aa.w();
			}
			else break;
		}
		// Check if the first leading ellipsoid sample to not be included was omitted only because its the very
		// last sample and the number of samples is not divisible by the skip count
		unsigned remainder = unsigned(positions_scaled.size()) - next_idx;
		if (remainder <= SW_SKIP && remainder > 0)
		{
			if (point_in_box(bbox_sampling, positions_scaled.back()))
				next_idx = unsigned(positions_scaled.size())-1;
			if (boxes_overlap(bbox_sampling, traj_bboxes.back()))
			{
				ellipsoids.push_back(quadric_template);
				ellipsoids.back().transl.set_extern_data(3, positions_scaled.back());
				aa = std::move(quat_to_aa(orientations->back()));
				ellipsoids.back().axis.set(aa.x(), aa.y(), aa.z());
				ellipsoids.back().angle = aa.w();
			}
		}

		// Report index of next sample to continue surface extraction with
		return next_idx;
	}

	real evaluate(const pnt_type& p) const
	{
		real min_val = std::numeric_limits<real>::infinity();

		//#pragma omp parallel for reduction(min : min_val)
		for (signed i=0; i<signed(ellipsoids.size()); i++)
		{
			real val = ellipsoids[i].evaluate(p);
			/*if (val < min_val)
				min_val = val;*/
			min_val = val < min_val ? val : min_val;
		}
		return min_val;
	}
};


////
// CGV surface extraction interface stuff

template <class real>
struct surface_extraction_handler : public cgv::media::mesh::streaming_mesh_callback_handler
{
	// State
	cgv::media::mesh::streaming_mesh<float> *mesh;
	std::unordered_map<voxel<real>, unsigned, typename voxel<real>::hash> duplicate_check;
	std::vector<cgv::math::fvec<real, 3> > points;
	std::vector<cgv::math::fvec<real, 3> > normals;
	std::vector<triangle> triangles;
	real cellsize;

	// Helpers
	bool check_duplicate (const cgv::math::fvec<real, 3> &point, real tolerance, voxel<real> *vxl)
	{
		// Init
		real tol_sqr = tolerance*tolerance;
		voxel<real> center(point, tolerance);
		
		// Check center voxel
		auto it = duplicate_check.find(center);
		if (it != duplicate_check.end())
			if ((point - points[it->second]).sqr_length() < tol_sqr)
				return false;

		// Check 26-neighborhood
		for (signed dz=-1; dz<2; dz++)
		for (signed dy=-1; dy<2; dy++)
		for (signed dx=-1; dx<2; dx++)
		if (dx!=0 || dy!=0 || dz!=0)
		{
			voxel<real> check(center.x+dx, center.y+dy, center.z+dz);
			it = duplicate_check.find(check);
			if (it != duplicate_check.end())
				if ((point - points[it->second]).sqr_length() < tol_sqr)
					return false;
		}

		// Not a duplicate
		*vxl = center;
		return true;
	}

	// Streaming-mesh callbacks (currently unused)
	virtual void new_vertex(unsigned int vi)
	{
		const real pos_epsilon = real(0.5)*cellsize;
		voxel<real> cell;
		if (check_duplicate(mesh->vertex_location(vi), pos_epsilon, &cell))
		{
			points.push_back(mesh->vertex_location(vi));
			normals.push_back(mesh->vertex_normal(vi));
			duplicate_check[cell] = (unsigned)points.size()-1;
		}
	}
	virtual void new_polygon(const std::vector<unsigned int>& vis)
	{
	}
	virtual void before_drop_vertex(unsigned int vi)
	{
	}

	// Export underlying mesh as Wavefront .obj to given stream
	void write_obj(std::ofstream &objfile)
	{
		// Write vertices
		for (unsigned i=0; i<points.size(); i++)
			objfile << "v " << points[i].x() << " " << points[i].y() << " " << points[i].z() << std::endl;

		// Write normals
		for (unsigned i=0; i<points.size(); i++)
			objfile << "vn " << normals[i].x() << " " << normals[i].y() << " " << normals[i].z() << std::endl;
	}
	// Export underlying mesh as Stanford .ply to given stream
	void write_ply(std::ofstream &plyfile)
	{
		// *.ply header
		plyfile << "ply" << std::endl
		        << "format ascii 1.0" << std::endl
		        << "element vertex " << points.size() << std::endl
		        << "property float x" << std::endl
		        << "property float y" << std::endl
		        << "property float z" << std::endl
		        << "property float nx" << std::endl
		        << "property float ny" << std::endl
		        << "property float nz" << std::endl
		        << "element face " << triangles.size() << std::endl
		        << "property list uchar int vertex_indices" << std::endl
		        << "end_header" << std::endl;

		// *.ply specific floating point settings
		plyfile.setf(std::ios::fixed); plyfile.precision(6);

		// Write vertex data
		for (unsigned i=0; i<points.size(); i++)
			plyfile
				<<  points[i].x() << " " <<  points[i].y() << " " <<  points[i].z() << " "
				<< normals[i].x() << " " << normals[i].y() << " " << normals[i].z() << std::endl;

		// Write triangle data
		for (unsigned i=0; i<triangles.size(); i++)
			plyfile
				<< 3 << " " << triangles[i][0] << " " << triangles[i][1] << " " << triangles[i][2] << " "
				<< std::endl;
	}
};



//////
//
// (Sub-)Module namespace-ing [END]
//

// Top level
}; // namespace ellipsoid_trajectory
