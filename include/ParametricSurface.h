#pragma once

#include <boost/multi_array.hpp>

#include "cinder/Vector.h"

namespace cg {
	
//! rectangular 2D grid of weights applied to a NURBS patch
typedef boost::multi_array<float, 2> PointWeightLatice;
//! rectangular 2D grid of 3D points which define a patch of splines
typedef boost::multi_array<ci::Vec3f, 2> ControlPointLatice;

/** 
 * @brief A surface defined by functions that are second order differentiable on two dimensions
 *
 * ParametricSurface is an abstract base class used to support
 * specific surface regions governed by spline curves. Use by
 * SurfaceVboMesh and SurfaceTriMesh classes to abstract the 
 * role of spline patches (BSplinePatch, NurbsPatch, etc).
 * 
 * The ParametricSurface class also defines special 2D arrays
 * for control points and control weights used by subclasses.
 *
 * @see cg::SurfaceTriMesh
 * @see cg::SurfaceVboMesh
 */
class ParametricSurface {
public:
    virtual ~ParametricSurface ();

    // The parametric domain is either rectangular or triangular.
	// Valid (u,v) values for a rectangular domain satisfy:
    //    umin <= u <= umax,  vmin <= v <= vmax
    // Valid (u,v) values for a triangular domain satisfy:
    //    umin <= u <= umax,  vmin <= v <= vmax,
    //    (vmax-vmin)*(u-umin)+(umax-umin)*(v-vmax) <= 0
    float getUMin() const { return mUMin; }
    float getUMax() const { return mUMax; }
    float getVMin() const { return mVMin; }
    float getVMax() const { return mVMax; }
    bool isRectangular() const { return mRectangular; }

    // position and derivatives up to second order
    virtual ci::Vec3f P(float u, float v) const = 0;
    virtual ci::Vec3f PU(float u, float v) const = 0;
    virtual ci::Vec3f PV(float u, float v) const = 0;
    virtual ci::Vec3f PUU(float u, float v) const = 0;
    virtual ci::Vec3f PUV(float u, float v) const = 0;
    virtual ci::Vec3f PVV(float u, float v) const = 0;

	ci::Vec3f tangent0(float u, float v) const;
	ci::Vec3f tangent1(float u, float v) const;
	ci::Vec3f position(float u, float v) const;
	ci::Vec3f normal(float u, float v) const;
	
    // Compute a coordinate frame. The set {T0,T1,N} is a right-handed
    // orthonormal set.
    void getFrame(float u, float v, ci::Vec3f& position, ci::Vec3f& tangent0,
        ci::Vec3f& tangent1, ci::Vec3f& normal) const;

    /**
	 * Computes differential geometric quantities. The returned scalars are
	 * the principal curvatures and the returned vectors are the corresponding
	 * principal directions.
	 *
	 * @param u the location on the x dimension to sample the surface at
	 * @param v the location on the y dimension to sample the surface at
	 * @param curve0
	 * @param curve1
	 * @param direction0
	 * @param direction1
	 */
    void computePrincipalCurvatureInfo(float u, float v, float& curve0, float& curve1, ci::Vec3f& direction0, ci::Vec3f& direction1);

protected:
    ParametricSurface(float umin, float umax, float vmin, float vmax, bool rectangular);
	
    float mUMin, mUMax, mVMin, mVMax;
    bool mRectangular;
};

}	// namespace cg
