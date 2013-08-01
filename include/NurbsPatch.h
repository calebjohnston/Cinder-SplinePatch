#pragma once

#include "cinder/BSpline.h"
#include "cinder/Vector.h"

#include "ParametricSurface.h"

namespace cg {

/**
 * @brief A surface defined by a 2D patch of NURBS
 *
 * NurbsPatch is a parametric surface whose geometry
 * is governed by a 2D rectangular grid of spline points
 * weighted by a corresponding 2D rectangular grid of 
 * floating point values.
 *
 * @see cinder::BSpline
 * @see cg::ParametricSurface
 * @see cg::SurfaceTriMesh
 * @see cg::SurfaceVboMesh
 */
class NurbsPatch : public ParametricSurface {
public:
	NurbsPatch();

    // The homogeneous input points are (x,y,z,w) where the (x,y,z) values are
    // stored in the akCtrlPoint array and the w values are stored in the
    // afCtrlWeight array.  The output points from curve evaluations are of
    // the form (x',y',z') = (x/w,y/w,z/w).

    // Spline types for curves are
    //    open uniform (OU)
    //    periodic uniform (PU)
    //    open nonuniform (ON)
    // For tensor product surfaces, you have to choose a type for each of two
    // dimensions, leading to nine possible spline types for surfaces.  The
    // constructors below represent these choices.

    // (OU,OU), (OU,PU), (PU,OU), or (PU,PU)
    NurbsPatch(const ControlPointLatice& ctrlPoints, const PointWeightLatice& ctrlWeights, 
        const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, bool vOpen);

    // (OU,ON) or (PU,ON)
    NurbsPatch(const ControlPointLatice& ctrlPoints, const PointWeightLatice& ctrlWeights, 
        const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, float* vKnot);

    // (ON,OU) or (ON,PU)
    NurbsPatch(const ControlPointLatice& ctrlPoints, const PointWeightLatice& ctrlWeights, 
        const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, bool vOpen);

    // (ON,ON)
    NurbsPatch(const ControlPointLatice& ctrlPoints, const PointWeightLatice& ctrlWeights, 
        const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, float* vKnot);

    virtual ~NurbsPatch();
	
	uint32_t getNumControlPoints(const uint16_t dim) const { return mNumCtrlPoints[dim]; }
	uint32_t getDegree(const uint16_t dim) const { return mBasis[dim].getDegree(); }
	uint32_t getNumSpans(const uint16_t dim) const { return mNumCtrlPoints[dim] - mBasis[dim].getDegree(); }
	bool isOpen(const uint16_t dim) const { return mBasis[dim].isOpen(); }
	bool isUniform(const uint16_t dim) const { return mBasis[dim].isUniform(); }
	bool isLoop(const uint16_t dim) const { return mLoop[dim]; }

    // Control points and weights may be changed at any time. If either input
    // index is invalid, GetControlPoint returns a vector whose components
    // are all MAX_REAL, and GetControlWeight returns MAX_REAL.
    void setControlPoint(const uint32_t uIndex, const uint32_t vIndex, const ci::Vec3f& point);
    ci::Vec3f getControlPoint(const uint32_t uIndex, const uint32_t vIndex) const;
    void setControlWeight(const uint32_t uIndex, const uint32_t vIndex, float weight);
    float getControlWeight(const uint32_t uIndex, const uint32_t vIndex) const;

    // The knot values can be changed only if the surface is nonuniform in the
    // selected dimension and only if the input index is valid. If these
    // conditions are not satisfied, GetKnot returns MAX_REAL.
    void setKnot(int dim, int i, float knot);
    float getKnot(int dim, int i) const;

    // The spline is defined for 0 <= u <= 1 and 0 <= v <= 1. The input
    // values should be in this domain.  Any inputs smaller than 0 are clamped
    // to 0.  Any inputs larger than 1 are clamped to 1.
    virtual ci::Vec3f P(float u, float v) const;
    virtual ci::Vec3f PU(float u, float v) const;
    virtual ci::Vec3f PV(float u, float v) const;
    virtual ci::Vec3f PUU(float u, float v) const;
    virtual ci::Vec3f PUV(float u, float v) const;
    virtual ci::Vec3f PVV(float u, float v) const;

	/**
	 * Returns the position and derivatives in the parameter list for the 
	 * input uv location on the surface. If most of this data is needed at
	 * the same time, it is more efficient to call this function instead of
	 * each the explicit functions above. You may optionally pass NULL in
	 * any argument whose value you do not want computed.
	 *
	 * @param u the location along the horizontal axis of the surface patch
	 * @param v the location along the vertical axis of the surface patch
	 * @param pos the position of the surface at the location (u,v)
	 * @param derU the first derivative on the horizontal axis at the location (u,v)
	 * @param derV the first derivative on the vertical axis at the location (u,v)
	 * @param derUU the second derivative on the horizontal axis at the location (u,v)
	 * @param derUV the first order partial derivative at the location (u,v)
	 * @param derVV the second derivative on the vertical axis at the location (u,v)
	 */
    void get(float u, float v, ci::Vec3f* pos, ci::Vec3f* derU, ci::Vec3f* derV, 
        ci::Vec3f* derUU, ci::Vec3f* derUV, ci::Vec3f* derVV) const;
	
	void create(const ControlPointLatice& ctrlPoints, const PointWeightLatice& ctrlWeights,
			   const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, bool vOpen);
	
	void create(const ControlPointLatice& ctrlPoints, const PointWeightLatice& ctrlWeights,
			   const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, float* vKnot);
	
    void create(const ControlPointLatice& ctrlPoints, const PointWeightLatice& ctrlWeights,
			   const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, bool vOpen);
	
    void create(const ControlPointLatice& ctrlPoints, const PointWeightLatice& ctrlWeights,
			   const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, float* vKnot);

protected:
	/**
	 * Replicates the necessary number of control points when the create
	 * function has bLoop equal to true. In which case the spline surface
	 * must be a closed surface in the corresponding dimension.
	 * 
	 * @param ctrlPoints A 2d grid of 3d points used as the spline patch control points
	 * @param ctrlWeights A 2d grid of floats used to modulate the curvature contribution from the corresponding 3D point
	 */
    void createControl(const ControlPointLatice& ctrlPoints, const PointWeightLatice& ctrlWeights);

	bool mLoop[2];						//!< Stores whether or not the basis function loops in either dimension
	uint32_t mReplicate[2];				//!< Stores whether or not the points are replicated in either dimension
	uint32_t mNumCtrlPoints[2];			//!< A count of the total number of control points in either dimension
	ci::BSplineBasis mBasis[2];			//!< Basis function used in either dimension
	ControlPointLatice mControlPoints;	//!< Rectangular latice of control points used with the spline basis functions
	PointWeightLatice mControlWeights;	//!< Rectangular latice of weights applied to spline control points
};

}
