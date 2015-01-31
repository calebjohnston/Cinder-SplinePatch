#pragma once

#include "cinder/BSpline.h"
#include "cinder/Vector.h"

#include <boost/multi_array.hpp>

//! rectangular 2D grid of 3D points which define a patch of splines
typedef boost::multi_array<ci::vec3, 2> ControlPointLatice;

/**
 * @brief A surface defined by a 2D patch of BSplines
 *
 * BSplinePatch is a parametric surface whose geometry
 * is governed by a 2D rectangular grid of spline points.
 *
 * @see cinder::BSpline
 * @see cg::SurfaceTriMesh
 * @see cg::SurfaceVboMesh
 */
class BSplinePatch {
public:
	BSplinePatch();
	
	// Spline types for curves are:
	//    open uniform (OU)
	//    periodic uniform (PU)
	//    open non-uniform (ON)
	// For tensor product surfaces, you have to choose a type for each of two
	// dimensions, leading to nine possible spline types for surfaces. The
	// constructors below represent these choices.
	
	// (OU,OU), (OU,PU), (PU,OU), or (PU,PU)
	BSplinePatch(const ControlPointLatice& ctrlPoints, const uint32_t uDegree,
				 const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, bool vOpen);
	
	// (OU,ON) or (PU,ON)
	BSplinePatch(const ControlPointLatice& ctrlPoints, const uint32_t uDegree,
				 const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, float* vKnot);
	
	// (ON,OU) or (ON,PU)
	BSplinePatch(const ControlPointLatice& ctrlPoints, const uint32_t uDegree,
				 const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, bool vOpen);
	
	// (ON,ON)
	BSplinePatch(const ControlPointLatice& ctrlPoints, const uint32_t uDegree,
				 const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, float* vKnot);
	
//	BSplinePatch( const BSplinePatch& bspline );
//	BSplinePatch& operator=( const BSplinePatch& bspline );
	virtual ~BSplinePatch();
	
	uint32_t getNumControlPoints(const uint16_t dim) const { return mNumCtrlPoints[dim]; }
	uint32_t getDegree(const uint16_t dim) const { return mBasis[dim].getDegree(); }
	uint32_t getNumSpans(const uint16_t dim) const { return mNumCtrlPoints[dim] - mBasis[dim].getDegree(); }
	bool isOpen(const uint16_t dim) const { return mBasis[dim].isOpen(); }
	bool isUniform(const uint16_t dim) const { return mBasis[dim].isUniform(); }
	bool isLoop(const uint16_t dim) const { return mLoop[dim]; }

	// Control points may be changed at any time. If either input index is
	// invalid, GetControlPoint returns a vector whose components are all MAX_REAL.
	void setControlPoint(const uint32_t uIndex, const uint32_t vIndex, const ci::vec3& point);
	ci::vec3 getControlPoint(const uint32_t uIndex, const uint32_t vIndex) const;
		
	/** */
	void setControlPointLatice(const ControlPointLatice& ctrlPoints);
	/** */
	ControlPointLatice getControlPointLatice() const;

	// The knot values can be changed only if the surface is nonuniform in the
	// selected dimension and only if the input index is valid. If these
	// conditions are not satisfied, GetKnot returns MAX_REAL.
	void setKnot(const uint32_t dim, const uint32_t i, float knot);
	float getKnot(const uint32_t dim, const uint32_t i) const;

	// The spline is defined for 0 <= u <= 1 and 0 <= v <= 1. The input
	// values should be in this domain. Any inputs smaller than 0 are clamped
	// to 0. Any inputs larger than 1 are clamped to 1.
	ci::vec3 P(float u, float v) const;
	ci::vec3 PU(float u, float v) const;
	ci::vec3 PV(float u, float v) const;
	ci::vec3 PUU(float u, float v) const;
	ci::vec3 PUV(float u, float v) const;
	ci::vec3 PVV(float u, float v) const;
	
	// ------------------------------------------------------------------------
	//	ci::vec3 getPosition(float u, float v) const;
	//	ci::vec3 getFirstDerivative(float u, float v, uint16_t dim) const;
	//	ci::vec3 getSecondDerivative(float u, float v, uint16_t dim) const;
	//	ci::vec3 getThirdDerivative(float u, float v, uint16_t dim) const;
	//	ci::vec3 getGradient(float u, float v) const;
	// ------------------------------------------------------------------------
	
	// The parametric domain is either rectangular or triangular.
	// Valid (u,v) values for a rectangular domain satisfy:
	//    umin <= u <= umax,  vmin <= v <= vmax
	// Valid (u,v) values for a triangular domain satisfy:
	//    umin <= u <= umax,  vmin <= v <= vmax,
	//    (vmax-vmin)*(u-umin)+(umax-umin)*(v-vmax) <= 0
	inline float getUMin() const { return mUMin; }
	inline float getUMax() const { return mUMax; }
	inline float getVMin() const { return mVMin; }
	inline float getVMax() const { return mVMax; }
	inline bool isRectangular() const { return mRectangular; }
	
	ci::vec3 tangent0(float u, float v) const;
	ci::vec3 tangent1(float u, float v) const;
	ci::vec3 position(float u, float v) const;
	ci::vec3 normal(float u, float v) const;
	
	// Compute a coordinate frame. The set {T0,T1,N} is a right-handed
	// orthonormal set.
	void getFrame(float u, float v, ci::vec3& position, ci::vec3& tangent0,
				  ci::vec3& tangent1, ci::vec3& normal) const;
	
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
	void computePrincipalCurvatureInfo(float u, float v, float& curve0, float& curve1, ci::vec3& direction0, ci::vec3& direction1);
	
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
	void get(float u, float v, ci::vec3* pos, ci::vec3* derU, ci::vec3* derV, 
			 ci::vec3* derUU, ci::vec3* derUV, ci::vec3* derVV) const;
	
	void create(const ControlPointLatice& ctrlPoints, const uint32_t uDegree,
				const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, bool vOpen);
	
	void create(const ControlPointLatice& ctrlPoints, const uint32_t uDegree,
				const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, float* vKnot);
	
	void create(const ControlPointLatice& ctrlPoints, const uint32_t uDegree,
				const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, bool vOpen);
	
	void create(const ControlPointLatice& ctrlPoints, const uint32_t uDegree,
				const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, float* vKnot);
	
protected:	
	// Replicate the necessary number of control points when the create
	// function has bLoop equal to true, in which case the spline surface
	// must be a closed surface in the corresponding dimension.
	void createControls(const ControlPointLatice& ctrlPoints);
	
	float mUMin, mUMax, mVMin, mVMax;
	bool mRectangular;
	
	bool mLoop[2];						//!< Stores whether or not the basis function loops in either dimension
	uint32_t mReplicate[2];				//!< Stores whether or not the points are replicated in either dimension
	uint32_t mNumCtrlPoints[2];			//!< A count of the total number of control points in either dimension
	ci::BSplineBasis mBasis[2];			//!< Basis function used in either dimension
	ControlPointLatice mControlPoints;	//!< Rectangular latice of control points used with the spline basis functions

};
