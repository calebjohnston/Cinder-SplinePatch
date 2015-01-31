#pragma once

#include <array>
#include <vector>

#include "cinder/BSpline.h"
#include "cinder/CinderGlm.h"
#include "cinder/Vector.h"

/**
 * @brief A surface defined by a 2D patch of BSplines
 *
 * BSplinePatch is a parametric surface whose geometry
 * is governed by a 2D rectangular grid of spline points.
 *
 * @see cinder::BSpline
 * @see BSplineSurface
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
	BSplinePatch(const std::vector<ci::vec3>& ctrlPoints,
				 const ci::ivec2& size, const ci::ivec2& degree,
				 const glm::bvec2& loop, bool uOpen, bool vOpen);
	
	// (OU,ON) or (PU,ON)
	BSplinePatch(const std::vector<ci::vec3>& ctrlPoints,
				 const ci::ivec2& size, const ci::ivec2& degree,
				 const glm::bvec2& loop, bool uOpen, const std::vector<float>& vKnots);
	
	// (ON,OU) or (ON,PU)
	BSplinePatch(const std::vector<ci::vec3>& ctrlPoints,
				 const ci::ivec2& size, const ci::ivec2& degree,
				 const glm::bvec2& loop, const std::vector<float>& uKnots, bool vOpen);
	
	// (ON,ON)
	BSplinePatch(const std::vector<ci::vec3>& ctrlPoints,
				 const ci::ivec2& size, const ci::ivec2& degree, const glm::bvec2& loop, 
				 const std::vector<float>& uKnots, const std::vector<float>& vKnots);
	
	virtual ~BSplinePatch();
	
	uint32_t	getNumControlPoints(const uint8_t dim) const { return mNumCtrlPoints[dim]; }
	uint32_t	getDegree(const uint8_t dim) const { return mBasis[dim].getDegree(); }
	uint32_t	getNumSpans(const uint8_t dim) const { return mNumCtrlPoints[dim] - mBasis[dim].getDegree(); }
	bool		isUniform(const uint8_t dim) const { return mBasis[dim].isUniform(); }
	bool		isOpen(const uint8_t dim) const { return mBasis[dim].isOpen(); }
	bool		isLoop(const uint8_t dim) const { return mLoop[dim]; }

	// Control points may be changed at any time. If either input index is
	// invalid, GetControlPoint returns a vector whose components are all MAX_REAL.
	void		setControlPoint(const ci::ivec2& index, const ci::vec3& point);
	ci::vec3	getControlPoint(const ci::ivec2& index) const;
		
	/** */
	void		updateControlPoints(const std::vector<ci::vec3>& ctrlPoints, const ci::ivec2& size);
	/** */
	std::vector<ci::vec3>& getControlPoints() { return mControlPoints; }

	// The knot values can be changed only if the surface is nonuniform in the
	// selected dimension and only if the input index is valid. If these
	// conditions are not satisfied, GetKnot returns MAX_REAL.
	void		setKnot(const uint8_t dim, const uint32_t i, float knot);
	float		getKnot(const uint8_t dim, const uint32_t i) const;

	// The spline is defined for 0 <= u <= 1 and 0 <= v <= 1. The input
	// values should be in this domain. Any inputs smaller than 0 are clamped
	// to 0. Any inputs larger than 1 are clamped to 1.
	ci::vec3	P(float u, float v) const;
	ci::vec3	PU(float u, float v) const;
	ci::vec3	PV(float u, float v) const;
	ci::vec3	PUU(float u, float v) const;
	ci::vec3	PUV(float u, float v) const;
	ci::vec3	PVV(float u, float v) const;
	
	// ------------------------------------------------------------------------
	//	ci::vec3 getPosition(float u, float v) const;
	//	ci::vec3 getFirstDerivative(float u, float v, uint16_t dim) const;
	//	ci::vec3 getSecondDerivative(float u, float v, uint16_t dim) const;
	//	ci::vec3 getThirdDerivative(float u, float v, uint16_t dim) const;
	//	ci::vec3 getGradient(float u, float v) const;
	// ------------------------------------------------------------------------
	
	// The parametric domain is rectangular for values (u,v).
	// Valid (u,v) values for a rectangular domain satisfy:
	//    umin <= u <= umax,  vmin <= v <= vmax
	inline float getUMin() const { return mDomainMin.s; }
	inline float getUMax() const { return mDomainMax.s; }
	inline float getVMin() const { return mDomainMin.t; }
	inline float getVMax() const { return mDomainMax.t; }
	
	ci::vec3	tangent0(float u, float v) const;	// rename to tangent()
	ci::vec3	tangent1(float u, float v) const;	// rename to bitangent()
	ci::vec3	position(float u, float v) const;
	ci::vec3	normal(float u, float v) const;
	
	// Compute a coordinate frame. The set {T0,T1,N} is a right-handed orthonormal set.
	void		getFrame(float u, float v, ci::vec3& position, ci::vec3& tangent0,
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
	void		computePrincipalCurvatureInfo(float u, float v, float& curve0, float& curve1,
											  ci::vec3& direction0, ci::vec3& direction1);
	
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
	void		get(float u, float v, ci::vec3* pos, ci::vec3* derU, ci::vec3* derV,
					ci::vec3* derUU, ci::vec3* derUV, ci::vec3* derVV) const;
	
	// Access the basis function to compute it without control points.
	ci::BSplineBasis& getBasis(const uint8_t dim) { return mBasis[dim]; }
	
protected:	
	// Replicate the necessary number of control points when the create
	// function has bLoop equal to true, in which case the spline surface
	// must be a closed surface in the corresponding dimension.
	void createControls(const std::vector<ci::vec3>& ctrlPoints);
	
	ci::vec2 mDomainMin, mDomainMax;
	
	glm::bvec2 mLoop;						//!< Stores whether or not the basis function loops in either dimension
	ci::ivec2 mReplicate;					//!< Stores whether or not the points are replicated in either dimension
	ci::ivec2 mNumCtrlPoints;				//!< A count of the total number of control points in either dimension
	std::array<ci::BSplineBasis,2> mBasis;	//!< Basis function used in either dimension
	std::vector<ci::vec3> mControlPoints;	//!< Rectangular latice of control points used with the spline basis functions

};
