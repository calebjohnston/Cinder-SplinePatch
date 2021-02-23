#pragma once

#include <array>
#include <vector>

#include "cinder/BSpline.h"
#include "cinder/CinderGlm.h"
#include "cinder/Vector.h"

/**
 * @brief A surface defined by a 2D patch of BSplines. BSplinePatch is a parametric surface
 * whose geometry is controlled by a topologically rectangular grid of spline points.
 *
 * Spline types for curves are:
 * 	- open uniform (OU)
 *  - periodic uniform (PU)
 *  - open non-uniform (ON)
 * An "open" side means that the spline evaluation will be treated as a boundary at the edge
 * A "periodic" side means that the spline evaluation will wrap around to the opposite side
 * to form a continuous curve. The constructors reflect these edge type combinations.
 *
 * @see cinder::BSpline
 * @see BSplineSurface
 */
class BSplinePatch {
public:
	BSplinePatch();
	
	//! C'stor for (OU,OU), (OU,PU), (PU,OU), or (PU,PU)
	BSplinePatch(const std::vector<ci::vec3>& ctrlPoints,
				 const ci::ivec2& size, const ci::ivec2& degree,
				 const glm::bvec2& loop, bool uOpen, bool vOpen);
	
	//! C'stor for (OU,ON) or (PU,ON)
	BSplinePatch(const std::vector<ci::vec3>& ctrlPoints,
				 const ci::ivec2& size, const ci::ivec2& degree,
				 const glm::bvec2& loop, bool uOpen, const std::vector<float>& vKnots);
	
	//! C'stor for (ON,OU) or (ON,PU)
	BSplinePatch(const std::vector<ci::vec3>& ctrlPoints,
				 const ci::ivec2& size, const ci::ivec2& degree,
				 const glm::bvec2& loop, const std::vector<float>& uKnots, bool vOpen);
	
	//! C'stor for (ON,ON)
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

	/**
	 * Assigns new control point coordinate at the given index
	 *
	 * @param index the 2-dimensional index into the control point grid
	 * @param point the updated control point
	 */
	void		setControlPoint(const ci::ivec2& index, const ci::vec3& point);
	
	/**
	 * Returns the existing control point coordinate at the given index, or a 2D MAX FLOAT if input is out of bounds.
	 *
	 * @param index the 2-dimensional index into the control point grid
	 */
	ci::vec3	getControlPoint(const ci::ivec2& index) const;
		
	/** 
	 * Performs internal update of control points with input parameters.
	 * 
	 * @param ctrlPoints a rectangular grid of 3D points that serve as the control points for the patch
	 * @param size the number of control points on each axis
	 */
	void		updateControlPoints(const std::vector<ci::vec3>& ctrlPoints, const ci::ivec2& size);
	
	/** returns a reference to the internal vector of control points */
	std::vector<ci::vec3>& getControlPoints() { return mControlPoints; }

	/**
	 * The knot values can be changed only if the surface is nonuniform in the
	 * selected dimension and only if the input index is valid. If these
	 * conditions are not satisfied, GetKnot returns MAX_REAL.
	 */
	void		setKnot(const uint8_t dim, const uint32_t i, float knot);
	float		getKnot(const uint8_t dim, const uint32_t i) const;

	/** returns the position of the patch at the input coordinate (u,v) of the domain [0,1]. */
	ci::vec3	P(float u, float v) const;
	/** returns the first derivative on the horizontal axis at the location (u,v) of the domain [0,1]. */
	ci::vec3	PU(float u, float v) const;
	/** returns the first derivative on the vertical axis at the location (u,v) of the domain [0,1]. */
	ci::vec3	PV(float u, float v) const;
	/** returns the second derivative on the horizontal axis at the location (u,v) of the domain [0,1]. */
	ci::vec3	PUU(float u, float v) const;
	/** returns the first order partial derivative at the location (u,v) of the domain [0,1]. */
	ci::vec3	PUV(float u, float v) const;
	/** returns the second derivative on the vertical axis at the location (u,v) of the domain [0,1]. */
	ci::vec3	PVV(float u, float v) const;
	
	/**
	 * The parametric domain is rectangular for values (u,v).
	 * Valid (u,v) values for a rectangular domain satisfy:
	 * 		umin <= u <= umax,  vmin <= v <= vmax
	 */
	inline float getUMin() const { return mDomainMin.s; }
	inline float getUMax() const { return mDomainMax.s; }
	inline float getVMin() const { return mDomainMin.t; }
	inline float getVMax() const { return mDomainMax.t; }
	
	ci::vec3	tangent(float u, float v) const;
	ci::vec3	bitangent(float u, float v) const;
	ci::vec3	position(float u, float v) const;
	ci::vec3	normal(float u, float v) const;
	
	//! Compute a coordinate frame. The set {T0,T1,N} is a right-handed orthonormal set.
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
	void computePrincipalCurvatureInfo(float u, float v, float& curve0, float& curve1,
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
	void get(float u, float v, ci::vec3* pos, ci::vec3* derU, ci::vec3* derV,
					ci::vec3* derUU, ci::vec3* derUV, ci::vec3* derVV) const;
	
	//! Access the basis function to compute it without control points.
	ci::BSplineBasis& getBasis(const uint8_t dim) { return mBasis[dim]; }
	
protected:
	/*
	 * Copies input control points to internal structure. Replicates the necessary
	 * control points for looping edges (closed edges). The size of the input vector
	 * must match the expression: getNumControlPoints(0) * getNumControlPoints(1)
	 *
	 * @param ctrlPoints a rectangular grid of 3D points that serve as the control points for the patch
	 */
	void createControls(const std::vector<ci::vec3>& ctrlPoints);
	
	ci::vec2 mDomainMin, mDomainMax;		//!< Stores the domain over which the functions will be evaluated. Default: [0,1),[0,1)
	
	glm::bvec2 mLoop;						//!< Stores whether or not the basis function loops in either dimension
	ci::ivec2 mReplicate;					//!< Stores whether or not the points are replicated in either dimension
	ci::ivec2 mNumCtrlPoints;				//!< A count of the total number of control points in either dimension
	std::array<ci::BSplineBasis,2> mBasis;	//!< Basis function used in either dimension
	std::vector<ci::vec3> mControlPoints;	//!< Rectangular latice of control points used with the spline basis functions

};
