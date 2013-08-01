#pragma once

#include "cinder/TriMesh.h"
#include "cinder/Vector.h"

#include "ParametricSurface.h"

namespace cg {

/**
 * @brief A surface object that generates a TriMesh from a ParametricSurface
 *
 * SurfaceTriMesh is an object that represents a TriMesh that is formed
 * and shaped by an external parametric surface. All modifications to
 * the parametric surface are reflected in the internally stored TriMesh
 * object via the updateSurface method. This class acts much like a CPU
 * dual to the SurfaceVboMesh -where updates are written directly to vbo.
 *
 * The updateSurface method and constructor will generate triangle
 * indices, vertices, normals, and texture coordinates.
 *
 * @see cg::SurfaceVboMesh
 * @see cg::ParametricSurface
 * @see cg::BSplinePatch
 * @see cg::NurbsPatch
 */
class SurfaceTriMesh {
public:
	/** C'stors */
	SurfaceTriMesh() { mNumSamples[0] = mNumSamples[1] = 0; }
	SurfaceTriMesh(const ParametricSurface& surface, const uint32_t numUSamples, const uint32_t numVSamples,
						   const ci::Vec2f& texcoordMin, const ci::Vec2f& texcoordMax);

	/** D'stor */
	virtual ~SurfaceTriMesh() {};	// de-allocates members as expected
	
	/** Updates the surface geometry using the input parametric surface */
	void updateSurface(const ParametricSurface& surface);
	
	/**
	 * Returns the number of quads (or tri pairs) created on the
	 * dimension specified by the input parameter (0 == x, 1 == y)
	 */
	inline uint32_t getNumUSamples(const uint16_t dim) const { return mNumSamples[dim]; }

	/** Returns a copy of the internal TriMesh instance */
	inline ci::TriMesh trimesh() const { return mTriMesh; }
	
	/**
	 * These overloaded operators emulate the shared_ptr behavior found
	 * in the TriMesh class using the unspecified_bool_type operator.
	 * The first operator allows one to draw the SurfaceTriMesh using the
	 * same method used to draw a TriMesh -- gl::draw(const TriMesh&)
	 */
	operator const ci::TriMesh&() const { return mTriMesh; }
	operator const bool() const { return mTriMesh.getVertices().size() > 0; }
	
protected:
	uint32_t mNumSamples[2];	//!< Stores the number of geometric quads (or tri pairs) to create on either axis
	ci::TriMesh mTriMesh;		//!< Internal copy of the TriMesh object manipulated by a parametric surface
};
	
}	// namespace cg