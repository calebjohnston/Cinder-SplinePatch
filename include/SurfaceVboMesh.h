#pragma once

#include "cinder/gl/Vbo.h"
#include "cinder/Vector.h"

#include "ParametricSurface.h"

/**
 * @brief A surface object that generates a VboMesh from a ParametricSurface
 *
 * SurfaceVboMesh is an object that represents a VboMesh that is
 * formed and shaped by an external parametric surface. All modifications
 * to the geometry are performed directly on the VBO data instead of an
 * internally stored TriMesh object. Thus, it acts much like a GPU dual
 * to the SurfaceTriMesh class.
 * 
 * The updateSurface method and constructor will generate triangle 
 * indices, vertices, normals, and texture coordinates.
 *
 * @see cg::SurfaceTriMesh
 * @see cg::ParametricSurface
 * @see cg::BSplinePatch
 * @see cg::NurbsPatch
 */
class SurfaceVboMesh {
public:
	/** C'stors */
	SurfaceVboMesh() { mNumSamples[0] = mNumSamples[1] = 0; }
	SurfaceVboMesh(const ParametricSurface& surface, const uint32_t numUSamples, const uint32_t numVSamples,
						   const ci::vec2& texcoordMin, const ci::vec2& texcoordMax);
	
	/** D'stor */
	virtual ~SurfaceVboMesh() {} // By default all members will be deallocated as desired
	
	/** Updates the surface geometry using the input parametric surface */
	void updateSurface(const ParametricSurface& surface);

	/** 
	 * Returns the number of quads (or tri pairs) created on the
	 * dimension specified by the input parameter (0 == x, 1 == y)
	 */
	inline uint32_t getNumSamples(const uint16_t dim) const { return mNumSamples[dim]; }
	
	/** Returns a copy of the internal VboMesh instance */
	inline ci::gl::VboMesh vboMesh() const { return mVboMesh; }
	
	/**
	 * These overloaded operators emulate the shared_ptr behavior found
	 * in the VboMesh class using the unspecified_bool_type operator.
	 * The first operator allows one to draw the SurfaceVboMesh using the
	 * same method used to draw a normal VboMesh -- gl::draw(const VboMesh&)
	 */
	operator const ci::gl::VboMesh&() const { return mVboMesh; }
	operator const bool() const { return mVboMesh; }

protected:
	uint32_t mNumSamples[2];	//!< Stores the number of geometric quads (or tri pairs) to create on either axis
	ci::gl::VboMesh mVboMesh;	//!< Internal VBO mesh object used for manipulation and drawing 
};
