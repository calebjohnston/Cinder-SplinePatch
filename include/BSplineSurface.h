#pragma once

#include <array>
#include <vector>

#include "cinder/GeomIo.h"
#include "cinder/Vector.h"

#include "BSplinePatch.h"


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
 * @see BSplinePatch
 */
class BSplineSurface : public ci::geom::Source {
public:
	/** C'stors */
	BSplineSurface( const BSplinePatch& patch, const ci::ivec2& subdivisions = ci::ivec2(10) );
	
	//BSplineSurface&	texCoords( const ci::vec2 &upperLeft, const ci::vec2 &upperRight, const ci::vec2 &lowerRight, const ci::vec2 &lowerLeft );
	BSplineSurface&	texCoords( const ci::vec2 &minCoord, const ci::vec2 &maxCoord );
	BSplineSurface&	subdivisions( const ci::ivec2 samples ) { init(samples); return *this; }
	BSplineSurface&	subdivisionsX( const uint32_t xSamples ) { init(ci::ivec2(xSamples, mSubdivisions.y)); return *this; }
	BSplineSurface&	subdivisionsY( const uint32_t ySamples ) { init(ci::ivec2(mSubdivisions.x, ySamples)); return *this; }
	
	/** D'stor */
	virtual ~BSplineSurface() {};	// de-allocates members as expected
	
	/** Updates the surface geometry using the input parametric surface */
	void		updateSurface(const BSplinePatch& surface);
	
	size_t		getNumVertices() const override { return mNumVertices; }
	size_t		getNumIndices() const override { return mNumVertices; }
	ci::ivec2	getSubdivisions() const { return mSubdivisions; }
	ci::geom::Primitive	getPrimitive() const override { return ci::geom::Primitive::TRIANGLES; }
	uint8_t		getAttribDims( ci::geom::Attrib attr ) const override;
	ci::geom::AttribSet	getAvailableAttribs() const override;
	void		loadInto( ci::geom::Target *target, const ci::geom::AttribSet &requestedAttribs ) const override;
	
protected:
	// creates surface geometry
	void		calculate( std::vector<ci::vec3> *positions, std::vector<ci::vec3> *normals,
						   std::vector<ci::vec3> *tangents, std::vector<ci::vec3> *bitangents,
						   std::vector<ci::vec2> *texcoords, std::vector<uint32_t> *indices ) const;

	// verify input patch and subdivision parameters + updates vert count
	void		init( const ci::ivec2& subdivisions );

	const BSplinePatch&				mPatch;
	ci::ivec2						mSubdivisions;
	uint32_t						mNumVertices;
	//std::array<ci::vec2, 4>		mInputTexCoords;
	ci::vec2						mMinTexCoord, mMaxTexCoord;


};
