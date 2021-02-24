#pragma once

#include <array>
#include <vector>

#include "cinder/GeomIo.h"
#include "cinder/Vector.h"

#include "BSplinePatch.h"

/**
 * @brief The BSplineSurface geometry source will generate a set of points, normals, texture coordinates,
 * tangents, and bitangents for a given BSplinePatch and a mesh subdivision parameter.
 *
 * @see BSplinePatch
 * @see cinder::geom::Source
 */
class BSplineSurface : public ci::geom::Source {
public:
	/** C'stors */
	BSplineSurface( const BSplinePatch* patch = nullptr, const ci::ivec2& subdivisions = ci::ivec2(10) );
	
	BSplineSurface&	texCoords( const ci::vec2 &minCoord, const ci::vec2 &maxCoord );
	BSplineSurface&	subdivisions( const ci::ivec2 samples ) { init(samples); return *this; }
	BSplineSurface&	subdivisionsX( const uint32_t xSamples ) { init(ci::ivec2(xSamples, mSubdivisions.y)); return *this; }
	BSplineSurface&	subdivisionsY( const uint32_t ySamples ) { init(ci::ivec2(mSubdivisions.x, ySamples)); return *this; }
	
	/** D'stor */
	virtual ~BSplineSurface() {};	// de-allocates members as expected
	
	size_t		getNumVertices() const override { return mNumVertices; }
	size_t		getNumIndices() const override { return mNumVertices * 6; }
	ci::ivec2	getSubdivisions() const { return mSubdivisions; }
	ci::geom::Primitive	getPrimitive() const override { return ci::geom::Primitive::TRIANGLES; }
	uint8_t		getAttribDims( ci::geom::Attrib attr ) const override;
	ci::geom::AttribSet	getAvailableAttribs() const override;
	void		loadInto( ci::geom::Target *target, const ci::geom::AttribSet &requestedAttribs ) const override;
	Source*		clone() const override { return new BSplineSurface( mPatch, mSubdivisions ); }
	
protected:
	//! creates surface geometry using input vectors for positions, texture coordinates, indices, and a tangent space
	void		calculate( std::vector<ci::vec3> *positions, std::vector<ci::vec3> *normals,
						   std::vector<ci::vec3> *tangents, std::vector<ci::vec3> *bitangents,
						   std::vector<ci::vec2> *texcoords, std::vector<uint32_t> *indices ) const;

	//! verify input patch and subdivision parameters and updates vertex count
	void		init( const ci::ivec2& subdivisions );

	const BSplinePatch*				mPatch;
	ci::ivec2						mSubdivisions;
	uint32_t						mNumVertices;
	ci::vec2						mMinTexCoord, mMaxTexCoord;

};
