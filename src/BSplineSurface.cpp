#include <cassert>
#include <vector>

#include "cinder/Vector.h"
#include "cinder/CinderAssert.h"
#include "cinder/CinderGlm.h"

#include "BSplineSurface.h"

using namespace std;
using namespace ci;
using namespace ci::geom;

BSplineSurface::BSplineSurface(const BSplinePatch* patch, const ivec2& subdivisions )
:	Source(), mPatch( patch ), mMinTexCoord(0,0), mMaxTexCoord(1,1)
{
	init( subdivisions );
}

BSplineSurface&	BSplineSurface::texCoords( const vec2 &minCoord, const vec2 &maxCoord )
{
	mMinTexCoord = minCoord;
	mMaxTexCoord = maxCoord;
	
	return *this;
}

uint8_t BSplineSurface::getAttribDims( Attrib attr ) const
{
	switch( attr ) {
		case Attrib::POSITION: return 3;
		case Attrib::NORMAL: return 3;
		case Attrib::TANGENT: return 3;
		case Attrib::BITANGENT: return 3;
		case Attrib::TEX_COORD_0: return 2;
		default:
			return 0;
	}
}

AttribSet BSplineSurface::getAvailableAttribs() const
{
	return { Attrib::POSITION, Attrib::NORMAL, Attrib::TANGENT, Attrib::BITANGENT, Attrib::TEX_COORD_0 };
}

void BSplineSurface::loadInto( Target *target, const AttribSet &requestedAttribs ) const
{
	vector<vec3> positions, normals, tangents, bitangents;
	vector<vec2> texcoords;
	vector<uint32_t> indices;
	
	calculate( &positions, &normals, &tangents, &bitangents, &texcoords, &indices );

	target->copyAttrib( Attrib::POSITION, 3, 0, value_ptr( *positions.data() ), positions.size() );
	
	if( requestedAttribs.count( Attrib::NORMAL ) ) {
		target->copyAttrib( Attrib::NORMAL, 3, 0, value_ptr( *normals.data() ), normals.size() );
	}
	if( requestedAttribs.count( Attrib::TANGENT ) ) {
		target->copyAttrib( Attrib::TANGENT, 3, 0, value_ptr( *tangents.data() ), tangents.size() );
	}
	if( requestedAttribs.count( Attrib::BITANGENT ) ) {
		target->copyAttrib( Attrib::BITANGENT, 3, 0, value_ptr( *bitangents.data() ), bitangents.size() );
	}
	if( requestedAttribs.count( Attrib::TEX_COORD_0 ) ) {
		target->copyAttrib( Attrib::TEX_COORD_0, 2, 0, value_ptr( *texcoords.data() ), texcoords.size() );
	}

	target->copyIndices( Primitive::TRIANGLES, indices.data(), indices.size(), 1 );
}

void BSplineSurface::calculate( vector<vec3> *positions, vector<vec3> *normals,
							    vector<vec3> *tangents, vector<vec3> *bitangents,
							    vector<vec2> *texcoords, vector<uint32_t> *indices ) const
{
	CI_ASSERT_MSG( mPatch != nullptr, "The Bspline surface must be initialized with a Bspline patch." );
	
	positions->reserve( mNumVertices * sizeof(vec3) );
	if (normals) normals->reserve( mNumVertices * sizeof(vec3) );
	if (tangents) tangents->reserve( mNumVertices * sizeof(vec3) );
	if (bitangents) bitangents->reserve( mNumVertices * sizeof(vec3) );
	if (texcoords) texcoords->reserve( mNumVertices * sizeof(vec2) );
	indices->reserve( mNumVertices * sizeof(uint32_t) );
	
	// collect surface information
    float uMin = mPatch->getUMin();
    float uRange = mPatch->getUMax() - uMin;
    float uDelta = uRange / static_cast<float>(mSubdivisions.x - 1);
    float vMin = mPatch->getVMin();
    float vRange = mPatch->getVMax() - vMin;
    float vDelta = vRange / static_cast<float>(mSubdivisions.y - 1);
	
	// compute texture coordinate deltas
    float tuDelta = 0.0f, tvDelta = 0.0f;
	tuDelta = (mMaxTexCoord.x - mMinTexCoord.x) / uRange;
	tvDelta = (mMaxTexCoord.y - mMinTexCoord.y) / vRange;
	
	// set vertex positions, normals, and texture coordinates within buffer
    uint32_t uIndex, vIndex, i;
    for (uIndex = 0, i = 0; uIndex < mSubdivisions.x; ++uIndex) {
        float uIncr = uDelta*uIndex;
        float u = uMin + uIncr;
        for (vIndex = 0; vIndex < mSubdivisions.y; ++vIndex, ++i) {
            float vIncr = vDelta*vIndex;
            float v = vMin + vIncr;
			
			// add normal vector and vertex position
			vec3 position, tan0, tan1, normal;
			mPatch->getFrame(u, v, position, tan0, tan1, normal);
			positions->emplace_back( position );
			
			if (normals) normals->emplace_back( normal * vec3(-1) );	// the triangle winding is wrong, requires inverse of normal
			if (tangents) tangents->emplace_back( tan0 );
			if (bitangents) bitangents->emplace_back( tan1 );
			
			// add texture coordinate
			if (texcoords) {
				vec2 tex_coord(mMinTexCoord.x + tuDelta * uIncr, mMinTexCoord.y + tvDelta * vIncr);
				texcoords->emplace_back( tex_coord );
			}
        }
    }
	
	// set the index buffer values
	int i0,i1,i2,i3;
	for (uIndex = 0, i = 0; uIndex < mSubdivisions.x - 1; ++uIndex) {
		i0 = i;
		i1 = i0 + 1;
		i += mSubdivisions.y;
		i2 = i;
		i3 = i2 + 1;
		for (vIndex = 0; vIndex < mSubdivisions.y - 1; ++vIndex) {
			indices->emplace_back(i0);
			indices->emplace_back(i1);
			indices->emplace_back(i2);
			
			indices->emplace_back(i1);
			indices->emplace_back(i3);
			indices->emplace_back(i2);
			
			i0++;
			i1++;
			i2++;
			i3++;
		}
	}
}

void BSplineSurface::init( const ivec2& subdivisions )
{
	uint32_t subdX = std::max( 2, subdivisions.x );
	uint32_t subdY = std::max( 2, subdivisions.y );

	mSubdivisions = ivec2(subdX, subdY);
	mNumVertices = mSubdivisions.x * mSubdivisions.y;
}
