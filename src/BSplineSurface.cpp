#include <cassert>
#include <vector>

#include "cinder/Vector.h"
#include "cinder/CinderAssert.h"

#include "BSplineSurface.h"

///////////////////////////////////////////////////////////////////////////
//
// TODO:
//
///////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace ci;
using namespace ci::geom;

BSplineSurface::BSplineSurface(const BSplinePatch& patch, const ivec2& subdivisions )
:	Source(), mPatch( patch )
{
	init( subdivisions );
}

//BSplineSurface&	BSplineSurface::texCoords( const vec2 &upperLeft, const vec2 &upperRight, const vec2 &lowerRight, const vec2 &lowerLeft )
BSplineSurface&	BSplineSurface::texCoords( const vec2 &minCoord, const vec2 &maxCoord )
{
	// TODO:
//	mInputTexCoords[0] = upperLeft;
//	mInputTexCoords[1] = upperRight;
//	mInputTexCoords[2] = lowerLeft;
//	mInputTexCoords[3] = lowerRight;
	
	mMinTexCoord = minCoord;
	mMaxTexCoord = maxCoord;
	
	return *this;
}

void BSplineSurface::updateSurface(const BSplinePatch& patch)
{
	/*
	// collect surface information
    float uMin = patch.getUMin();
    float uDelta = (patch.getUMax() - uMin) / static_cast<float>(mSubdivisions.x - 1);
    float vMin = patch.getVMin();
    float vDelta = (patch.getVMax() - vMin) / static_cast<float>(mSubdivisions.y - 1);
	
	// update all vertices and normals within the vertex buffer
	float u, v;
	uint32_t uIndex, vIndex, i;
	vector<vec3>& verts = mTriMesh.getVertices();
	vector<vec3>& norms = mTriMesh.getNormals();
	vector<vec3>::iterator vert_itr = verts.begin();
	vector<vec3>::iterator norm_itr = norms.begin();
    for (uIndex = 0, i = 0; uIndex < mSubdivisions.x; ++uIndex) {
        u = uMin + uDelta*uIndex;
        for (vIndex = 0; vIndex < mSubdivisions.y; ++vIndex, ++i, ++vert_itr, ++norm_itr) {
            v = vMin + vDelta*vIndex;
			vert_itr->set(patch.position(u,v));
			norm_itr->set(patch.normal(u,v).inverse());	// the triangle winding is wrong, requires inverse of normal
        }
    }
	 */
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
	target->copyAttrib( Attrib::NORMAL, 3, 0, value_ptr( *normals.data() ), normals.size() );
	target->copyAttrib( Attrib::TANGENT, 3, 0, value_ptr( *tangents.data() ), tangents.size() );
	target->copyAttrib( Attrib::BITANGENT, 3, 0, value_ptr( *bitangents.data() ), bitangents.size() );
	target->copyAttrib( Attrib::TEX_COORD_0, 2, 0, value_ptr( *texcoords.data() ), texcoords.size() );

	target->copyIndices( Primitive::TRIANGLES, indices.data(), indices.size(), 1 );
}

void BSplineSurface::calculate( vector<vec3> *positions, vector<vec3> *normals,
							    vector<vec3> *tangents, vector<vec3> *bitangents,
							    vector<vec2> *texcoords, vector<uint32_t> *indices ) const
{
	positions->reserve( mNumVertices * sizeof(vec3) );
	normals->reserve( mNumVertices * sizeof(vec3) );
	tangents->reserve( mNumVertices * sizeof(vec3) );
	bitangents->reserve( mNumVertices * sizeof(vec3) );
	texcoords->reserve( mNumVertices * sizeof(vec2) );
	indices->reserve( mNumVertices * sizeof(uint32_t) );
	
	// collect surface information
    float uMin = mPatch.getUMin();
    float uRange = mPatch.getUMax() - uMin;
    float uDelta = uRange / static_cast<float>(mSubdivisions.x - 1);
    float vMin = mPatch.getVMin();
    float vRange = mPatch.getVMax() - vMin;
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
			mPatch.getFrame(u, v, position, tan0, tan1, normal);
			positions->emplace_back( position );
			normals->emplace_back( normal * vec3(-1) );	// the triangle winding is wrong, requires inverse of normal
			tangents->emplace_back( tan0 );
			bitangents->emplace_back( tan1 );
			
			// add texture coordinate
			vec2 tex_coord(mMinTexCoord.x + tuDelta * uIncr, mMinTexCoord.y + tvDelta * vIncr);
			texcoords->emplace_back( tex_coord );
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

//void BSplineSurface::init( const BSplinePatch& patch, const ivec2& subdivisions )
void BSplineSurface::init( const ivec2& subdivisions )
{
	uint32_t subdX = std::max( 2, subdivisions.x );
	uint32_t subdY = std::max( 2, subdivisions.y );

	mSubdivisions = ivec2(subdX, subdY);
	mNumVertices = mSubdivisions.x * mSubdivisions.y;
}
