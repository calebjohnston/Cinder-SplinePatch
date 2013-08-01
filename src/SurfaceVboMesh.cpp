#include <cassert>
#include <vector>

#include "cinder/gl/gl.h"

#include "ParametricSurface.h"
#include "SurfaceVboMesh.h"

///////////////////////////////////////////////////////////////////////////
//
// TODO:	null
//
///////////////////////////////////////////////////////////////////////////

using namespace ci;
using namespace cg;

SurfaceVboMesh::SurfaceVboMesh(const ParametricSurface& surface,
	const uint32_t numUSamples, const uint32_t numVSamples, const ci::Vec2f& tcoordMin, const ci::Vec2f& tcoordMax)
{
	assert(surface.isRectangular());
	
	mNumSamples[0] = numUSamples;
	mNumSamples[1] = numVSamples;
	
	// collect surface information
    float uMin = surface.getUMin();
    float uRange = surface.getUMax() - uMin;
    float uDelta = uRange / static_cast<float>(mNumSamples[0] - 1);
    float vMin = surface.getVMin();
    float vRange = surface.getVMax() - vMin;
    float vDelta = vRange / static_cast<float>(mNumSamples[1] - 1);

	// compute texture coordinate deltas
    float tuDelta = 0.0f, tvDelta = 0.0f;
	tuDelta = (tcoordMax.x - tcoordMin.x) / uRange;
	tvDelta = (tcoordMax.y - tcoordMin.y) / vRange;

	// compute vertex texture coordinates
	std::vector<uint32_t> indices;
	std::vector<Vec2f> tex_coords;
    uint32_t uIndex, vIndex, i;
    for (uIndex = 0, i = 0; uIndex < mNumSamples[0]; ++uIndex) {
        float uIncr = uDelta*uIndex;
        for (vIndex = 0; vIndex < mNumSamples[1]; ++vIndex, ++i) {
            float vIncr = vDelta*vIndex;
			Vec2f tex_coord(tcoordMin.x + tuDelta * uIncr, tcoordMin.y + tvDelta * vIncr);
			tex_coords.push_back(tex_coord);
        }
    }

	// compute vertex indices
	int i0,i1,i2,i3;
	for (uIndex = 0, i = 0; uIndex < mNumSamples[0] - 1; ++uIndex) {
		i0 = i;
		i1 = i0 + 1;
		i += mNumSamples[1];
		i2 = i;
		i3 = i2 + 1;
		for (vIndex = 0; vIndex < mNumSamples[1] - 1; ++vIndex) {
			indices.push_back(i0);
			indices.push_back(i1);
			indices.push_back(i2);
			indices.push_back(i1);
			indices.push_back(i3);
			indices.push_back(i2);
			i0++;
			i1++;
			i2++;
			i3++;
		}
	}
	
	// configure geometry layout
	gl::VboMesh::Layout layout;
	layout.setStaticIndices();
	layout.setStaticTexCoords2d();
	layout.setDynamicPositions();
	layout.setDynamicNormals();
	mVboMesh = gl::VboMesh(tex_coords.size(), indices.size(), layout, GL_TRIANGLES);
	
	// buffer all data
	mVboMesh.bufferIndices( indices );
	mVboMesh.bufferTexCoords2d( 0, tex_coords );
	updateSurface(surface);
}

void SurfaceVboMesh::updateSurface(const ParametricSurface& surface)
{
	// collect surface information
    float uMin = surface.getUMin();
    float uDelta = (surface.getUMax() - uMin) / static_cast<float>(mNumSamples[0] - 1);
    float vMin = surface.getVMin();
    float vDelta = (surface.getVMax() - vMin) / static_cast<float>(mNumSamples[1] - 1);
	
	// update all vertices and normals within the vertex buffer
	uint32_t uIndex, vIndex, i;
	float u, v;
	gl::VboMesh::VertexIter vert_itr = mVboMesh.mapVertexBuffer();
	for (uIndex = 0, i = 0; uIndex < mNumSamples[0]; ++uIndex) {
        u = uMin + uDelta*uIndex;
        for (vIndex = 0; vIndex < mNumSamples[1]; ++vIndex, ++i, ++vert_itr) {
            v = vMin + vDelta*vIndex;
			vert_itr.setPosition(surface.position(u,v));
			vert_itr.setNormal(surface.normal(u,v).inverse());	// the triangle winding is wrong, requires inverse of normal
        }
    }
}
