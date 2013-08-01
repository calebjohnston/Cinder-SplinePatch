#include <cassert>
#include <vector>

#include "cinder/Vector.h"

#include "ParametricSurface.h"
#include "SurfaceTriMesh.h"

///////////////////////////////////////////////////////////////////////////
//
// TODO:	null
//
///////////////////////////////////////////////////////////////////////////

using namespace ci;
using namespace cg;

SurfaceTriMesh::SurfaceTriMesh(const ParametricSurface& surface,
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

	// set vertex positions, normals, and texture coordinates within buffer
    uint32_t uIndex, vIndex, i;
    for (uIndex = 0, i = 0; uIndex < mNumSamples[0]; ++uIndex) {
        float uIncr = uDelta*uIndex;
        float u = uMin + uIncr;
        for (vIndex = 0; vIndex < mNumSamples[1]; ++vIndex, ++i) {
            float vIncr = vDelta*vIndex;
            float v = vMin + vIncr;
			// add normal vector and vertex position
			Vec3f position, tan0, tan1, normal;
			surface.getFrame(u, v, position, tan0, tan1, normal);
			mTriMesh.appendVertex(position);
			mTriMesh.appendNormal(normal.inverse());	// the triangle winding is wrong, requires inverse of normal
			// add texture coordinate
			Vec2f tex_coord(tcoordMin.x + tuDelta * uIncr, tcoordMin.y + tvDelta * vIncr);
			mTriMesh.appendTexCoord(tex_coord);
        }
    }

	// set the index buffer values
	int i0,i1,i2,i3;
	for (uIndex = 0, i = 0; uIndex < mNumSamples[0] - 1; ++uIndex) {
		i0 = i;
		i1 = i0 + 1;
		i += mNumSamples[1];
		i2 = i;
		i3 = i2 + 1;
		for (vIndex = 0; vIndex < mNumSamples[1] - 1; ++vIndex) {
			mTriMesh.appendTriangle(i0,i1,i2);
			mTriMesh.appendTriangle(i1,i3,i2);
			i0++;
			i1++;
			i2++;
			i3++;
		}
	}
}

void SurfaceTriMesh::updateSurface(const ParametricSurface& surface)
{
	// collect surface information
    float uMin = surface.getUMin();
    float uDelta = (surface.getUMax() - uMin) / static_cast<float>(mNumSamples[0] - 1);
    float vMin = surface.getVMin();
    float vDelta = (surface.getVMax() - vMin) / static_cast<float>(mNumSamples[1] - 1);
	
	// update all vertices and normals within the vertex buffer
	float u, v;
	uint32_t uIndex, vIndex, i;
	std::vector<Vec3f>& verts = mTriMesh.getVertices();
	std::vector<Vec3f>& norms = mTriMesh.getNormals();
	std::vector<Vec3f>::iterator vert_itr = verts.begin();
	std::vector<Vec3f>::iterator norm_itr = norms.begin();
    for (uIndex = 0, i = 0; uIndex < mNumSamples[0]; ++uIndex) {
        u = uMin + uDelta*uIndex;
        for (vIndex = 0; vIndex < mNumSamples[1]; ++vIndex, ++i, ++vert_itr, ++norm_itr) {
            v = vMin + vDelta*vIndex;
			vert_itr->set(surface.position(u,v));
			norm_itr->set(surface.normal(u,v).inverse());	// the triangle winding is wrong, requires inverse of normal
        }
    }
}
