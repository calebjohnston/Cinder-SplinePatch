/**
 * Tensor calculations in computePrincipalCurvatureInfo method are sourced from:
 * Geometric Tools, LLC
 * Copyright (c) 1998-2010
 * Distributed under the Boost Software License, Version 1.0.
 * http://www.boost.org/LICENSE_1_0.txt
 * http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
 */

#include <limits>

#include "cinder/CinderAssert.h"
#include "cinder/CinderGlm.h"
#include "cinder/Log.h"
#include "cinder/Matrix.h"

#include "BSplinePatch.h"

#define ZERO_TOLERANCE 1e-06

using namespace ci;

BSplinePatch::BSplinePatch()
:	mDomainMin(0), mDomainMax(1)
{
	mLoop[0] = false;
	mLoop[1] = false;
	mReplicate = ivec2(0);
	mNumCtrlPoints = ivec2(0);
}

BSplinePatch::~BSplinePatch()
{
}

BSplinePatch::BSplinePatch(const std::vector<vec3>& ctrlPoints,
						   const ci::ivec2& size, const ci::ivec2& degree,
						   const glm::bvec2& loop, bool uOpen, bool vOpen)
:	mDomainMin(0), mDomainMax(1)
{
	CI_ASSERT_MSG( size.s >= 2, "Patch size must be greater than 1 on each both dimensions" );
	CI_ASSERT_MSG( size.t >= 2, "Patch size must be greater than 1 on each both dimensions" );
	
	ivec2 validDegree = glm::clamp( degree, ivec2(1), ivec2(size -1) );
	if (validDegree != degree)
		CI_LOG_W( "Degree value of [" << degree.s << "," << degree.t << "] is invalid for size [" << size.s << ","
				  << size.t << "]. Clamping degree to: [" << validDegree.s << "," << validDegree.t << "]" );
	
	mLoop[0] = loop.s;
    mLoop[1] = loop.t;
	
    mNumCtrlPoints = size;
    mReplicate.s = (loop.s ? (uOpen ? 1 : validDegree.s) : 0);
    mReplicate.t = (loop.t ? (vOpen ? 1 : validDegree.t) : 0);
	
    mBasis[0].create(mNumCtrlPoints.s + mReplicate.s, validDegree.s, uOpen);
    mBasis[1].create(mNumCtrlPoints.t + mReplicate.t, validDegree.t, vOpen);
	
    createControls(ctrlPoints);
}

BSplinePatch::BSplinePatch(const std::vector<vec3>& ctrlPoints,
						   const ci::ivec2& size, const ci::ivec2& degree,
						   const glm::bvec2& loop, bool uOpen, const std::vector<float>& vKnots)
:	mDomainMin(0), mDomainMax(1)
{
	CI_ASSERT_MSG( size.s >= 2, "Patch size must be greater than 1 on each both dimensions" );
	CI_ASSERT_MSG( size.t >= 2, "Patch size must be greater than 1 on each both dimensions" );
	CI_ASSERT_MSG( size.t - degree.t == vKnots.size(), "Knot vector must have (size - degree) elements." );
	
	ivec2 validDegree = glm::clamp( degree, ivec2(1), ivec2(size -1) );
	if (validDegree != degree)
		CI_LOG_W( "Degree value of [" << degree.s << "," << degree.t << "] is invalid for size [" << size.s << ","
				  << size.t << "]. Clamping degree to: [" << validDegree.s << "," << validDegree.t << "]" );
	
    mLoop[0] = loop.s;
    mLoop[1] = loop.t;
	
    mNumCtrlPoints = size;
    mReplicate.s = (loop.s ? (uOpen ? 1 : validDegree.s) : 0);
    mReplicate.t = (loop.t ? 1 : 0);
	
    mBasis[0].create(mNumCtrlPoints.s + mReplicate.s, validDegree.s, uOpen);
    mBasis[1].create(mNumCtrlPoints.t + mReplicate.t, validDegree.t, &vKnots[0]);
	
    createControls(ctrlPoints);
}

BSplinePatch::BSplinePatch(const std::vector<vec3>& ctrlPoints,
						   const ci::ivec2& size, const ci::ivec2& degree,
						   const glm::bvec2& loop, const std::vector<float>& uKnots, bool vOpen)
:	mDomainMin(0), mDomainMax(1)
{
	CI_ASSERT_MSG( size.s >= 2, "Patch size must be greater than 1 on each both dimensions" );
	CI_ASSERT_MSG( size.t >= 2, "Patch size must be greater than 1 on each both dimensions" );
	CI_ASSERT_MSG( size.s - degree.s == uKnots.size(), "Knot vector must have (size - degree) elements." );
	
	ivec2 validDegree = glm::clamp( degree, ivec2(1), ivec2(size -1) );
	if (validDegree != degree)
		CI_LOG_W( "Degree value of [" << degree.s << "," << degree.t << "] is invalid for size [" << size.s << ","
				  << size.t << "]. Clamping degree to: [" << validDegree.s << "," << validDegree.t << "]" );
	
    mLoop[0] = loop.s;
    mLoop[1] = loop.t;
	
    mNumCtrlPoints = size;
    mReplicate.s = (loop.s ? 1 : 0);
    mReplicate.t = (loop.t ? (vOpen ? 1 : validDegree.t) : 0);
	
    mBasis[0].create(mNumCtrlPoints.s + mReplicate.s, validDegree.s, &uKnots[0]);
    mBasis[1].create(mNumCtrlPoints.t + mReplicate.t, validDegree.t, vOpen);
	
    createControls(ctrlPoints);
}

BSplinePatch::BSplinePatch(const std::vector<vec3>& ctrlPoints,
						   const ci::ivec2& size, const ci::ivec2& degree, const glm::bvec2& loop,
						   const std::vector<float>& uKnots, const std::vector<float>& vKnots)
:	mDomainMin(0), mDomainMax(1)
{
	CI_ASSERT_MSG( size.s >= 2, "Patch size must be greater than 1 on each both dimensions" );
	CI_ASSERT_MSG( size.t >= 2, "Patch size must be greater than 1 on each both dimensions" );
	CI_ASSERT_MSG( size.s - degree.s == uKnots.size(), "Knot vector must have (size - degree) elements." );
	CI_ASSERT_MSG( size.t - degree.t == vKnots.size(), "Knot vector must have (size - degree) elements." );
	
	ivec2 validDegree = glm::clamp( degree, ivec2(1), ivec2(size -1) );
	if (validDegree != degree)
		CI_LOG_W( "Degree value of [" << degree.s << "," << degree.t << "] is invalid for size [" << size.s << ","
				  << size.t << "]. Clamping degree to: [" << validDegree.s << "," << validDegree.t << "]" );
	
    mLoop[0] = loop.s;
    mLoop[1] = loop.t;
	
    mNumCtrlPoints = size;
    mReplicate.s = (loop.s ? 1 : 0);
    mReplicate.t = (loop.t ? 1 : 0);
	
    mBasis[0].create(mNumCtrlPoints.s + mReplicate.s, validDegree.s, &uKnots[0]);
    mBasis[1].create(mNumCtrlPoints.t + mReplicate.t, validDegree.t, &vKnots[0]);
	
    createControls(ctrlPoints);
}

void BSplinePatch::createControls(const std::vector<vec3>& ctrlPoints)
{
	// NOT CURRENTLY SUPPORTING A RESIZE OF CONTROL POINT GRID!
	CI_ASSERT_MSG(ctrlPoints.size() == mNumCtrlPoints.s * mNumCtrlPoints.t, "The implementation does not currently support resizing the control point grid.");
	
    const uint32_t ctrlPtsRow = mNumCtrlPoints.s + mReplicate.s;
    const uint32_t ctrlPtsCol = mNumCtrlPoints.t + mReplicate.t;
	
	mControlPoints.clear();
	mControlPoints.reserve( ctrlPtsRow * ctrlPtsCol );
	if (ctrlPtsRow == mNumCtrlPoints.s && ctrlPtsCol == mNumCtrlPoints.t) {
		mControlPoints = ctrlPoints;
	}
	else {
		mControlPoints.resize( ctrlPtsRow * ctrlPtsCol );
		for (size_t u = 0; u < ctrlPtsRow; ++u) {
			int uOld = u % mNumCtrlPoints.s;
			for (size_t v = 0; v < ctrlPtsCol; ++v) {
				int vOld = v % mNumCtrlPoints.t;
				mControlPoints[u * ctrlPtsCol + v] = ctrlPoints[uOld * mNumCtrlPoints.t + vOld];
			}
		}
	}
}

void BSplinePatch::setControlPoint(const ci::ivec2& index, const vec3& point)
{
    if (index.s < mNumCtrlPoints.s && index.t < mNumCtrlPoints.t) {
		mControlPoints[index.s * mNumCtrlPoints.t + index.t] = point;

        // Set the replicated control point.
        bool uReplicate = (index.s < mReplicate.s);
        bool vReplicate = (index.t < mReplicate.t);
        uint32_t iUExt = 0, iVExt = 0;

        if (uReplicate){
            iUExt = mNumCtrlPoints.s + index.s;
            mControlPoints[iUExt * mNumCtrlPoints.t + index.t] = point;
        }
		
        if (vReplicate) {
            iVExt = mNumCtrlPoints.t + index.t;
            mControlPoints[index.s * mNumCtrlPoints.t + iVExt] = point;
        }
		
        if (uReplicate && vReplicate) {
            mControlPoints[iUExt * mNumCtrlPoints.t + iVExt] = point;
        }
    }
}

vec3 BSplinePatch::getControlPoint(const ci::ivec2& index) const
{
    if (index.s < mNumCtrlPoints.s && index.t < mNumCtrlPoints.t) {
        return mControlPoints[index.s * mNumCtrlPoints.t + index.t];
    }

    return vec3( std::numeric_limits<float>::max() );
}

void BSplinePatch::updateControlPoints(const std::vector<vec3>& ctrlPoints, const ci::ivec2& size)
{
	// NOT CURRENTLY SUPPORTING A RESIZE OF CONTROL POINT GRID!
	CI_ASSERT_MSG(size == mNumCtrlPoints, "The implementation does not currently support resizing the control point grid.");
	
	createControls(ctrlPoints);
}

void BSplinePatch::setKnot(const uint8_t dim, const uint32_t i, float knot)
{
	if (mBasis.size() > dim) {
		mBasis[dim].setKnot(i, static_cast<float>(knot));
	}
}

float BSplinePatch::getKnot(const uint8_t dim, const uint32_t i) const
{
    if (mBasis.size() > dim) {
		mBasis[dim].getKnot(i);
	}

    return std::numeric_limits<float>::max();
}

void BSplinePatch::get(float u, float v, vec3* pos, vec3* derU,
	vec3* derV, vec3* derUU, vec3* derUV, vec3* derVV) const
{
    int iu, iumin, iumax;
    if (derUU) {
        mBasis[0].compute(u, 0, iumin, iumax);
        mBasis[0].compute(u, 1, iumin, iumax);
        mBasis[0].compute(u, 2, iumin, iumax);
    }
    else if (derUV || derU) {
        mBasis[0].compute(u, 0, iumin, iumax);
        mBasis[0].compute(u, 1, iumin, iumax);
    }
    else {
        mBasis[0].compute(u, 0, iumin, iumax);
    }

    int iv, ivmin, ivmax;
    if (derVV) {
        mBasis[1].compute(v, 0, ivmin, ivmax);
        mBasis[1].compute(v, 1, ivmin, ivmax);
        mBasis[1].compute(v, 2, ivmin, ivmax);
    }
    else if (derUV || derV) {
        mBasis[1].compute(v, 0, ivmin, ivmax);
        mBasis[1].compute(v, 1, ivmin, ivmax);
    }
    else {
        mBasis[1].compute(v, 0, ivmin, ivmax);
    }

    float tmp;
    const uint32_t ctrlPtsRow = mNumCtrlPoints.t + mReplicate.t;
    if (pos) {
        *pos = vec3(0);
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD0(iu)*mBasis[1].getD0(iv);
                *pos += tmp * mControlPoints[iu * ctrlPtsRow + iv];
            }
        }
    }

    if (derU) {
        *derU = vec3(0);
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD1(iu)*mBasis[1].getD0(iv);
                *derU += tmp * mControlPoints[iu * ctrlPtsRow + iv];
            }
        }
    }

    if (derV) {
        *derV = vec3(0);
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD0(iu)*mBasis[1].getD1(iv);
                *derV += tmp * mControlPoints[iu * ctrlPtsRow + iv];
            }
        }
    }

    if (derUU) {
        *derUU = vec3(0);
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD2(iu)*mBasis[1].getD0(iv);
                *derUU += tmp * mControlPoints[iu * ctrlPtsRow + iv];
            }
        }
    }

    if (derUV) {
        *derUV = vec3(0);
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD1(iu)*mBasis[1].getD1(iv);
                *derUV += tmp * mControlPoints[iu * ctrlPtsRow + iv];
            }
        }
    }

    if (derVV) {
        *derVV = vec3(0);
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD0(iu)*mBasis[1].getD2(iv);
                *derVV += tmp * mControlPoints[iu * ctrlPtsRow + iv];
            }
        }
    }
}

vec3 BSplinePatch::P(float u, float v) const
{
    vec3 pos;
    get(u, v, &pos, 0, 0, 0, 0, 0);
    return pos;
}

vec3 BSplinePatch::PU(float u, float v) const
{
    vec3 derU;
    get(u, v, 0, &derU, 0, 0, 0, 0);
    return derU;
}

vec3 BSplinePatch::PV(float u, float v) const
{
    vec3 derV;
    get(u, v, 0, 0, &derV, 0, 0, 0);
    return derV;
}

vec3 BSplinePatch::PUU(float u, float v) const
{
    vec3 derUU;
    get(u, v, 0, 0, 0, &derUU, 0, 0);
    return derUU;
}

vec3 BSplinePatch::PUV(float u, float v) const
{
    vec3 derUV;
    get(u, v, 0, 0, 0, 0, &derUV, 0);
    return derUV;
}

vec3 BSplinePatch::PVV(float u, float v) const
{
    vec3 derVV;
    get(u, v, 0, 0, 0, 0, 0, &derVV);
    return derVV;
}

vec3 BSplinePatch::tangent(float u, float v) const
{
    return normalize( PU(u, v) );
}

vec3 BSplinePatch::bitangent(float u, float v) const
{
    vec3 tangent0 = PU(u, v);
    vec3 tangent1 = PV(u, v);
    normalize( tangent0 );
    vec3 normal = normalize( cross(tangent0, tangent1) );
    tangent1 = cross(normal, tangent0);
    return tangent1;
}

vec3 BSplinePatch::position(float u, float v) const
{
	return P(u, v);
}

vec3 BSplinePatch::normal(float u, float v) const
{
    vec3 tangent0 = PU(u, v);
    vec3 tangent1 = PV(u, v);
    normalize( tangent0 );  // Include this to be consistent with GetFrame.
    return normalize( cross(tangent0, tangent1) );
}

void BSplinePatch::getFrame(float u, float v, vec3& position, vec3& tangent0, vec3& tangent1, vec3& normal) const
{
    position = P(u, v);
    tangent0 = PU(u, v);
    tangent1 = PV(u, v);
    normalize( tangent0 );  // T0
    normalize( tangent1 );  // temporary T1 just to compute N
    normal = normalize( cross(tangent0, tangent1) );  // N
	
    // The normalized first derivatives are not necessarily orthogonal.
    // Recompute T1 so that {T0,T1,N} is an orthonormal set.
    tangent1 = cross(normal, tangent0);
}

void BSplinePatch::computePrincipalCurvatureInfo(float u, float v, float& curve0, float& curve1, vec3& direction0, vec3& direction1)
{
	//======================================================================
    // Tangents:          T0 = (x_u, y_u, z_u)
	//                    T1 = (x_v, y_v, z_v)
    // Normal:            N = cross(T0,T1) / length(cross(T0,T1))
    // Metric tensor:     G = +-                      -+
    //                        | dot(T0,T0)  dot(T0,T1) |
    //                        | dot(T1,T0)  dot(T1,T1) |
    //                        +-                      -+
    //
    // Curvature tensor:  B = +-                          -+
    //                        | -dot(N,T0_u)  -dot(N,T0_v) |
    //                        | -dot(N,T1_u)  -dot(N,T1_v) |
    //                        +-                          -+
    //
    // Principal curvatures k are the generalized eigenvalues of:
    //
    //     Bw = kGw
    //
    // If k is a curvature and w=(a,b) is the corresponding solution to
    // Bw = kGw, then the principal direction as a 3D vector is d = a*U+b*V.
    //
    // Let k1 and k2 be the principal curvatures. The mean curvature
    // is (k1+k2)/2 and the Gaussian curvature is k1*k2.
	//======================================================================
	
    // Compute derivatives.
    vec3 derU = PU(u,v);
    vec3 derV = PV(u,v);
    vec3 derUU = PUU(u,v);
    vec3 derUV = PUV(u,v);
    vec3 derVV = PVV(u,v);
	
    // Compute the metric tensor.
    mat2 metricTensor;
    metricTensor[0][0] = dot(derU, derU);
    metricTensor[0][1] = dot(derU, derV);
    metricTensor[1][0] = metricTensor[0][1];
    metricTensor[1][1] = dot(derV, derV);
	
    // Compute the curvature tensor.
    vec3 normal = normalize( cross(derU, derV) );
    mat2 curvatureTensor;
    curvatureTensor[0][0] = -dot(normal, derUU);
    curvatureTensor[0][1] = -dot(normal, derUV);
    curvatureTensor[1][0] = curvatureTensor[0][1];
    curvatureTensor[1][1] = -dot(normal, derVV);
	
    // Characteristic polynomial is 0 = det(B-kG) = c2*k^2+c1*k+c0.
    float c0 = glm::determinant( curvatureTensor );
    float c1 = 2.0f * curvatureTensor[0][1] * metricTensor[0][1] - curvatureTensor[0][0] * metricTensor[1][1] - curvatureTensor[1][1] * metricTensor[0][0];
    float c2 = glm::determinant( metricTensor );
	
    // Principal curvatures are roots of characteristic polynomial.
    float temp = math<float>::sqrt(math<float>::abs(c1*c1 - (4.0f * c0 * c2)));
    float mult = 0.5f / c2;
    curve0 = -mult * (c1+temp);
    curve1 = mult * (-c1+temp);
	
    // Principal directions are solutions to (B-kG)w = 0,
    // w1 = (b12-k1*g12,-(b11-k1*g11)) OR (b22-k1*g22,-(b12-k1*g12)).
    float a0 = curvatureTensor[0][1] - curve0*metricTensor[0][1];
    float a1 = curve0*metricTensor[0][0] - curvatureTensor[0][0];
    float length = math<float>::sqrt(a0*a0 + a1*a1);
    if (length >= ZERO_TOLERANCE) {
        direction0 = a0*derU + a1*derV;
    }
    else {
        a0 = curvatureTensor[1][1] - curve0*metricTensor[1][1];
        a1 = curve0*metricTensor[0][1] - curvatureTensor[0][1];
        length = math<float>::sqrt(a0*a0 + a1*a1);
        if (length >= ZERO_TOLERANCE) {
            direction0 = a0*derU + a1*derV;
        }
        else {
            // Umbilic (surface is locally sphere, any direction principal).
            direction0 = derU;
        }
    }
    normalize( direction0 );
	
    // Second tangent is cross product of first tangent and normal.
    direction1 = cross(direction0, normal);
}
