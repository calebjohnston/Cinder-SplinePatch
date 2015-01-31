/**
 * Math portions copyright:
 * Geometric Tools, LLC
 * Copyright (c) 1998-2010
 * Distributed under the Boost Software License, Version 1.0.
 * http://www.boost.org/LICENSE_1_0.txt
 * http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
 */

#include <limits>

#include "cinder/CinderAssert.h"

#include "BSplinePatch.h"

#define ZERO_TOLERANCE 1e-06

///////////////////////////////////////////////////////////////////////////
//
// TODO:	Must FIX a bug with the setControlPoints method
// TODO:	Test knot implementation
//
///////////////////////////////////////////////////////////////////////////

using namespace ci;

BSplinePatch::BSplinePatch()
:	mDomainMin(0), mDomainMax(1)
{
	mLoop[0] = false;
	mLoop[1] = false;
	mReplicate.s = 0;
	mReplicate.t = 0;
	mNumCtrlPoints.s = 0;
	mNumCtrlPoints.t = 0;
}

BSplinePatch::~BSplinePatch()
{
}

BSplinePatch::BSplinePatch(const std::vector<vec3>& ctrlPoints,
						   const uint32_t uSize, const uint32_t vSize,
						   const uint32_t uDegree, const uint32_t vDegree,
						   bool uLoop, bool vLoop, bool uOpen, bool vOpen)
:	mDomainMin(0), mDomainMax(1)
{
	CI_ASSERT(mDomainMin.s < mDomainMax.s && mDomainMin.t < mDomainMax.t);
	
	this->create(ctrlPoints, uSize, vSize, uDegree, vDegree, uLoop, vLoop, uOpen, vOpen);
}

BSplinePatch::BSplinePatch(const std::vector<vec3>& ctrlPoints,
						   const uint32_t uSize, const uint32_t vSize,
						   const uint32_t uDegree, const uint32_t vDegree,
						   bool uLoop, bool vLoop, bool uOpen, float* vKnot)
:	mDomainMin(0), mDomainMax(1)
{
	CI_ASSERT(mDomainMin.s < mDomainMax.s && mDomainMin.t < mDomainMax.t);
	this->create(ctrlPoints, uSize, vSize, uDegree, vDegree, uLoop, vLoop, uOpen, vKnot);
}

BSplinePatch::BSplinePatch(const std::vector<vec3>& ctrlPoints,
						   const uint32_t uSize, const uint32_t vSize,
						   const uint32_t uDegree, const uint32_t vDegree,
						   bool uLoop, bool vLoop, float* uKnot, bool vOpen)
:	mDomainMin(0), mDomainMax(1)
{
	CI_ASSERT(mDomainMin.s < mDomainMax.s && mDomainMin.t < mDomainMax.t);
	this->create(ctrlPoints, uSize, vSize, uDegree, vDegree, uLoop, vLoop, uKnot, vOpen);
}

BSplinePatch::BSplinePatch(const std::vector<vec3>& ctrlPoints,
						   const uint32_t uSize, const uint32_t vSize,
						   const uint32_t uDegree, const uint32_t vDegree,
						   bool uLoop, bool vLoop, float* uKnot, float* vKnot)
:	mDomainMin(0), mDomainMax(1)
{
	CI_ASSERT(mDomainMin.s < mDomainMax.s && mDomainMin.t < mDomainMax.t);
	this->create(ctrlPoints, uSize, vSize, uDegree, vDegree, uLoop, vLoop, uKnot, vKnot);
}

void BSplinePatch::create(const std::vector<vec3>& ctrlPoints,
						  const uint32_t uSize, const uint32_t vSize,
						  const uint32_t uDegree, const uint32_t vDegree,
						  bool uLoop, bool vLoop, bool uOpen, bool vOpen)
{
	size_t numUCtrlPoints = uSize;
	size_t numVCtrlPoints = vSize;
	
	CI_ASSERT(numUCtrlPoints >= 2);
	CI_ASSERT(1 <= uDegree && uDegree <= numUCtrlPoints - 1);
	CI_ASSERT(numVCtrlPoints >= 2);
	CI_ASSERT(1 <= vDegree && vDegree <= numVCtrlPoints - 1);
	
	mLoop[0] = uLoop;
    mLoop[1] = vLoop;
	
    mNumCtrlPoints.s = numUCtrlPoints;
    mNumCtrlPoints.t = numVCtrlPoints;
    mReplicate.s = (uLoop ? (uOpen ? 1 : uDegree) : 0);
    mReplicate.t = (vLoop ? (vOpen ? 1 : vDegree) : 0);
	
    mBasis[0].create(mNumCtrlPoints.s + mReplicate.s, uDegree, uOpen);
    mBasis[1].create(mNumCtrlPoints.t + mReplicate.t, vDegree, vOpen);
	
    createControls(ctrlPoints);
}

void BSplinePatch::create(const std::vector<vec3>& ctrlPoints,
						  const uint32_t uSize, const uint32_t vSize,
						  const uint32_t uDegree, const uint32_t vDegree,
						  bool uLoop, bool vLoop, bool uOpen, float* vKnot)
{
	size_t numUCtrlPoints = uSize;
	size_t numVCtrlPoints = vSize;
	
	CI_ASSERT(numUCtrlPoints >= 2);
	CI_ASSERT(1 <= uDegree && uDegree <= numUCtrlPoints - 1);
	CI_ASSERT(numVCtrlPoints >= 2);
	CI_ASSERT(1 <= vDegree && vDegree <= numVCtrlPoints - 1);
	
    mLoop[0] = uLoop;
    mLoop[1] = vLoop;
	
    mNumCtrlPoints.s = numUCtrlPoints;
    mNumCtrlPoints.t = numVCtrlPoints;
    mReplicate.s = (uLoop ? (uOpen ? 1 : uDegree) : 0);
    mReplicate.t = (vLoop ? 1 : 0);
	
    mBasis[0].create(mNumCtrlPoints.s + mReplicate.s, uDegree, uOpen);
    mBasis[1].create(mNumCtrlPoints.t + mReplicate.t, vDegree, vKnot);
	
    createControls(ctrlPoints);
}

void BSplinePatch::create(const std::vector<vec3>& ctrlPoints,
						  const uint32_t uSize, const uint32_t vSize,
						  const uint32_t uDegree, const uint32_t vDegree,
						  bool uLoop, bool vLoop, float* uKnot, bool vOpen)
{
	size_t numUCtrlPoints = uSize;
	size_t numVCtrlPoints = vSize;
	
	CI_ASSERT(numUCtrlPoints >= 2);
	CI_ASSERT(1 <= uDegree && uDegree <= numUCtrlPoints - 1);
	CI_ASSERT(numVCtrlPoints >= 2);
	CI_ASSERT(1 <= vDegree && vDegree <= numVCtrlPoints - 1);
	
    mLoop[0] = uLoop;
    mLoop[1] = vLoop;
	
    mNumCtrlPoints.s = numUCtrlPoints;
    mNumCtrlPoints.t = numVCtrlPoints;
    mReplicate.s = (uLoop ? 1 : 0);
    mReplicate.t = (vLoop ? (vOpen ? 1 : vDegree) : 0);
	
    mBasis[0].create(mNumCtrlPoints.s + mReplicate.s, uDegree, uKnot);
    mBasis[1].create(mNumCtrlPoints.t + mReplicate.t, vDegree, vOpen);
	
    createControls(ctrlPoints);
}

void BSplinePatch::create(const std::vector<vec3>& ctrlPoints,
						  const uint32_t uSize, const uint32_t vSize,
						  const uint32_t uDegree, const uint32_t vDegree,
						  bool uLoop, bool vLoop, float* uKnot, float* vKnot)
{
	size_t numUCtrlPoints = uSize;
	size_t numVCtrlPoints = vSize;
	
	CI_ASSERT(numUCtrlPoints >= 2);
	CI_ASSERT(1 <= uDegree && uDegree <= numUCtrlPoints - 1);
	CI_ASSERT(numVCtrlPoints >= 2);
	CI_ASSERT(1 <= vDegree && vDegree <= numVCtrlPoints - 1);
	
    mLoop[0] = uLoop;
    mLoop[1] = vLoop;
	
    mNumCtrlPoints.s = numUCtrlPoints;
    mNumCtrlPoints.t = numVCtrlPoints;
    mReplicate.s = (uLoop ? 1 : 0);
    mReplicate.t = (vLoop ? 1 : 0);
	
    mBasis[0].create(mNumCtrlPoints.s + mReplicate.s, uDegree, uKnot);
    mBasis[1].create(mNumCtrlPoints.t + mReplicate.t, vDegree, vKnot);
	
    createControls(ctrlPoints);
}

void BSplinePatch::createControls(const std::vector<vec3>& ctrlPoints)
{
    const uint32_t newNumUCtrlPoints = mNumCtrlPoints.s + mReplicate.s;
    const uint32_t newNumVCtrlPoints = mNumCtrlPoints.t + mReplicate.t;
	
//	const uint32_t uSize = ctrlPoints.shape()[0];
//	const uint32_t vSize = ctrlPoints.shape()[1];
//	if (uSize != mNumCtrlPoints.s || vSize != mNumCtrlPoints.t) {
//		NOT CURRENTLY SUPPORTING A RESIZE OF CONTROL POINT GRID
//	}
	mControlPoints.clear();
	mControlPoints.reserve( newNumUCtrlPoints * newNumVCtrlPoints );
	if (newNumUCtrlPoints == mNumCtrlPoints.s && newNumVCtrlPoints == mNumCtrlPoints.t) {
		mControlPoints = ctrlPoints;
	}
	else {
		for (size_t u = 0; u < newNumUCtrlPoints; ++u) {
			int uOld = u % mNumCtrlPoints.s;
			for (size_t v = 0; v < newNumVCtrlPoints; ++v) {
				int vOld = v % mNumCtrlPoints.t;
				mControlPoints[u * mNumCtrlPoints.t + v] = ctrlPoints[uOld * mNumCtrlPoints.t + vOld];
			}
		}
	}
}

void BSplinePatch::setControlPoint(const uint32_t uIndex, const uint32_t vIndex, const vec3& point)
{
    if (uIndex < mNumCtrlPoints.s && vIndex < mNumCtrlPoints.t) {
		mControlPoints[uIndex * mNumCtrlPoints.t + vIndex] = point;

        // Set the replicated control point.
        bool uReplicate = (uIndex < mReplicate.s);
        bool vReplicate = (vIndex < mReplicate.t);
        uint32_t iUExt = 0, iVExt = 0;

        if (uReplicate){
            iUExt = mNumCtrlPoints.s + uIndex;
            mControlPoints[iUExt * mNumCtrlPoints.t + vIndex] = point;
        }
		
        if (vReplicate) {
            iVExt = mNumCtrlPoints.t + vIndex;
            mControlPoints[uIndex * mNumCtrlPoints.t + iVExt] = point;
        }
		
        if (uReplicate && vReplicate) {
            mControlPoints[iUExt * mNumCtrlPoints.t + iVExt] = point;
        }
    }
}

vec3 BSplinePatch::getControlPoint(const uint32_t uIndex, const uint32_t vIndex) const
{
    if (uIndex < mNumCtrlPoints.s && vIndex < mNumCtrlPoints.t) {
        return mControlPoints[uIndex * mNumCtrlPoints.t + vIndex];
    }

    return vec3( std::numeric_limits<float>::max() );
}

void BSplinePatch::updateControlPoints(const std::vector<vec3>& ctrlPoints, const uint32_t uSize, const uint32_t vSize)
{
	CI_ASSERT(uSize == mNumCtrlPoints.s);
	CI_ASSERT(vSize == mNumCtrlPoints.t);
	
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

    if (pos) {
        *pos = vec3(0);
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD0(iu)*mBasis[1].getD0(iv);
                *pos += tmp * mControlPoints[iu * mNumCtrlPoints.t + iv];
            }
        }
    }

    if (derU) {
        *derU = vec3(0);
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD1(iu)*mBasis[1].getD0(iv);
                *derU += tmp * mControlPoints[iu * mNumCtrlPoints.t + iv];
            }
        }
    }

    if (derV) {
        *derV = vec3(0);
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD0(iu)*mBasis[1].getD1(iv);
                *derV += tmp * mControlPoints[iu * mNumCtrlPoints.t + iv];
            }
        }
    }

    if (derUU) {
        *derUU = vec3(0);
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD2(iu)*mBasis[1].getD0(iv);
                *derUU += tmp * mControlPoints[iu * mNumCtrlPoints.t + iv];
            }
        }
    }

    if (derUV) {
        *derUV = vec3(0);
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD1(iu)*mBasis[1].getD1(iv);
                *derUV += tmp * mControlPoints[iu * mNumCtrlPoints.t + iv];
            }
        }
    }

    if (derVV) {
        *derVV = vec3(0);
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD0(iu)*mBasis[1].getD2(iv);
                *derVV += tmp * mControlPoints[iu * mNumCtrlPoints.t + iv];
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

vec3 BSplinePatch::tangent0(float u, float v) const
{
    return normalize( PU(u, v) );
}

vec3 BSplinePatch::tangent1(float u, float v) const
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
    curvatureTensor[0][0] = -dot(normal, derUU);	// TODO: negate instead?
    curvatureTensor[0][1] = -dot(normal, derUV);	// TODO: negate instead?
    curvatureTensor[1][0] = curvatureTensor[0][1];
    curvatureTensor[1][1] = -dot(normal, derVV);	// TODO: negate instead?
	
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
