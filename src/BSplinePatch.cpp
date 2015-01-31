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
:	mUMin(0), mUMax(1), mVMin(0), mVMax(1), mRectangular(true)
{
	mLoop[0] = false;
	mLoop[1] = false;
	mReplicate[0] = 0;
	mReplicate[1] = 0;
	mNumCtrlPoints[0] = 0;
	mNumCtrlPoints[1] = 0;
}

//BSplinePatch::BSplinePatch( const BSplinePatch& bspline )
//{
//	this->mUMin = bspline.mUMin;
//	this->mUMax = bspline.mUMax;
//	this->mVMin = bspline.mVMin;
//	this->mVMax = bspline.mVMax;
//	this->mRectangular = bspline.mRectangular;
//	std::memmove(this->mLoop, bspline.mLoop, 2 * sizeof(bool));
//	std::memmove(this->mReplicate, bspline.mReplicate, 2 * sizeof(uint32_t));
//	std::memmove(this->mNumCtrlPoints, bspline.mNumCtrlPoints, 2 * sizeof(uint32_t));
//	std::memmove(this->mBasis, bspline.mBasis, 2 * sizeof(ci::BSplineBasis));

	// Cannot copy the boost::multi_array. Super cool!
	// so dumb...
//	std::vector<size_t> extents;
//	const size_t* shape = bspline.mControlPoints.shape();
//	extents.assign( shape, shape + bspline.mControlPoints.num_dimensions() );
//	this->mControlPoints.resize( extents );
//	this->mControlPoints.reshape( extents );
//	this->mControlPoints = bspline.mControlPoints;
	
	//this->mControlPoints = boost::multi_array<ci::vec3, 2>( bspline.mControlPoints );
//}

//BSplinePatch& BSplinePatch::operator=( const BSplinePatch& bspline )
//{
//	
//	return *this;
//}

BSplinePatch::~BSplinePatch()
{
}

BSplinePatch::BSplinePatch(const ControlPointLatice& ctrlPoints,
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, bool vOpen)
:	mUMin(0), mUMax(1), mVMin(0), mVMax(1), mRectangular(true)
{
	CI_ASSERT(mUMin < mUMax && mVMin < mVMax);
	
	this->create(ctrlPoints, uDegree, vDegree, uLoop, vLoop, uOpen, vOpen);
}

BSplinePatch::BSplinePatch(const ControlPointLatice& ctrlPoints,
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, float* vKnot)
:	mUMin(0), mUMax(1), mVMin(0), mVMax(1), mRectangular(true)
{
	CI_ASSERT(mUMin < mUMax && mVMin < mVMax);
	this->create(ctrlPoints, uDegree, vDegree, uLoop, vLoop, uOpen, vKnot);
}

BSplinePatch::BSplinePatch(const ControlPointLatice& ctrlPoints,
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, bool vOpen)
:	mUMin(0), mUMax(1), mVMin(0), mVMax(1), mRectangular(true)
{
	CI_ASSERT(mUMin < mUMax && mVMin < mVMax);
	this->create(ctrlPoints, uDegree, vDegree, uLoop, vLoop, uKnot, vOpen);
}

BSplinePatch::BSplinePatch(const ControlPointLatice& ctrlPoints,
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, float* vKnot)
:	mUMin(0), mUMax(1), mVMin(0), mVMax(1), mRectangular(true)
{
	CI_ASSERT(mUMin < mUMax && mVMin < mVMax);
	this->create(ctrlPoints, uDegree, vDegree, uLoop, vLoop, uKnot, vKnot);
}

void BSplinePatch::create(const ControlPointLatice& ctrlPoints,
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, bool vOpen)
{
	size_t numUCtrlPoints = ctrlPoints.shape()[0];
	size_t numVCtrlPoints = ctrlPoints.shape()[1];
	
	CI_ASSERT(numUCtrlPoints >= 2);
	CI_ASSERT(1 <= uDegree && uDegree <= numUCtrlPoints - 1);
	CI_ASSERT(numVCtrlPoints >= 2);
	CI_ASSERT(1 <= vDegree && vDegree <= numVCtrlPoints - 1);
	
	mLoop[0] = uLoop;
    mLoop[1] = vLoop;
	
    mNumCtrlPoints[0] = numUCtrlPoints;
    mNumCtrlPoints[1] = numVCtrlPoints;
    mReplicate[0] = (uLoop ? (uOpen ? 1 : uDegree) : 0);
    mReplicate[1] = (vLoop ? (vOpen ? 1 : vDegree) : 0);
    createControls(ctrlPoints);
	
    mBasis[0].create(mNumCtrlPoints[0] + mReplicate[0], uDegree, uOpen);
    mBasis[1].create(mNumCtrlPoints[1] + mReplicate[1], vDegree, vOpen);
}

void BSplinePatch::create(const ControlPointLatice& ctrlPoints,
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, float* vKnot)
{
	size_t numUCtrlPoints = ctrlPoints.shape()[0];
	size_t numVCtrlPoints = ctrlPoints.shape()[1];
	
	CI_ASSERT(numUCtrlPoints >= 2);
	CI_ASSERT(1 <= uDegree && uDegree <= numUCtrlPoints - 1);
	CI_ASSERT(numVCtrlPoints >= 2);
	CI_ASSERT(1 <= vDegree && vDegree <= numVCtrlPoints - 1);
	
    mLoop[0] = uLoop;
    mLoop[1] = vLoop;
	
    mNumCtrlPoints[0] = numUCtrlPoints;
    mNumCtrlPoints[1] = numVCtrlPoints;
    mReplicate[0] = (uLoop ? (uOpen ? 1 : uDegree) : 0);
    mReplicate[1] = (vLoop ? 1 : 0);
    createControls(ctrlPoints);
	
    mBasis[0].create(mNumCtrlPoints[0] + mReplicate[0], uDegree, uOpen);
    mBasis[1].create(mNumCtrlPoints[1] + mReplicate[1], vDegree, vKnot);
}

void BSplinePatch::create(const ControlPointLatice& ctrlPoints,
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, bool vOpen)
{
	size_t numUCtrlPoints = ctrlPoints.shape()[0];
	size_t numVCtrlPoints = ctrlPoints.shape()[1];
	
	CI_ASSERT(numUCtrlPoints >= 2);
	CI_ASSERT(1 <= uDegree && uDegree <= numUCtrlPoints - 1);
	CI_ASSERT(numVCtrlPoints >= 2);
	CI_ASSERT(1 <= vDegree && vDegree <= numVCtrlPoints - 1);
	
    mLoop[0] = uLoop;
    mLoop[1] = vLoop;
	
    mNumCtrlPoints[0] = numUCtrlPoints;
    mNumCtrlPoints[1] = numVCtrlPoints;
    mReplicate[0] = (uLoop ? 1 : 0);
    mReplicate[1] = (vLoop ? (vOpen ? 1 : vDegree) : 0);
    createControls(ctrlPoints);
	
    mBasis[0].create(mNumCtrlPoints[0] + mReplicate[0], uDegree, uKnot);
    mBasis[1].create(mNumCtrlPoints[1] + mReplicate[1], vDegree, vOpen);
}

void BSplinePatch::create(const ControlPointLatice& ctrlPoints,
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, float* vKnot)
{
	size_t numUCtrlPoints = ctrlPoints.shape()[0];
	size_t numVCtrlPoints = ctrlPoints.shape()[1];
	
	CI_ASSERT(numUCtrlPoints >= 2);
	CI_ASSERT(1 <= uDegree && uDegree <= numUCtrlPoints - 1);
	CI_ASSERT(numVCtrlPoints >= 2);
	CI_ASSERT(1 <= vDegree && vDegree <= numVCtrlPoints - 1);
	
    mLoop[0] = uLoop;
    mLoop[1] = vLoop;
	
    mNumCtrlPoints[0] = numUCtrlPoints;
    mNumCtrlPoints[1] = numVCtrlPoints;
    mReplicate[0] = (uLoop ? 1 : 0);
    mReplicate[1] = (vLoop ? 1 : 0);
    createControls(ctrlPoints);
	
    mBasis[0].create(mNumCtrlPoints[0] + mReplicate[0], uDegree, uKnot);
    mBasis[1].create(mNumCtrlPoints[1] + mReplicate[1], vDegree, vKnot);
}

void BSplinePatch::createControls(const ControlPointLatice& ctrlPoints)
{
    const uint32_t newNumUCtrlPoints = mNumCtrlPoints[0] + mReplicate[0];
    const uint32_t newNumVCtrlPoints = mNumCtrlPoints[1] + mReplicate[1];
	
//	const uint32_t uSize = ctrlPoints.shape()[0];
//	const uint32_t vSize = ctrlPoints.shape()[1];
//	if (uSize != mNumCtrlPoints[0] || vSize != mNumCtrlPoints[1]) {
//		
//	}
	mControlPoints.resize(boost::extents[newNumUCtrlPoints][newNumVCtrlPoints]);
	if (newNumUCtrlPoints == mNumCtrlPoints[0] && newNumVCtrlPoints == mNumCtrlPoints[1]) {
		mControlPoints = ControlPointLatice(ctrlPoints);
	}
	else {
		for (size_t u = 0; u < newNumUCtrlPoints; ++u) {
			int uOld = u % mNumCtrlPoints[0];
			for (size_t v = 0; v < newNumVCtrlPoints; ++v) {
				int vOld = v % mNumCtrlPoints[1];
				mControlPoints[u][v] = ctrlPoints[uOld][vOld];
			}
		}
	}
}

void BSplinePatch::setControlPoint(const uint32_t uIndex, const uint32_t vIndex, const vec3& point)
{
    if (uIndex < mNumCtrlPoints[0] && vIndex < mNumCtrlPoints[1]) {
        // Set the control point.
		mControlPoints[uIndex][vIndex] = point;

        // Set the replicated control point.
        bool doUReplicate = (uIndex < mReplicate[0]);
        bool doVReplicate = (vIndex < mReplicate[1]);
        uint32_t iUExt = 0, iVExt = 0;

        if (doUReplicate){
            iUExt = mNumCtrlPoints[0] + uIndex;
            mControlPoints[iUExt][vIndex] = point;
        }
		
        if (doVReplicate) {
            iVExt = mNumCtrlPoints[1] + vIndex;
            mControlPoints[uIndex][iVExt] = point;
        }
		
        if (doUReplicate && doVReplicate) {
            mControlPoints[iUExt][iVExt] = point;
        }
    }
}

vec3 BSplinePatch::getControlPoint(const uint32_t uIndex, const uint32_t vIndex) const
{
    if (uIndex < mNumCtrlPoints[0] && vIndex < mNumCtrlPoints[1]) {
        return mControlPoints[uIndex][vIndex];
    }

    return vec3( std::numeric_limits<float>::max() );
}

void BSplinePatch::setControlPointLatice(const ControlPointLatice& ctrlPoints)
{
	CI_ASSERT(ctrlPoints.shape()[0] == mNumCtrlPoints[0]);
	CI_ASSERT(ctrlPoints.shape()[1] == mNumCtrlPoints[1]);
	
	createControls(ctrlPoints);
}

ControlPointLatice BSplinePatch::getControlPointLatice() const
{
	return mControlPoints;
}

void BSplinePatch::setKnot(const uint32_t dim, const uint32_t i, float knot)
{
	if (dim == 0) {
		mBasis[0].setKnot(i, static_cast<float>(knot));
	}
	else if (dim == 1) {
		mBasis[1].setKnot(i, static_cast<float>(knot));
	}
}

float BSplinePatch::getKnot(const uint32_t dim, const uint32_t i) const
{
    if (dim == 0) {
		return mBasis[0].getKnot(i);
	}
	else if (dim == 1) {
		return mBasis[1].getKnot(i);
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
                *pos += tmp * mControlPoints[iu][iv];
            }
        }
    }

    if (derU) {
        *derU = vec3(0);
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD1(iu)*mBasis[1].getD0(iv);
                *derU += tmp * mControlPoints[iu][iv];
            }
        }
    }

    if (derV) {
        *derV = vec3(0);
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD0(iu)*mBasis[1].getD1(iv);
                *derV += tmp * mControlPoints[iu][iv];
            }
        }
    }

    if (derUU) {
        *derUU = vec3(0);
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD2(iu)*mBasis[1].getD0(iv);
                *derUU += tmp * mControlPoints[iu][iv];
            }
        }
    }

    if (derUV) {
        *derUV = vec3(0);
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD1(iu)*mBasis[1].getD1(iv);
                *derUV += tmp * mControlPoints[iu][iv];
            }
        }
    }

    if (derVV) {
        *derVV = vec3(0);
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD0(iu)*mBasis[1].getD2(iv);
                *derVV += tmp * mControlPoints[iu][iv];
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
