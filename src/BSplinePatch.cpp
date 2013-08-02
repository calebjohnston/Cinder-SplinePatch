/**
 * Math portions copyright:
 * Geometric Tools, LLC
 * Copyright (c) 1998-2010
 * Distributed under the Boost Software License, Version 1.0.
 * http://www.boost.org/LICENSE_1_0.txt
 * http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
 */

#include <cassert>
#include <limits>

#include "BSplinePatch.h"

///////////////////////////////////////////////////////////////////////////
//
// TODO:	Consider refactoring role of ParametricSurface
// TODO:	Must FIX a bug with the setControlPoints method
// TODO:	Test knot implementation
//
///////////////////////////////////////////////////////////////////////////

using namespace ci;
using namespace cg;

BSplinePatch::BSplinePatch() : ParametricSurface(0, 1, 0, 1, true)
{
	mLoop[0] = false;
	mLoop[1] = false;
	mReplicate[0] = 0;
	mReplicate[1] = 0;
	mNumCtrlPoints[0] = 0;
	mNumCtrlPoints[1] = 0;
}

BSplinePatch::~BSplinePatch()
{
}

BSplinePatch::BSplinePatch(const ControlPointLatice& ctrlPoints, const uint32_t uSize, const uint32_t vSize, 
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, bool vOpen)
:	ParametricSurface(0, 1, 0, 1, true)
{
	this->create(ctrlPoints, uSize, vSize, uDegree, vDegree, uLoop, vLoop, uOpen, vOpen);
}

BSplinePatch::BSplinePatch(const ControlPointLatice& ctrlPoints, const uint32_t uSize, const uint32_t vSize, 
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, float* vKnot)
:	ParametricSurface(0, 1, 0, 1, true)
{
	this->create(ctrlPoints, uSize, vSize, uDegree, vDegree, uLoop, vLoop, uOpen, vKnot);
}

BSplinePatch::BSplinePatch(const ControlPointLatice& ctrlPoints, const uint32_t uSize, const uint32_t vSize, 
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, bool vOpen)
:	ParametricSurface(0, 1, 0, 1, true)
{
	this->create(ctrlPoints, uSize, vSize, uDegree, vDegree, uLoop, vLoop, uKnot, vOpen);
}

BSplinePatch::BSplinePatch(const ControlPointLatice& ctrlPoints, const uint32_t uSize, const uint32_t vSize, 
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, float* vKnot)
:	ParametricSurface(0, 1, 0, 1, true)
{
	this->create(ctrlPoints, uSize, vSize, uDegree, vDegree, uLoop, vLoop, uKnot, vKnot);
}

void BSplinePatch::create(const ControlPointLatice& ctrlPoints, const uint32_t uSize, const uint32_t vSize, 
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, bool vOpen)
{
	
	size_t numUCtrlPoints = uSize;
	size_t numVCtrlPoints = vSize;
	
	assert(numUCtrlPoints >= 2);
	assert(1 <= uDegree && uDegree <= numUCtrlPoints - 1);
	assert(numVCtrlPoints >= 2);
	assert(1 <= vDegree && vDegree <= numVCtrlPoints - 1);
	
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

void BSplinePatch::create(const ControlPointLatice& ctrlPoints, const uint32_t uSize, const uint32_t vSize,
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, float* vKnot)
{
	size_t numUCtrlPoints = uSize;
	size_t numVCtrlPoints = vSize;
	
	assert(numUCtrlPoints >= 2);
	assert(1 <= uDegree && uDegree <= numUCtrlPoints - 1);
	assert(numVCtrlPoints >= 2);
	assert(1 <= vDegree && vDegree <= numVCtrlPoints - 1);
	
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

void BSplinePatch::create(const ControlPointLatice& ctrlPoints, const uint32_t uSize, const uint32_t vSize, 
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, bool vOpen)
{
	size_t numUCtrlPoints = uSize;
	size_t numVCtrlPoints = vSize;
	
	assert(numUCtrlPoints >= 2);
	assert(1 <= uDegree && uDegree <= numUCtrlPoints - 1);
	assert(numVCtrlPoints >= 2);
	assert(1 <= vDegree && vDegree <= numVCtrlPoints - 1);
	
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

void BSplinePatch::create(const ControlPointLatice& ctrlPoints, const uint32_t uSize, const uint32_t vSize, 
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, float* vKnot)
{
	size_t numUCtrlPoints = uSize;
	size_t numVCtrlPoints = vSize;
	
	assert(numUCtrlPoints >= 2);
	assert(1 <= uDegree && uDegree <= numUCtrlPoints - 1);
	assert(numVCtrlPoints >= 2);
	assert(1 <= vDegree && vDegree <= numVCtrlPoints - 1);
	
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
	mControlPoints.resize(newNumUCtrlPoints * newNumVCtrlPoints);
	if (newNumUCtrlPoints == mNumCtrlPoints[0] && newNumVCtrlPoints == mNumCtrlPoints[1]) {
		mControlPoints = ControlPointLatice(ctrlPoints);
	}
	else {
		for (size_t u = 0; u < newNumUCtrlPoints; ++u) {
			int uOld = u % mNumCtrlPoints[0];
			for (size_t v = 0; v < newNumVCtrlPoints; ++v) {
				int vOld = v % mNumCtrlPoints[1];
				mControlPoints[u + (newNumUCtrlPoints * v)] = ctrlPoints[uOld + (mNumCtrlPoints[0] * vOld)];
			}
		}
	}
}

void BSplinePatch::setControlPoint(const uint32_t uIndex, const uint32_t vIndex, const Vec3f& point)
{
    if (uIndex < mNumCtrlPoints[0] && vIndex < mNumCtrlPoints[1]) {
        // Set the control point.
		mControlPoints[uIndex + (mNumCtrlPoints[0] * vIndex)] = point;

        // Set the replicated control point.
        bool doUReplicate = (uIndex < mReplicate[0]);
        bool doVReplicate = (vIndex < mReplicate[1]);
        uint32_t iUExt = 0, iVExt = 0;

        if (doUReplicate){
            iUExt = mNumCtrlPoints[0] + uIndex;
            mControlPoints[iUExt + (mNumCtrlPoints[0] * vIndex)] = point;
        }
		
        if (doVReplicate) {
            iVExt = mNumCtrlPoints[1] + vIndex;
            mControlPoints[uIndex + (mNumCtrlPoints[0] * iVExt)] = point;
        }
		
        if (doUReplicate && doVReplicate) {
            mControlPoints[iUExt + (mNumCtrlPoints[0] * iVExt)] = point;
        }
    }
}

Vec3f BSplinePatch::getControlPoint(const uint32_t uIndex, const uint32_t vIndex) const
{
    if (uIndex < mNumCtrlPoints[0] && vIndex < mNumCtrlPoints[1]) {
        return mControlPoints[uIndex + (mNumCtrlPoints[0] * vIndex)];
    }

    return Vec3f::max();
}

void BSplinePatch::setControlPointLatice(const ControlPointLatice& ctrlPoints)
{
	/*
	// TODO: remove asserts, determine how to handle reallocation
	assert(ctrlPoints.shape()[0] == mNumCtrlPoints[0]);
	assert(ctrlPoints.shape()[1] == mNumCtrlPoints[1]);
	 */
	
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

void BSplinePatch::get(float u, float v, Vec3f* pos, Vec3f* derU,
	Vec3f* derV, Vec3f* derUU, Vec3f* derUV, Vec3f* derVV) const
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

	int idiff = iumax - iumin;
    if (pos) {
        *pos = Vec3f::zero();
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD0(iu)*mBasis[1].getD0(iv);
                //*pos += tmp * mControlPoints[iu][iv];
				*pos += tmp * mControlPoints[iu + (idiff * iv)];
            }
        }
    }

    if (derU) {
        *derU = Vec3f::zero();
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD1(iu)*mBasis[1].getD0(iv);
                //*derU += tmp * mControlPoints[iu][iv];
				*derU += tmp * mControlPoints[iu + (idiff * iv)];
            }
        }
    }

    if (derV) {
        *derV = Vec3f::zero();
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD0(iu)*mBasis[1].getD1(iv);
                //*derV += tmp * mControlPoints[iu][iv];
				*derV += tmp * mControlPoints[iu + (idiff * iv)];
            }
        }
    }

    if (derUU) {
        *derUU = Vec3f::zero();
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD2(iu)*mBasis[1].getD0(iv);
                //*derUU += tmp * mControlPoints[iu][iv];
				*derUU += tmp * mControlPoints[iu + (idiff * iv)];
            }
        }
    }

    if (derUV) {
        *derUV = Vec3f::zero();
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD1(iu)*mBasis[1].getD1(iv);
                //*derUV += tmp * mControlPoints[iu][iv];
				*derUV += tmp * mControlPoints[iu + (idiff * iv)];
            }
        }
    }

    if (derVV) {
        *derVV = Vec3f::zero();
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD0(iu)*mBasis[1].getD2(iv);
                //*derVV += tmp * mControlPoints[iu][iv];
				*derVV += tmp * mControlPoints[iu + (idiff * iv)];
            }
        }
    }
}

Vec3f BSplinePatch::P(float u, float v) const
{
    Vec3f pos;
    get(u, v, &pos, 0, 0, 0, 0, 0);
    return pos;
}

Vec3f BSplinePatch::PU(float u, float v) const
{
    Vec3f derU;
    get(u, v, 0, &derU, 0, 0, 0, 0);
    return derU;
}

Vec3f BSplinePatch::PV(float u, float v) const
{
    Vec3f derV;
    get(u, v, 0, 0, &derV, 0, 0, 0);
    return derV;
}

Vec3f BSplinePatch::PUU(float u, float v) const
{
    Vec3f derUU;
    get(u, v, 0, 0, 0, &derUU, 0, 0);
    return derUU;
}

Vec3f BSplinePatch::PUV(float u, float v) const
{
    Vec3f derUV;
    get(u, v, 0, 0, 0, 0, &derUV, 0);
    return derUV;
}

Vec3f BSplinePatch::PVV(float u, float v) const
{
    Vec3f derVV;
    get(u, v, 0, 0, 0, 0, 0, &derVV);
    return derVV;
}
