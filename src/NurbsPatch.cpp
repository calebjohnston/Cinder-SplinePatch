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

#include "NurbsPatch.h"

using namespace ci;
using namespace cg;

///////////////////////////////////////////////////////////////////////////
//
// TODO:	Consider refactoring role of ParametricSurface
// TODO:	Must FIX a bug with the setControlPoints method
// TODO:	Test knot implementation
//
///////////////////////////////////////////////////////////////////////////

NurbsPatch::NurbsPatch() : ParametricSurface(0, 1, 0, 1, true)
{
	mLoop[0] = false;
	mLoop[1] = false;
	mReplicate[0] = 0;
	mReplicate[1] = 0;
	mNumCtrlPoints[0] = 0;
	mNumCtrlPoints[1] = 0;
}
	
NurbsPatch::~NurbsPatch()
{
}

NurbsPatch::NurbsPatch(const ControlPointLatice& ctrlPoints, const PointWeightLatice& ctrlWeights,
    const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, bool vOpen)
:	ParametricSurface(0, 1, 0, 1, true)
{
	this->create(ctrlPoints, ctrlWeights, uDegree, vDegree, uLoop, vLoop, uOpen, vOpen);
}

NurbsPatch::NurbsPatch(const ControlPointLatice& ctrlPoints, const PointWeightLatice& ctrlWeights,
    const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, float* vKnot)
:	ParametricSurface(0, 1, 0, 1, true)
{
	this->create(ctrlPoints, ctrlWeights, uDegree, vDegree, uLoop, vLoop, uOpen, vKnot);
}

NurbsPatch::NurbsPatch(const ControlPointLatice& ctrlPoints, const PointWeightLatice& ctrlWeights,
    const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, bool vOpen)
:	ParametricSurface(0, 1, 0, 1, true)
{
	this->create(ctrlPoints, ctrlWeights, uDegree, vDegree, uLoop, vLoop, uKnot, vOpen);
}

NurbsPatch::NurbsPatch(const ControlPointLatice& ctrlPoints, const PointWeightLatice& ctrlWeights,
    const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, float* vKnot)
:	ParametricSurface(0, 1, 0, 1, true)
{
	this->create(ctrlPoints, ctrlWeights, uDegree, vDegree, uLoop, vLoop, uKnot, vKnot);
}
	
void NurbsPatch::create(const ControlPointLatice& ctrlPoints, const PointWeightLatice& ctrlWeights,
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, bool vOpen)
{
	size_t numUCtrlPoints = ctrlPoints.shape()[0];
	size_t numVCtrlPoints = ctrlPoints.shape()[1];
	
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
    createControl(ctrlPoints, ctrlWeights);
	
    mBasis[0].create(mNumCtrlPoints[0] + mReplicate[0], uDegree, uOpen);
    mBasis[1].create(mNumCtrlPoints[1] + mReplicate[1], vDegree, vOpen);
}
	
void NurbsPatch::create(const ControlPointLatice& ctrlPoints, const PointWeightLatice& ctrlWeights,
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, bool uOpen, float* vKnot)
{
	size_t numUCtrlPoints = ctrlPoints.shape()[0];
	size_t numVCtrlPoints = ctrlPoints.shape()[1];
	
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
    createControl(ctrlPoints, ctrlWeights);
	
    mBasis[0].create(mNumCtrlPoints[0] + mReplicate[0], uDegree, uOpen);
    mBasis[1].create(mNumCtrlPoints[1] + mReplicate[1], vDegree, vKnot);
}

void NurbsPatch::create(const ControlPointLatice& ctrlPoints, const PointWeightLatice& ctrlWeights,
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, bool vOpen)
{
	size_t numUCtrlPoints = ctrlPoints.shape()[0];
	size_t numVCtrlPoints = ctrlPoints.shape()[1];
	
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
    createControl(ctrlPoints, ctrlWeights);
	
    mBasis[0].create(mNumCtrlPoints[0] + mReplicate[0], uDegree, uKnot);
    mBasis[1].create(mNumCtrlPoints[1] + mReplicate[1], vDegree, vOpen);
}

void NurbsPatch::create(const ControlPointLatice& ctrlPoints, const PointWeightLatice& ctrlWeights,
	const uint32_t uDegree, const uint32_t vDegree, bool uLoop, bool vLoop, float* uKnot, float* vKnot)
{
	size_t numUCtrlPoints = ctrlPoints.shape()[0];
	size_t numVCtrlPoints = ctrlPoints.shape()[1];
	
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
    createControl(ctrlPoints, ctrlWeights);
	
    mBasis[0].create(mNumCtrlPoints[0] + mReplicate[0], uDegree, uKnot);
    mBasis[1].create(mNumCtrlPoints[1] + mReplicate[1], vDegree, vKnot);
}

void NurbsPatch::createControl(const ControlPointLatice& ctrlPoints, const PointWeightLatice& ctrlWeights)
{
	const uint32_t newNumUCtrlPoints = mNumCtrlPoints[0] + mReplicate[0];
	const uint32_t newNumVCtrlPoints = mNumCtrlPoints[1] + mReplicate[1];

	mControlPoints.resize(boost::extents[newNumUCtrlPoints][newNumVCtrlPoints]);
	mControlPoints = ControlPointLatice(ctrlPoints);
	mControlWeights.resize(boost::extents[newNumUCtrlPoints][newNumVCtrlPoints]);
	mControlWeights = PointWeightLatice(ctrlWeights);
}

void NurbsPatch::setControlPoint(const uint32_t uIndex, const uint32_t vIndex, const Vec3f& ctrl)
{
    if (uIndex < mNumCtrlPoints[0] && vIndex < mNumCtrlPoints[1]) {
        // Set the control point.
        mControlPoints[uIndex][vIndex] = ctrl;

        // Set the replicated control point.
        bool doUReplicate = (uIndex < mReplicate[0]);
        bool doVReplicate = (vIndex < mReplicate[1]);
        int uExt = 0, vExt = 0;

        if (doUReplicate) {
            uExt = mNumCtrlPoints[0] + uIndex;
            mControlPoints[uExt][vIndex] = ctrl;
        }
		
        if (doVReplicate) {
            vExt = mNumCtrlPoints[1] + vIndex;
            mControlPoints[uIndex][vExt] = ctrl;
        }
		
        if (doUReplicate && doVReplicate) {
            mControlPoints[uExt][vExt] = ctrl;
        }
    }
}

Vec3f NurbsPatch::getControlPoint(const uint32_t uIndex, const uint32_t vIndex) const
{
    if (uIndex < mNumCtrlPoints[0] && vIndex < mNumCtrlPoints[1]) {
        return mControlPoints[uIndex][vIndex];
    }

    return Vec3f::max();
}

void NurbsPatch::setControlWeight(const uint32_t uIndex, const uint32_t vIndex, float weight)
{
    if (uIndex < mNumCtrlPoints[0] && vIndex < mNumCtrlPoints[1]) {
        // Set the control weight.
        mControlWeights[uIndex][vIndex] = weight;

        // Set the replicated control weight.
        bool doUReplicate = (uIndex < mReplicate[0] );
        bool doVReplicate = (vIndex < mReplicate[1]);
        int uExt = 0, vExt = 0;

        if (doUReplicate) {
            uExt = mNumCtrlPoints[0] + uIndex;
            mControlWeights[uExt][vIndex] = weight;
        }
        if (doVReplicate) {
            vExt = mNumCtrlPoints[1] + vIndex;
            mControlWeights[uIndex][vExt] = weight;
        }
		
        if (doUReplicate && doVReplicate) {
            mControlWeights[uExt][vExt] = weight;
        }
    }
}

float NurbsPatch::getControlWeight(const uint32_t uIndex, const uint32_t vIndex) const
{
    if (uIndex < mNumCtrlPoints[0] && vIndex < mNumCtrlPoints[1]) return mControlWeights[uIndex][vIndex];

    return std::numeric_limits<float>::max();
}


void NurbsPatch::setKnot(int dim, int i, float knot)
{
	if (dim == 0) {
		mBasis[0].setKnot(i, static_cast<float>(knot));
	}
	else if (dim == 1) {
		mBasis[1].setKnot(i, static_cast<float>(knot));
	}
}


float NurbsPatch::getKnot(int dim, int i) const
{
	if (dim == 0) {
		return mBasis[0].getKnot(i);
	}
	else if (dim == 1) {
		return mBasis[1].getKnot(i);
	}

    return std::numeric_limits<float>::max();
}


void NurbsPatch::get(float u, float v, Vec3f* pos, Vec3f* derU, 
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

    Vec3f X = Vec3f::zero();
    float w = 0.0f;
    for (iu = iumin; iu <= iumax; ++iu) {
        for (iv = ivmin; iv <= ivmax; ++iv) {
            tmp = mBasis[0].getD0(iu) * mBasis[1].getD0(iv) * mControlWeights[iu][iv];
            X += tmp * mControlPoints[iu][iv];
            w += tmp;
        }
    }
	
    float invW = 1.0f / w;
    Vec3f P = invW*X;
    if (pos) {
        *pos = P;
    }

    if (!derU && !derV && !derUU && !derUV && !derVV) {
        return;
    }

    float wDerU = 0.0f;
    float wDerV = 0.0f;
    Vec3f PDerU = Vec3f::zero();
    Vec3f PDerV = Vec3f::zero();
    if (derU || derUU || derUV) {
        Vec3f XDerU = Vec3f::zero();
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD1(iu) * mBasis[1].getD0(iv) * mControlWeights[iu][iv];
                XDerU += tmp * mControlPoints[iu][iv];
                wDerU += tmp;
            }
        }
        PDerU = invW*(XDerU - wDerU*P);
        if (derU) {
            *derU = PDerU;
        }
    }

    if (derV || derVV || derUV) {
        Vec3f XDerV = Vec3f::zero();
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD0(iu) * mBasis[1].getD1(iv) * mControlWeights[iu][iv];
                XDerV += tmp * mControlPoints[iu][iv];
                wDerV += tmp;
            }
        }
        PDerV = invW*(XDerV - wDerV*P);
        if (derV) {
            *derV = PDerV;
        }
    }

    if (!derUU && !derUV && !derVV) {
        return;
    }

    if (derUU) {
        Vec3f XDerUU = Vec3f::zero();
        float wDerUU = 0.0f;
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD2(iu) * mBasis[1].getD0(iv) * mControlWeights[iu][iv];
                XDerUU += tmp * mControlPoints[iu][iv];
                wDerUU += tmp;
            }
        }
        *derUU = invW*(XDerUU - 2.0f*wDerU*PDerU - wDerUU*P);
    }

    if (derUV) {
        Vec3f XDerUV = Vec3f::zero();
        float wDerUV = 0.0f;
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD1(iu) * mBasis[1].getD1(iv) * mControlWeights[iu][iv];
                XDerUV += tmp * mControlPoints[iu][iv];
                wDerUV += tmp;
            }
        }
        *derUV = invW*(XDerUV - wDerU*PDerV - wDerV*PDerU - wDerUV*P);
    }

    if (derVV) {
        Vec3f XDerVV = Vec3f::zero();
        float wDerVV = 0.0f;
        for (iu = iumin; iu <= iumax; ++iu) {
            for (iv = ivmin; iv <= ivmax; ++iv) {
                tmp = mBasis[0].getD0(iu) * mBasis[1].getD2(iv) * mControlWeights[iu][iv];
                XDerVV += tmp * mControlPoints[iu][iv];
                wDerVV += tmp;
            }
        }
        *derVV = invW * (XDerVV - 2.0f*wDerV*PDerV - wDerVV*P);
    }
}

Vec3f NurbsPatch::P(float u, float v) const
{
    Vec3f pos;
    get(u, v, &pos, 0, 0, 0, 0, 0);
    return pos;
}

Vec3f NurbsPatch::PU(float u, float v) const
{
    Vec3f derU;
    get(u, v, 0, &derU, 0, 0, 0, 0);
    return derU;
}

Vec3f NurbsPatch::PV(float u, float v) const
{
    Vec3f derV;
    get(u, v, 0, 0, &derV, 0, 0, 0);
    return derV;
}

Vec3f NurbsPatch::PUU(float u, float v) const
{
    Vec3f derUU;
    get(u, v, 0, 0, 0, &derUU, 0, 0);
    return derUU;
}

Vec3f NurbsPatch::PUV(float u, float v) const
{
    Vec3f derUV;
    get(u, v, 0, 0, 0, 0, &derUV, 0);
    return derUV;
}

Vec3f NurbsPatch::PVV(float u, float v) const
{
    Vec3f derVV;
    get(u, v, 0, 0, 0, 0, 0, &derVV);
    return derVV;
}

