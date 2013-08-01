/**
 * Significant Portions Copyright:
 * Geometric Tools, LLC
 * Copyright (c) 1998-2010
 * Distributed under the Boost Software License, Version 1.0.
 * http://www.boost.org/LICENSE_1_0.txt
 * http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
 */

#include <cassert>

#include "cinder/Matrix22.h"
#include "cinder/Vector.h"

#include "ParametricSurface.h"

#define ZERO_TOLERANCE 1e-06

///////////////////////////////////////////////////////////////////////////
//
// TODO:	Refactor out ZERO_TOLERANCE definition
// TODO:	Consider refactoring role of ParametricSurface with patch surfaces
//
///////////////////////////////////////////////////////////////////////////

using namespace ci;
using namespace cg;

ParametricSurface::ParametricSurface(float umin, float umax, float vmin, float vmax, bool rectangular)
{
	assert(umin < umax && vmin < vmax);

    mUMin = umin;
    mUMax = umax;
    mVMin = vmin;
    mVMax = vmax;
    mRectangular = rectangular;
}

ParametricSurface::~ParametricSurface()
{
}

Vec3f ParametricSurface::tangent0(float u, float v) const
{
    return PU(u, v).normalized();
}

Vec3f ParametricSurface::tangent1(float u, float v) const
{
    Vec3f tangent0 = PU(u, v);
    Vec3f tangent1 = PV(u, v);
    tangent0.normalize();
    Vec3f normal = tangent0.cross(tangent1).normalized();
    tangent1 = normal.cross(tangent0);
    return tangent1;
}

Vec3f ParametricSurface::position(float u, float v) const
{
	return P(u, v);
}

Vec3f ParametricSurface::normal(float u, float v) const
{
    Vec3f tangent0 = PU(u, v);
    Vec3f tangent1 = PV(u, v);
    tangent0.normalize();  // Include this to be consistent with GetFrame.
    return tangent0.cross(tangent1).normalized();
}

void ParametricSurface::getFrame(float u, float v, Vec3f& position, Vec3f& tangent0, Vec3f& tangent1, Vec3f& normal) const
{
    position = P(u, v);
    tangent0 = PU(u, v);
    tangent1 = PV(u, v);
    tangent0.normalize();  // T0
    tangent1.normalize();  // temporary T1 just to compute N
    normal = tangent0.cross(tangent1).normalized();  // N

    // The normalized first derivatives are not necessarily orthogonal.
    // Recompute T1 so that {T0,T1,N} is an orthonormal set.
    tangent1 = normal.cross(tangent0);
}

void ParametricSurface::computePrincipalCurvatureInfo(float u, float v, float& curve0, float& curve1, Vec3f& direction0, Vec3f& direction1)
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
    Vec3f derU = PU(u,v);
    Vec3f derV = PV(u,v);
    Vec3f derUU = PUU(u,v);
    Vec3f derUV = PUV(u,v);
    Vec3f derVV = PVV(u,v);
	
    // Compute the metric tensor.
    Matrix22f metricTensor;
    metricTensor.at(0,0) = derU.dot(derU);
    metricTensor.at(0,1) = derU.dot(derV);
    metricTensor.at(1,0) = metricTensor.at(0,1);
    metricTensor.at(1,1) = derV.dot(derV);
	
    // Compute the curvature tensor.
    Vec3f normal = derU.cross(derV).normalized();
    Matrix22f curvatureTensor;
    curvatureTensor.at(0,0) = -normal.dot(derUU);
    curvatureTensor.at(0,1) = -normal.dot(derUV);
    curvatureTensor.at(1,0) = curvatureTensor.at(0,1);
    curvatureTensor.at(1,1) = -normal.dot(derVV);
	
    // Characteristic polynomial is 0 = det(B-kG) = c2*k^2+c1*k+c0.
    float c0 = curvatureTensor.determinant();
    float c1 = 2.0f *curvatureTensor.at(0,1) * metricTensor.at(0,1) - curvatureTensor.at(0,0)*metricTensor.at(1,1) - curvatureTensor.at(1,1)*metricTensor.at(0,0);
    float c2 = metricTensor.determinant();
	
    // Principal curvatures are roots of characteristic polynomial.
    float temp = math<float>::sqrt(math<float>::abs(c1*c1 - (4.0f * c0 * c2)));
    float mult = 0.5f / c2;
    curve0 = -mult * (c1+temp);
    curve1 = mult * (-c1+temp);
	
    // Principal directions are solutions to (B-kG)w = 0,
    // w1 = (b12-k1*g12,-(b11-k1*g11)) OR (b22-k1*g22,-(b12-k1*g12)).
    float a0 = curvatureTensor.at(0,1) - curve0*metricTensor.at(0,1);
    float a1 = curve0*metricTensor.at(0,0) - curvatureTensor.at(0,0);
    float length = math<float>::sqrt(a0*a0 + a1*a1);
    if (length >= ZERO_TOLERANCE) {
        direction0 = a0*derU + a1*derV;
    }
    else {
        a0 = curvatureTensor.at(1,1) - curve0*metricTensor.at(1,1);
        a1 = curve0*metricTensor.at(0,1) - curvatureTensor.at(0,1);
        length = math<float>::sqrt(a0*a0 + a1*a1);
        if (length >= ZERO_TOLERANCE) {
            direction0 = a0*derU + a1*derV;
        }
        else {
            // Umbilic (surface is locally sphere, any direction principal).
            direction0 = derU;
        }
    }
    direction0.normalize();
	
    // Second tangent is cross product of first tangent and normal.
    direction1 = direction0.cross(normal);
}
