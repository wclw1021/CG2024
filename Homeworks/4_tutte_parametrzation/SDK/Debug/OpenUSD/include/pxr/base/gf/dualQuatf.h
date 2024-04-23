//
// Copyright 2021 Pixar
//
// Licensed under the Apache License, Version 2.0 (the "Apache License")
// with the following modification; you may not use this file except in
// compliance with the Apache License and the following modification to it:
// Section 6. Trademarks. is deleted and replaced with:
//
// 6. Trademarks. This License does not grant permission to use the trade
//    names, trademarks, service marks, or product names of the Licensor
//    and its affiliates, except as required to comply with Section 4(c) of
//    the License and to reproduce the content of the NOTICE file.
//
// You may obtain a copy of the Apache License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the Apache License with the above modification is
// distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied. See the Apache License for the specific
// language governing permissions and limitations under the Apache License.
//
////////////////////////////////////////////////////////////////////////
// This file is generated by a script.  Do not edit directly.  Edit the
// dualQuat.template.h file to make changes.

#ifndef PXR_BASE_GF_DUALQUATF_H
#define PXR_BASE_GF_DUALQUATF_H

/// \file gf/dualQuatf.h
/// \ingroup group_gf_LinearAlgebra

#include "pxr/pxr.h"
#include "pxr/base/gf/api.h"
#include "pxr/base/gf/declare.h"
#include "pxr/base/gf/traits.h"

#include "pxr/base/gf/quatf.h"
#include "pxr/base/tf/hash.h"

#include <iosfwd>

PXR_NAMESPACE_OPEN_SCOPE

template <>
struct GfIsGfDualQuat<class GfDualQuatf> { static const bool value = true; };

/// Return the dot (inner) product of two dual quaternions.
float GfDot(const GfDualQuatf& dq1, const GfDualQuatf& dq2);


/// \class GfDualQuatf
/// \ingroup group_gf_LinearAlgebra
///
/// Basic type: a real part quaternion and a dual part quaternion.
///
/// This class represents a generalized dual quaternion that has a real part
/// and a dual part quaternions. Dual quaternions are used to represent a
/// combination of rotation and translation.
///
/// References:
///    https://www.cs.utah.edu/~ladislav/kavan06dual/kavan06dual.pdf
///    http://web.cs.iastate.edu/~cs577/handouts/dual-quaternion.pdf
///
class GfDualQuatf final
{
  public:
    typedef float ScalarType;

    /// The default constructor leaves the dual quaternion undefined.
    GfDualQuatf() {}

    /// Initialize the real part to \p realVal and the imaginary part
    /// to zero quaternion.
    ///
    /// Since quaternions typically must be normalized, reasonable values for
    /// \p realVal are -1, 0, or 1.  Other values are legal but are likely to
    /// be meaningless.
    ///
    explicit GfDualQuatf( float realVal ) : _real( realVal ), _dual( 0 ) {}

    /// Initialize the real part to \p real quaternion and the imaginary part
    /// to zero quaternion.
    ///
    explicit GfDualQuatf( const GfQuatf &real )
        : _real( real ), _dual( 0 ) {
    }

    /// This constructor initializes the real and dual parts.
    GfDualQuatf( const GfQuatf &real, const GfQuatf &dual )
        : _real( real ), _dual( dual ) {
    }

    /// This constructor initializes from a rotation and a translation components.
    GfDualQuatf( const GfQuatf &rotation, const GfVec3f &translation )
        : _real( rotation ) {
        SetTranslation( translation );
    }

    /// Construct from GfDualQuatd.
    GF_API
    explicit GfDualQuatf(const GfDualQuatd &other);
    /// Implicitly convert from GfDualQuath.
    GF_API
    GfDualQuatf(const GfDualQuath &other);

    /// Sets the real part of the dual quaternion.
    void SetReal(const GfQuatf &real) {
        _real  = real;
    }

    /// Sets the dual part of the dual quaternion.
    void SetDual(const GfQuatf &dual) {
        _dual  = dual;
    }

    /// Returns the real part of the dual quaternion.
    const GfQuatf &GetReal() const {
        return _real;
    }

    /// Returns the dual part of the dual quaternion.
    const GfQuatf &GetDual() const {
        return _dual;
    }

    /// Returns the zero dual quaternion, which has a real part of (0,0,0,0) and
    /// a dual part of (0,0,0,0).
    static GfDualQuatf GetZero() {
        return GfDualQuatf(GfQuatf::GetZero(), GfQuatf::GetZero());
    }

    /// Returns the identity dual quaternion, which has a real part of (1,0,0,0) and
    /// a dual part of (0,0,0,0).
    static GfDualQuatf GetIdentity() {
        return GfDualQuatf(GfQuatf::GetIdentity(), GfQuatf::GetZero());
    }

    /// Returns geometric length of this dual quaternion.
    GF_API
    std::pair<float, float> GetLength() const;

    /// Returns a normalized (unit-length) version of this dual quaternion.
    /// If the length of this dual quaternion is smaller than \p
    /// eps, this returns the identity dual quaternion.
    GF_API
    GfDualQuatf GetNormalized(float eps = GF_MIN_VECTOR_LENGTH) const;

    /// Normalizes this dual quaternion in place.
    /// Normalizes this dual quaternion in place to unit length, returning the
    /// length before normalization. If the length of this dual quaternion is
    /// smaller than \p eps, this sets the dual quaternion to identity.
    GF_API
    std::pair<float, float> Normalize(float eps = GF_MIN_VECTOR_LENGTH);

    /// Returns the conjugate of this dual quaternion.
    GF_API
    GfDualQuatf GetConjugate() const;

    /// Returns the inverse of this dual quaternion.
    GF_API
    GfDualQuatf GetInverse() const;

    /// Set the translation component of this dual quaternion.
    GF_API
    void SetTranslation( const GfVec3f &translation );

    /// Get the translation component of this dual quaternion.
    GF_API
    GfVec3f GetTranslation() const;

    /// Hash.
    friend inline size_t hash_value(const GfDualQuatf &dq) {
        return TfHash::Combine(dq.GetReal(), dq.GetDual());
    }

    /// Component-wise dual quaternion equality test. The real and dual parts
    /// must match exactly for dual quaternions to be considered equal.
    bool operator ==(const GfDualQuatf &dq) const {
        return (GetReal() == dq.GetReal() &&
                GetDual() == dq.GetDual());
    }

    /// Component-wise dual quaternion inequality test. The real and dual
    /// parts must match exactly for dual quaternions to be considered equal.
    bool operator !=(const GfDualQuatf &dq) const {
        return ! (*this == dq);
    }

    /// Component-wise unary sum operator.
    GfDualQuatf &operator +=(const GfDualQuatf &dq)  {
        _real += dq._real;
        _dual += dq._dual;
        return *this;
    }

    /// Component-wise unary difference operator.
    GfDualQuatf &operator -=(const GfDualQuatf &dq)  {
        _real -= dq._real;
        _dual -= dq._dual;
        return *this;
    }

    /// Post-multiplies dual quaternion \p dq into this dual quaternion.
    GF_API
    GfDualQuatf &operator *=(const GfDualQuatf &dq);

    /// Scales this dual quaternion by \p s.
    GfDualQuatf &operator *=(float s) {
        _real *= s;
        _dual *= s;
        return *this;
    }

    /// Scales this dual quaternion by 1 / \p s.
    GfDualQuatf &operator /=(float s) {
        return (*this) *= 1.0 / s;
    }

    /// Component-wise binary sum operator.
    friend GfDualQuatf operator +(const GfDualQuatf &dq1,
                                     const GfDualQuatf &dq2) {
        GfDualQuatf dqt = dq1;
        return dqt += dq2;
    }

    /// Component-wise binary difference operator.
    friend GfDualQuatf operator -(const GfDualQuatf &dq1,
                                     const GfDualQuatf &dq2) {
        GfDualQuatf dqt = dq1;
        return dqt -= dq2;
    }

    /// Returns the product of dual quaternions \p dq1 and \p dq2.
    friend GfDualQuatf operator *(const GfDualQuatf &dq1,
                                     const GfDualQuatf &dq2) {
        GfDualQuatf dqt  = dq1;
        return dqt *= dq2;
    }

    /// Returns the product of dual quaternion \p dq and scalar \p s.
    friend GfDualQuatf operator *(const GfDualQuatf &dq, float s) {
        GfDualQuatf dqt  = dq;
        return dqt *= s;
    }

    /// Returns the product of dual quaternion \p dq and scalar \p s.
    friend GfDualQuatf operator *(float s, const GfDualQuatf &dq) {
        GfDualQuatf dqt  = dq;
        return dqt *= s;
    }

    /// Returns the product of dual quaternion \p dq and scalar 1 / \p s.
    friend GfDualQuatf operator /(const GfDualQuatf &dq, float s) {
        GfDualQuatf dqt  = dq;
        return dqt /= s;
    }

    /// Transforms the row vector \e vec by the dual quaternion.
    GF_API
    GfVec3f Transform(const GfVec3f &vec) const;

  private:
    GfQuatf _real;   // for rotation
    GfQuatf _dual;   // for translation
};


/// Output a GfDualQuatf using the format ((rw, rx, ry, rz), (dw, dx, dy, dz)).
/// \ingroup group_gf_DebuggingOutput
GF_API std::ostream &operator<<(std::ostream &out, const GfDualQuatf &dq);


/// Returns the dot (inner) product of two dual quaternions.
inline float
GfDot(const GfDualQuatf& dq1, const GfDualQuatf& dq2) {
    return GfDot(dq1.GetReal(), dq2.GetReal()) + GfDot(dq1.GetDual(), dq2.GetDual());
}

PXR_NAMESPACE_CLOSE_SCOPE

#endif // PXR_BASE_GF_DUALQUATF_H