// Copyright (C) 1995-2007 Rosen Diankov, James Kuffner
// Computer Science Department
// Carnegie Mellon University, Robotics Institute
// 5000 Forbes Ave. Pittsburgh, PA  15213
// U.S.A.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
/*! --------------------------------------------------------------------
  \file   math.h
  \brief  Defines basic math and gemoetric primitives. dReal is defined in ODE. Any
    functions not in here like conversions between quaterions 
    and matrices are most likely defined in ODE.
 -------------------------------------------------------------------- */

#ifndef RAVE_MATH_H
#define RAVE_MATH_H

#include <math.h>

#ifndef PI
#define PI ((float)3.141592654)
#endif

#define rswap(x, y) *(int*)&(x) ^= *(int*)&(y) ^= *(int*)&(x) ^= *(int*)&(y);

#define g_fEpsilon 1e-8
#define distinctRoots	0			// roots r0 < r1 < r2
#define singleRoot		1			// root r0
#define floatRoot01		2			// roots r0 = r1 < r2
#define floatRoot12		4			// roots r0 < r1 = r2
#define tripleRoot		6			// roots r0 = r1 = r2

template <class T> inline T RAD_2_DEG(T radians)  { return (radians * (T)57.29577951); }

template <class T> class RaveTransform;
template <class T> class RaveTransformMatrix;

inline float* normalize3(float* pfout, const float* pf);
inline float* normalize4(float* pfout, const float* pf);
inline float* cross3(float* pfout, const float* pf1, const float* pf2);

// multiplies 3x3 matrices
inline float* mult3_s4(float* pfres, const float* pf1, const float* pf2);
inline double* mult3_s4(double* pfres, const double* pf1, const double* pf2);

inline float* inv3(const float* pf, float* pfres, float* pfdet, int stride);
inline float* inv4(const float* pf, float* pfres);

// class used for 3 and 4 dim vectors and quaternions
// It is better to use this for a 3 dim vector because it is 16byte aligned and SIMD instructions can be used
template <class T>
class RaveVector
{
public:
    T x, y, z, w;
    
    RaveVector() : x(0), y(0), z(0), w(0) {}
        
    RaveVector(T x, T y, T z) : x(x), y(y), z(z), w(0) {}
    RaveVector(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}
    template<class U> RaveVector(const RaveVector<U> &vec) : x((T)vec.x), y((T)vec.y), z((T)vec.z), w((T)vec.w) {}

    // note, it only copes 3 values!
    template<class U> RaveVector(const U* pf) { assert(pf != NULL); x = (T)pf[0]; y = (T)pf[1]; z = (T)pf[2]; w = 0; }
    
    T  operator[](int i) const       { return (&x)[i]; }
    T& operator[](int i)             { return (&x)[i]; }
    
    // casting operators
    operator T* () { return &x; }
    operator const T* () const { return (const T*)&x; }

    template <class U>
    RaveVector<T>& operator=(const RaveVector<U>& r) { x = (T)r.x; y = (T)r.y; z = (T)r.z; w = (T)r.w; return *this; }
    
    // SCALAR FUNCTIONS
    template <class U> inline T dot(const RaveVector<U> &v) const { return x*v.x + y*v.y + z*v.z + w*v.w; }
    inline void normalize() { normalize4(); }

    inline void normalize4() {
        T f = x*x+y*y+z*z+w*w;
        assert( f > 0 );
        f = 1.0f / sqrt(f);
        x *= f; y *= f; z *= f; w *= f;
    }
    inline void normalize3() {
        T f = x*x+y*y+z*z;
        assert( f > 0 );
        f = 1.0f / sqrt(f);
        x *= f; y *= f; z *= f;
    }

    inline T getnormsqr() const { return x*x + y*y + z*z + w*w;}

    inline T lengthsqr2() const { return x*x + y*y; }
    inline T lengthsqr3() const { return x*x + y*y + z*z; }
    inline T lengthsqr4() const { return x*x + y*y + z*z + w*w; }

    inline void Set3(const T* pvals) { x = pvals[0]; y = pvals[1]; z = pvals[2]; }
    inline void Set3(T val1, T val2, T val3) { x = val1; y = val2; z = val3; }
    inline void Set4(const T* pvals) { x = pvals[0]; y = pvals[1]; z = pvals[2]; w = pvals[3]; }
    inline void Set4(T val1, T val2, T val3, T val4) { x = val1; y = val2; z = val3; w = val4;}
    // 3 dim cross product, w is not touched
    /// this = this x v
    inline void Cross(const RaveVector<T> &v) { Cross(*this, v); }

    /// this = u x v
    inline void Cross(const RaveVector<T> &u, const RaveVector<T> &v) {
        RaveVector<T> ucrossv;
        ucrossv[0] = u[1] * v[2] - u[2] * v[1];
        ucrossv[1] = u[2] * v[0] - u[0] * v[2];
        ucrossv[2] = u[0] * v[1] - u[1] * v[0];
        *this = ucrossv;
    }

    inline RaveVector<T> operator-() const { RaveVector<T> v; v.x = -x; v.y = -y; v.z = -z; v.w = -w; return v; }
    template <class U> inline RaveVector<T> operator+(const RaveVector<U> &r) const { RaveVector<T> v; v.x = x+r.x; v.y = y+r.y; v.z = z+r.z; v.w = w+r.w; return v; }
    template <class U> inline RaveVector<T> operator-(const RaveVector<U> &r) const { RaveVector<T> v; v.x = x-r.x; v.y = y-r.y; v.z = z-r.z; v.w = w-r.w; return v; }
    template <class U> inline RaveVector<T> operator*(const RaveVector<U> &r) const { RaveVector<T> v; v.x = r.x*x; v.y = r.y*y; v.z = r.z*z; v.w = r.w*w; return v; }
    inline RaveVector<T> operator*(T k) const { RaveVector<T> v; v.x = k*x; v.y = k*y; v.z = k*z; v.w = k*w; return v; }

    template <class U> inline RaveVector<T>& operator += (const RaveVector<U>& r) { x += r.x; y += r.y; z += r.z; w += r.w; return *this; }
    template <class U> inline RaveVector<T>& operator -= (const RaveVector<U>& r) { x -= r.x; y -= r.y; z -= r.z; w -= r.w; return *this; }
    template <class U> inline RaveVector<T>& operator *= (const RaveVector<U>& r) { x *= r.x; y *= r.y; z *= r.z; w *= r.w; return *this; }
    
    inline RaveVector<T>& operator *= (const T k) { x *= k; y *= k; z *= k; w *= k; return *this; }
    inline RaveVector<T>& operator /= (const T _k) { T k=1/_k; x *= k; y *= k; z *= k; w *= k; return *this; }

    template <class U> friend RaveVector<U> operator* (float f, const RaveVector<U>& v);
    template <class U> friend RaveVector<U> operator* (double f, const RaveVector<U>& v);
    
    template <class U> friend std::ostream& operator<<(std::ostream& O, const RaveVector<U>& v);
    template <class U> friend std::istream& operator>>(std::istream& I, RaveVector<U>& v);
};

typedef RaveVector<float> Vector;

template <class T>
inline RaveVector<T> operator* (float f, const RaveVector<T>& left)
{
    RaveVector<T> v;
    v.x = (T)f * left.x;
    v.y = (T)f * left.y;
    v.z = (T)f * left.z;
    v.w = (T)f * left.w;
    return v;
}

template <class T>
inline RaveVector<T> operator* (double f, const RaveVector<T>& left)
{
    RaveVector<T> v;
    v.x = (T)f * left.x;
    v.y = (T)f * left.y;
    v.z = (T)f * left.z;
    v.w = (T)f * left.w;
    return v;
}

/// rigid transformations
template <class T>
class RaveTransform
{
public:
    RaveTransform() { rot.x = 1; }
    template <class U> RaveTransform(const RaveTransform<U>& t) {
        rot = t.rot;
        trans = t.trans;
    }
    template <class U> RaveTransform(const RaveVector<U>& rot, const RaveVector<U>& trans) : rot(rot), trans(trans) {}
    inline RaveTransform(const RaveTransformMatrix<T>& t);

    void identity() {
        rot.x = 1; rot.y = rot.z = rot.w = 0;
        trans.x = trans.y = trans.z = 0;
    }
    
    template <class U>
    inline void rotfromaxisangle(const RaveVector<U>& axis, U angle) {
        U sinang = (U)sinf(angle/2);
        rot.x = (U)cosf(angle/2);
        rot.y = axis.x*sinang;
        rot.z = axis.y*sinang;
        rot.w = axis.z*sinang;
    }

    // transform a 3 dim vector
    inline RaveVector<T> operator* (const RaveVector<T>& r) const {
        return trans + rotate(r);
    }

    inline RaveVector<T> rotate(const RaveVector<T>& r) const {
        T xx = 2 * rot.y * rot.y;
        T xy = 2 * rot.y * rot.z;
        T xz = 2 * rot.y * rot.w;
        T xw = 2 * rot.y * rot.x;
        T yy = 2 * rot.z * rot.z;
        T yz = 2 * rot.z * rot.w;
        T yw = 2 * rot.z * rot.x;
        T zz = 2 * rot.w * rot.w;
        T zw = 2 * rot.w * rot.x;

        RaveVector<T> v;
        v.x = (1-yy-zz) * r.x + (xy-zw) * r.y + (xz+yw)*r.z;
        v.y = (xy+zw) * r.x + (1-xx-zz) * r.y + (yz-xw)*r.z;
        v.z = (xz-yw) * r.x + (yz+xw) * r.y + (1-xx-yy)*r.z;
        return v;
    }

    /// t = this * r
    inline RaveTransform<T> operator* (const RaveTransform<T>& r) const {
        RaveTransform<T> t;
        t.trans = operator*(r.trans);
        t.rot.x = rot.x*r.rot.x - rot.y*r.rot.y - rot.z*r.rot.z - rot.w*r.rot.w;
        t.rot.y = rot.x*r.rot.y + rot.y*r.rot.x + rot.z*r.rot.w - rot.w*r.rot.z;
        t.rot.z = rot.x*r.rot.z + rot.z*r.rot.x + rot.w*r.rot.y - rot.y*r.rot.w;
        t.rot.w = rot.x*r.rot.w + rot.w*r.rot.x + rot.y*r.rot.z - rot.z*r.rot.y;

        return t;
    }

    inline RaveTransform<T> inverse() const {
        RaveTransform<T> inv;
        inv.rot.x = rot.x;
        inv.rot.y = -rot.y;
        inv.rot.z = -rot.z;
        inv.rot.w = -rot.w;
        
        inv.trans = -inv.rotate(trans);
        return inv;
    }

    template <class U> friend std::ostream& operator<<(std::ostream& O, const RaveTransform<U>& v);
    template <class U> friend std::istream& operator>>(std::istream& I, RaveTransform<U>& v);

    RaveVector<T> rot, trans; // rot is a quaternion=(cos(ang/2),axisx*sin(ang/2),axisy*sin(ang/2),axisz*sin(ang/2))
};

typedef RaveTransform<float> Transform;

template <class T>
class RaveTransformMatrix
{
public:
    inline RaveTransformMatrix() { identity(); m[3] = m[7] = m[11] = 0; }
    template <class U> RaveTransformMatrix(const RaveTransformMatrix<U>& t) {
        // don't memcpy!
        m[0] = t.m[0]; m[1] = t.m[1]; m[2] = t.m[2]; m[3] = t.m[3];
        m[4] = t.m[4]; m[5] = t.m[5]; m[6] = t.m[6]; m[7] = t.m[7];
        m[8] = t.m[8]; m[9] = t.m[9]; m[10] = t.m[10]; m[11] = t.m[11];
        trans = t.trans;
    }
    inline RaveTransformMatrix(const RaveTransform<T>& t);

    inline void identity() {
        m[0] = 1; m[1] = 0; m[2] = 0;
        m[4] = 0; m[5] = 1; m[6] = 0;
        m[8] = 0; m[9] = 0; m[10] = 1;
        trans.x = trans.y = trans.z = 0;
    }
    
    template <class U>
    inline void rotfromaxisangle(const RaveVector<U>& axis, U angle) {
        RaveVector<T> quat;
        U sinang = (U)sinf(angle/2);
        quat.x = (U)cosf(angle/2);
        quat.y = axis.x*sinang;
        quat.z = axis.y*sinang;
        quat.w = axis.z*sinang;
    }

    template <class U>
    inline void rotfromquat(const RaveVector<U>& quat) {
            // q = (s,vx,vy,vz)
        T qq1 = 2*quat[1]*quat[1];
        T qq2 = 2*quat[2]*quat[2];
        T qq3 = 2*quat[3]*quat[3];
        m[4*0+0] = 1 - qq2 - qq3;
        m[4*0+1] = 2*(quat[1]*quat[2] - quat[0]*quat[3]);
        m[4*0+2] = 2*(quat[1]*quat[3] + quat[0]*quat[2]);
        m[4*0+3] = 0;
        m[4*1+0] = 2*(quat[1]*quat[2] + quat[0]*quat[3]);
        m[4*1+1] = 1 - qq1 - qq3;
        m[4*1+2] = 2*(quat[2]*quat[3] - quat[0]*quat[1]);
        m[4*1+3] = 0;
        m[4*2+0] = 2*(quat[1]*quat[3] - quat[0]*quat[2]);
        m[4*2+1] = 2*(quat[2]*quat[3] + quat[0]*quat[1]);
        m[4*2+2] = 1 - qq1 - qq2;
        m[4*2+3] = 0;
    }
    
    inline void rotfrommat(T m_00, T m_01, T m_02, T m_10, T m_11, T m_12, T m_20, T m_21, T m_22) {
        m[0] = m_00; m[1] = m_01; m[2] = m_02; m[3] = 0;
        m[4] = m_10; m[5] = m_11; m[6] = m_12; m[7] = 0;
        m[8] = m_20; m[9] = m_21; m[10] = m_22; m[11] = 0;
    }

    inline T rot(int i, int j) const {
        assert( i >= 0 && i < 3 && j >= 0 && j < 3);
        return m[4*i+j];
    }
    inline T& rot(int i, int j) {
        assert( i >= 0 && i < 3 && j >= 0 && j < 3);
        return m[4*i+j];
    }
    
    template <class U>
    inline RaveVector<T> operator* (const RaveVector<U>& r) const {
        RaveVector<T> v;
        v[0] = r[0] * m[0] + r[1] * m[1] + r[2] * m[2] + trans.x;
        v[1] = r[0] * m[4] + r[1] * m[5] + r[2] * m[6] + trans.y;
        v[2] = r[0] * m[8] + r[1] * m[9] + r[2] * m[10] + trans.z;
        return v;
    }

    /// t = this * r
    inline RaveTransformMatrix operator* (const RaveTransformMatrix<T>& r) const {
        RaveTransformMatrix<T> t;
        mult3_s4(&t.m[0], &m[0], &r.m[0]);
        t.trans[0] = r.trans[0] * m[0] + r.trans[1] * m[1] + r.trans[2] * m[2] + trans.x;
        t.trans[1] = r.trans[0] * m[4] + r.trans[1] * m[5] + r.trans[2] * m[6] + trans.y;
        t.trans[2] = r.trans[0] * m[8] + r.trans[1] * m[9] + r.trans[2] * m[10] + trans.z;
        return t;
    }

    template <class U>
    inline RaveVector<U> rotate(const RaveVector<U>& r) const {
        RaveVector<U> v;
        v.x = r.x * m[0] + r.y * m[1] + r.z * m[2];
        v.y = r.x * m[4] + r.y * m[5] + r.z * m[6];
        v.z = r.x * m[8] + r.y * m[9] + r.z * m[10];
        return v;
    }
    inline RaveTransformMatrix<T> inverse() const {
        RaveTransformMatrix<T> inv;
        inv3(m, inv.m, NULL, 4);
        inv.trans = -inv.rotate(trans);
        return inv;
    }

    template <class U>
    inline void Extract(RaveVector<U>& right, RaveVector<U>& up, RaveVector<U>& dir, RaveVector<U>& pos) const {
        pos = trans;
        right.x = m[0]; up.x = m[1]; dir.x = m[2];
        right.y = m[4]; up.y = m[5]; dir.y = m[6];
        right.z = m[8]; up.z = m[9]; dir.z = m[10];
    }

    template <class U> friend std::ostream& operator<<(std::ostream& O, const RaveTransformMatrix<U>& v);
    template <class U> friend std::istream& operator>>(std::istream& I, RaveTransformMatrix<U>& v);

    T m[12];
    RaveVector<T> trans; ///< translation component
};
typedef RaveTransformMatrix<float> TransformMatrix;

template <class T>
RaveTransform<T>::RaveTransform(const RaveTransformMatrix<T>& t)
{
    trans = t.trans;
    float tr,s;
    tr = t.m[4*0+0] + t.m[4*1+1] + t.m[4*2+2];
    if (tr >= 0) {
        s = sqrtf (tr + 1);
        rot[0] = (float)(0.5) * s;
        s = (float)(0.5) / s;
        rot[1] = (t.m[4*2+1] - t.m[4*1+2]) * s;
        rot[2] = (t.m[4*0+2] - t.m[4*2+0]) * s;
        rot[3] = (t.m[4*1+0] - t.m[4*0+1]) * s;
    }
    else {
        // find the largest diagonal element and jump to the appropriate case
        if (t.m[4*1+1] > t.m[4*0+0]) {
            if (t.m[4*2+2] > t.m[4*1+1]) goto case_2;
            goto case_1;
        }
        if (t.m[4*2+2] > t.m[4*0+0]) goto case_2;
        goto case_0;
        
    case_0:
        s = sqrtf((t.m[4*0+0] - (t.m[4*1+1] + t.m[4*2+2])) + 1);
        rot[1] = (float)(0.5) * s;
        s = (float)(0.5) / s;
        rot[2] = (t.m[4*0+1] + t.m[4*1+0]) * s;
        rot[3] = (t.m[4*2+0] + t.m[4*0+2]) * s;
        rot[0] = (t.m[4*2+1] - t.m[4*1+2]) * s;
        return;
        
    case_1:
        s = sqrtf((t.m[4*1+1] - (t.m[4*2+2] + t.m[4*0+0])) + 1);
        rot[2] = (float)(0.5) * s;
        s = (float)(0.5) / s;
        rot[3] = (t.m[4*1+2] + t.m[4*2+1]) * s;
        rot[1] = (t.m[4*0+1] + t.m[4*1+0]) * s;
        rot[0] = (t.m[4*0+2] - t.m[4*2+0]) * s;
        return;
        
    case_2:
        s = sqrtf((t.m[4*2+2] - (t.m[4*0+0] + t.m[4*1+1])) + 1);
        rot[3] = (float)(0.5) * s;
        s = (float)(0.5) / s;
        rot[1] = (t.m[4*2+0] + t.m[4*0+2]) * s;
        rot[2] = (t.m[4*1+2] + t.m[4*2+1]) * s;
        rot[0] = (t.m[4*1+0] - t.m[4*0+1]) * s;
        return;
    }
}

template <class T>
RaveTransformMatrix<T>::RaveTransformMatrix(const RaveTransform<T>& t)
{
    rotfromquat(t.rot);
    trans = t.trans;

}

struct RAY
{
    RAY() {}
    RAY(const Vector& _pos, const Vector& _dir) : pos(_pos), dir(_dir) {}
    Vector pos, dir;
};

struct AABB
{
    AABB() {}
    AABB(const Vector& vpos, const Vector& vextents) : pos(vpos), extents(vextents) {}
    Vector pos, extents;
};

struct OBB
{
    Vector right, up, dir, pos, extents;
};

struct TRIANGLE
{
    TRIANGLE() {}
    TRIANGLE(const Vector& v1, const Vector& v2, const Vector& v3) : v1(v1), v2(v2), v3(v3) {}
    ~TRIANGLE() {}

    Vector v1, v2, v3;      //!< the vertices of the triangle

    const Vector& operator[](int i) const { return (&v1)[i]; }
    Vector&       operator[](int i)       { return (&v1)[i]; }

    /// assumes CCW ordering of vertices 
    inline Vector ComputeNormal() {
        Vector normal;
        cross3(normal, v2-v1, v3-v1);
        return normal;
    }
};


// computes (*pmat) * v
inline float* transcoord3(float* pfout, const TransformMatrix* pmat, const float* pf);

// uses only 3x3 upper submatrix
inline float* transnorm3(float* pfout, const TransformMatrix* pmat, const float* pf);

// Routines made for 3D graphics that deal with 3 or 4 dim algebra structures
// Functions with postfix 3 are for 3x3 operations, etc

// all fns return pfout on success or NULL on failure
// results and arguments can share pointers


// multiplies 4x4 matrices
inline float* mult4(float* pfres, const float* pf1, const float* pf2);
inline double* mult4(double* pfres, const double* pf1, const double* pf2);

// pf1^T * pf2
inline float* multtrans3(float* pfres, const float* pf1, const float* pf2);
inline double* multtrans3(double* pfres, const double* pf1, const double* pf2);
inline float* multtrans4(float* pfres, const float* pf1, const float* pf2);
inline double* multtrans4(double* pfres, const double* pf1, const double* pf2);

inline float* transpose3(const float* pf, float* pfres);
inline double* transpose3(const double* pf, double* pfres);
inline float* transpose4(const float* pf, float* pfres);
inline double* transpose4(const double* pf, double* pfres);

inline float dot2(const float* pf1, const float* pf2);
inline float dot3(const float* pf1, const float* pf2);
inline float dot4(const float* pf1, const float* pf2);

inline float lengthsqr2(const float* pf);
inline float lengthsqr3(const float* pf);
inline float lengthsqr4(const float* pf);

inline float* normalize2(float* pfout, const float* pf);
inline float* normalize3(float* pfout, const float* pf);
inline float* normalize4(float* pfout, const float* pf);

////
// More complex ops that deal with arbitrary matrices //
////

// extract eigen values and vectors from a 2x2 matrix and returns true if all values are real
// returned eigen vectors are normalized
inline bool eig2(const float* pfmat, float* peigs, float& fv1x, float& fv1y, float& fv2x, float& fv2y);

// Simple routines for linear algebra algorithms //
int CubicRoots (double c0, double c1, double c2, double *r0, double *r1, double *r2);
template <class T, class S> void Tridiagonal3 (S* mat, T* diag, T* subd);
bool QLAlgorithm3 (float* m_aafEntry, float* afDiag, float* afSubDiag);

void EigenSymmetric3(float* fCovariance, float* eval, float* fAxes);

void GetCovarBasisVectors(float fCovariance[3][3], Vector* vRight, Vector* vUp, Vector* vDir);

/// SVD of a 3x3 matrix A such that A = U*diag(D)*V'
/// where U is a 3x3 matrix, V is a 3x3 matrix, and D is a 3x1 vector
/// The row stride for all matrices is 9 bytes
void svd3(const float* A, float* U, float* D, float* V);

// first root returned is always >= second, roots are defined if the quadratic doesn't have real solutions
void QuadraticSolver(float* pfQuadratic, float* pfRoots);

int insideQuadrilateral(const Vector* p0,const Vector* p1, const Vector* p2,const Vector* p3);
int insideTriangle(const Vector* p0, const Vector* p1, const Vector* p2);

bool RayOBBTest(const RAY& r, const OBB& obb);
float DistVertexOBBSq(const Vector& v, const OBB& o);

template <class T> int Min(T* pts, int stride, int numPts); // returns the index, stride in units of T
template <class T> int Max(T* pts, int stride, int numPts); // returns the index

// multiplies a matrix by a scalar
template <class T> inline void mult(T* pf, T fa, int r);

// multiplies a r1xc1 by c1xc2 matrix into pfres, if badd is true adds the result to pfres
// does not handle cases where pfres is equal to pf1 or pf2, use multtox for those cases
template <class T, class S, class R>
inline T* mult(T* pf1, R* pf2, int r1, int c1, int c2, S* pfres, bool badd = false);

// pf1 is transposed before mult
// rows of pf2 must equal rows of pf1
// pfres will be c1xc2 matrix
template <class T, class S, class R>
inline T* multtrans(T* pf1, R* pf2, int r1, int c1, int c2, S* pfres, bool badd = false);

// pf2 is transposed before mult
// the columns of both matrices must be the same and equal to c1
// r2 is the number of rows in pf2
// pfres must be an r1xr2 matrix
template <class T, class S, class R>
inline T* multtrans_to2(T* pf1, R* pf2, int r1, int c1, int r2, S* pfres, bool badd = false);

// multiplies rxc matrix pf1 and cxc matrix pf2 and stores the result in pf1, 
// the function needs a temporary buffer the size of c doubles, if pftemp == NULL,
// the function will allocate the necessary memory, otherwise pftemp should be big
// enough to store all the entries
template <class T> inline T* multto1(T* pf1, T* pf2, int r1, int c1, T* pftemp = NULL);

// same as multto1 except stores the result in pf2, pf1 has to be an r2xr2 matrix
// pftemp must be of size r2 if not NULL
template <class T, class S> inline T* multto2(T* pf1, S* pf2, int r2, int c2, S* pftemp = NULL);

// add pf1 + pf2 and store in pf1
template <class T> inline void sub(T* pf1, T* pf2, int r);
template <class T> inline T normsqr(const T* pf1, int r);
template <class T> inline T lengthsqr(const T* pf1, const T* pf2, int length);
template <class T> inline T dot(T* pf1, T* pf2, int length);

template <class T> inline T sum(T* pf, int length);

// takes the inverse of the 3x3 matrix pf and stores it into pfres, returns true if matrix is invertible
template <class T> inline bool inv2(T* pf, T* pfres);

///////////////////////
// Function Definitions
///////////////////////
bool eig2(const float* pfmat, float* peigs, float& fv1x, float& fv1y, float& fv2x, float& fv2y)
{
	// x^2 + bx + c
	float a, b, c, d;
	b = -(pfmat[0] + pfmat[3]);
	c = pfmat[0] * pfmat[3] - pfmat[1] * pfmat[2];
	d = b * b - 4.0f * c + 1e-16f;

	if( d < 0 ) return false;
	if( d < 1e-16f ) {
		a = -0.5f * b;
		peigs[0] = a;	peigs[1] = a;
		fv1x = pfmat[1];		fv1y = a - pfmat[0];
		c = 1 / sqrtf(fv1x*fv1x + fv1y*fv1y);
		fv1x *= c;		fv1y *= c;
		fv2x = -fv1y;		fv2y = fv1x;
		return true;
	}
	
	// two roots
	d = sqrtf(d);
	a = -0.5f * (b + d);
	peigs[0] = a;
	fv1x = pfmat[1];		fv1y = a-pfmat[0];
	c = 1 / sqrtf(fv1x*fv1x + fv1y*fv1y);
	fv1x *= c;		fv1y *= c;

	a += d;
	peigs[1] = a;
	fv2x = pfmat[1];		fv2y = a-pfmat[0];
	c = 1 / sqrtf(fv2x*fv2x + fv2y*fv2y);
	fv2x *= c;		fv2y *= c;
	return true;
}

// returns the number of real roots, fills r1 and r2 with the answers
template <class T>
inline int solvequad(T a, T b, T c, T& r1, T& r2)
{
    T d = b * b - (T)4 * c * a + (T)1e-16;

	if( d < 0 ) return 0;

	if( d < (T)1e-16 ) {
		r1 = r2 = (T)-0.5 * b / a;
		return 1;
	}
	
	// two roots
	d = sqrt(d);
	r1 = (T)-0.5 * (b + d) / a;
    r2 = r1 + d/a;
    return 2;
}

//#ifndef TI_USING_IPP

#define MULT3(stride) { \
	pfres2[0*stride+0] = pf1[0*stride+0]*pf2[0*stride+0]+pf1[0*stride+1]*pf2[1*stride+0]+pf1[0*stride+2]*pf2[2*stride+0]; \
	pfres2[0*stride+1] = pf1[0*stride+0]*pf2[0*stride+1]+pf1[0*stride+1]*pf2[1*stride+1]+pf1[0*stride+2]*pf2[2*stride+1]; \
	pfres2[0*stride+2] = pf1[0*stride+0]*pf2[0*stride+2]+pf1[0*stride+1]*pf2[1*stride+2]+pf1[0*stride+2]*pf2[2*stride+2]; \
	pfres2[1*stride+0] = pf1[1*stride+0]*pf2[0*stride+0]+pf1[1*stride+1]*pf2[1*stride+0]+pf1[1*stride+2]*pf2[2*stride+0]; \
	pfres2[1*stride+1] = pf1[1*stride+0]*pf2[0*stride+1]+pf1[1*stride+1]*pf2[1*stride+1]+pf1[1*stride+2]*pf2[2*stride+1]; \
	pfres2[1*stride+2] = pf1[1*stride+0]*pf2[0*stride+2]+pf1[1*stride+1]*pf2[1*stride+2]+pf1[1*stride+2]*pf2[2*stride+2]; \
	pfres2[2*stride+0] = pf1[2*stride+0]*pf2[0*stride+0]+pf1[2*stride+1]*pf2[1*stride+0]+pf1[2*stride+2]*pf2[2*stride+0]; \
	pfres2[2*stride+1] = pf1[2*stride+0]*pf2[0*stride+1]+pf1[2*stride+1]*pf2[1*stride+1]+pf1[2*stride+2]*pf2[2*stride+1]; \
	pfres2[2*stride+2] = pf1[2*stride+0]*pf2[0*stride+2]+pf1[2*stride+1]*pf2[1*stride+2]+pf1[2*stride+2]*pf2[2*stride+2]; \
}

/// mult3 with a 3x3 matrix whose row stride is 16 bytes
template <class T> inline T* _mult3_s4(T* pfres, const T* pf1, const T* pf2)
{
	assert( pf1 != NULL && pf2 != NULL && pfres != NULL );

	T* pfres2;
	if( pfres == pf1 || pfres == pf2 ) pfres2 = (T*)alloca(12 * sizeof(T));
	else pfres2 = pfres;

    MULT3(4)

	if( pfres2 != pfres ) memcpy(pfres, pfres2, 12*sizeof(T));

	return pfres;
}

inline float* mult3_s4(float* pfres, const float* pf1, const float* pf2) { return _mult3_s4<float>(pfres, pf1, pf2); }
inline double* mult3_s4(double* pfres, const double* pf1, const double* pf2) { return _mult3_s4<double>(pfres, pf1, pf2); }

/// mult3 with a 3x3 matrix whose row stride is 12 bytes
template <class T> inline T* _mult3_s3(T* pfres, const T* pf1, const T* pf2)
{
	assert( pf1 != NULL && pf2 != NULL && pfres != NULL );

	T* pfres2;
	if( pfres == pf1 || pfres == pf2 ) pfres2 = (T*)alloca(9 * sizeof(T));
	else pfres2 = pfres;

    MULT3(3)

	if( pfres2 != pfres ) memcpy(pfres, pfres2, 9*sizeof(T));

	return pfres;
}

inline float* mult3_s3(float* pfres, const float* pf1, const float* pf2) { return _mult3_s3<float>(pfres, pf1, pf2); }
inline double* mult3_s3(double* pfres, const double* pf1, const double* pf2) { return _mult3_s3<double>(pfres, pf1, pf2); }

// mult4
template <class T> 
inline T* _mult4(T* pfres, const T* p1, const T* p2)
{
	assert( pfres != NULL && p1 != NULL && p2 != NULL );

	T* pfres2;
	if( pfres == p1 || pfres == p2 ) pfres2 = (T*)alloca(16 * sizeof(T));
	else pfres2 = pfres;

	pfres2[0*4+0] = p1[0*4+0]*p2[0*4+0] + p1[0*4+1]*p2[1*4+0] + p1[0*4+2]*p2[2*4+0] + p1[0*4+3]*p2[3*4+0];
	pfres2[0*4+1] = p1[0*4+0]*p2[0*4+1] + p1[0*4+1]*p2[1*4+1] + p1[0*4+2]*p2[2*4+1] + p1[0*4+3]*p2[3*4+1];
	pfres2[0*4+2] = p1[0*4+0]*p2[0*4+2] + p1[0*4+1]*p2[1*4+2] + p1[0*4+2]*p2[2*4+2] + p1[0*4+3]*p2[3*4+2];
	pfres2[0*4+3] = p1[0*4+0]*p2[0*4+3] + p1[0*4+1]*p2[1*4+3] + p1[0*4+2]*p2[2*4+3] + p1[0*4+3]*p2[3*4+3];

	pfres2[1*4+0] = p1[1*4+0]*p2[0*4+0] + p1[1*4+1]*p2[1*4+0] + p1[1*4+2]*p2[2*4+0] + p1[1*4+3]*p2[3*4+0];
	pfres2[1*4+1] = p1[1*4+0]*p2[0*4+1] + p1[1*4+1]*p2[1*4+1] + p1[1*4+2]*p2[2*4+1] + p1[1*4+3]*p2[3*4+1];
	pfres2[1*4+2] = p1[1*4+0]*p2[0*4+2] + p1[1*4+1]*p2[1*4+2] + p1[1*4+2]*p2[2*4+2] + p1[1*4+3]*p2[3*4+2];
	pfres2[1*4+3] = p1[1*4+0]*p2[0*4+3] + p1[1*4+1]*p2[1*4+3] + p1[1*4+2]*p2[2*4+3] + p1[1*4+3]*p2[3*4+3];

	pfres2[2*4+0] = p1[2*4+0]*p2[0*4+0] + p1[2*4+1]*p2[1*4+0] + p1[2*4+2]*p2[2*4+0] + p1[2*4+3]*p2[3*4+0];
	pfres2[2*4+1] = p1[2*4+0]*p2[0*4+1] + p1[2*4+1]*p2[1*4+1] + p1[2*4+2]*p2[2*4+1] + p1[2*4+3]*p2[3*4+1];
	pfres2[2*4+2] = p1[2*4+0]*p2[0*4+2] + p1[2*4+1]*p2[1*4+2] + p1[2*4+2]*p2[2*4+2] + p1[2*4+3]*p2[3*4+2];
	pfres2[2*4+3] = p1[2*4+0]*p2[0*4+3] + p1[2*4+1]*p2[1*4+3] + p1[2*4+2]*p2[2*4+3] + p1[2*4+3]*p2[3*4+3];

	pfres2[3*4+0] = p1[3*4+0]*p2[0*4+0] + p1[3*4+1]*p2[1*4+0] + p1[3*4+2]*p2[2*4+0] + p1[3*4+3]*p2[3*4+0];
	pfres2[3*4+1] = p1[3*4+0]*p2[0*4+1] + p1[3*4+1]*p2[1*4+1] + p1[3*4+2]*p2[2*4+1] + p1[3*4+3]*p2[3*4+1];
	pfres2[3*4+2] = p1[3*4+0]*p2[0*4+2] + p1[3*4+1]*p2[1*4+2] + p1[3*4+2]*p2[2*4+2] + p1[3*4+3]*p2[3*4+2];
	pfres2[3*4+3] = p1[3*4+0]*p2[0*4+3] + p1[3*4+1]*p2[1*4+3] + p1[3*4+2]*p2[2*4+3] + p1[3*4+3]*p2[3*4+3];

	if( pfres != pfres2 ) memcpy(pfres, pfres2, sizeof(T)*16);
	return pfres;
}

inline float* mult4(float* pfres, const float* pf1, const float* pf2) { return _mult4<float>(pfres, pf1, pf2); }
inline double* mult4(double* pfres, const double* pf1, const double* pf2) { return _mult4<double>(pfres, pf1, pf2); }

template <class T> 
inline T* _multtrans3(T* pfres, const T* pf1, const T* pf2)
{
	T* pfres2;
	if( pfres == pf1 ) pfres2 = (T*)alloca(9 * sizeof(T));
	else pfres2 = pfres;

	pfres2[0] = pf1[0]*pf2[0]+pf1[3]*pf2[3]+pf1[6]*pf2[6];
	pfres2[1] = pf1[0]*pf2[1]+pf1[3]*pf2[4]+pf1[6]*pf2[7];
	pfres2[2] = pf1[0]*pf2[2]+pf1[3]*pf2[5]+pf1[6]*pf2[8];

	pfres2[3] = pf1[1]*pf2[0]+pf1[4]*pf2[3]+pf1[7]*pf2[6];
	pfres2[4] = pf1[1]*pf2[1]+pf1[4]*pf2[4]+pf1[7]*pf2[7];
	pfres2[5] = pf1[1]*pf2[2]+pf1[4]*pf2[5]+pf1[7]*pf2[8];

	pfres2[6] = pf1[2]*pf2[0]+pf1[5]*pf2[3]+pf1[8]*pf2[6];
	pfres2[7] = pf1[2]*pf2[1]+pf1[5]*pf2[4]+pf1[8]*pf2[7];
	pfres2[8] = pf1[2]*pf2[2]+pf1[5]*pf2[5]+pf1[8]*pf2[8];

	if( pfres2 != pfres ) memcpy(pfres, pfres2, 9*sizeof(T));

	return pfres;
}

template <class T> 
inline T* _multtrans4(T* pfres, const T* pf1, const T* pf2)
{
	T* pfres2;
	if( pfres == pf1 ) pfres2 = (T*)alloca(16 * sizeof(T));
	else pfres2 = pfres;

	for(int i = 0; i < 4; ++i) {
		for(int j = 0; j < 4; ++j) {
			pfres[4*i+j] = pf1[i] * pf2[j] + pf1[i+4] * pf2[j+4] + pf1[i+8] * pf2[j+8] + pf1[i+12] * pf2[j+12];
		}
	}

	return pfres;
}

inline float* multtrans3(float* pfres, const float* pf1, const float* pf2) { return _multtrans3<float>(pfres, pf1, pf2); }
inline double* multtrans3(double* pfres, const double* pf1, const double* pf2) { return _multtrans3<double>(pfres, pf1, pf2); }
inline float* multtrans4(float* pfres, const float* pf1, const float* pf2) { return _multtrans4<float>(pfres, pf1, pf2); }
inline double* multtrans4(double* pfres, const double* pf1, const double* pf2) { return _multtrans4<double>(pfres, pf1, pf2); }


//generate a random quaternion
//inline Vector GetRandomQuat(void)
//{
//    Vector q;
//
//    while(1) {
//        q.x = -1 + 2*RANDOM_FLOAT();
//        q.y = -1 + 2*RANDOM_FLOAT();
//        q.z = -1 + 2*RANDOM_FLOAT();
//        q.w = -1 + 2*RANDOM_FLOAT();
//
//        float norm = lengthsqr4(q);
//        if(norm <= 1) {
//            q = q * (1 / sqrtf(norm));
//            break;
//        }
//    }
//    
//    return q;
//}

inline Vector AxisAngle2Quat(const Vector* rotaxis,const float angle)
{
    Vector out;

    out.x = cos(angle/2);
    out.y = rotaxis->x*sin(angle/2);
    out.z = rotaxis->y*sin(angle/2);
    out.w = rotaxis->z*sin(angle/2);

    return out;
}

inline Vector dQSlerp(const Vector* q1, const Vector* q2, float t)
{
	// quaternion to return
	Vector qm;
    Vector qa = *q1;
    Vector qb = *q2;
	// Calculate angle between them.
	float cosHalfTheta = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z;
	// if qa=qb or qa=-qb then theta = 0 and we can return qa
	if (fabs(cosHalfTheta) >= 1.0){
		qm.w = qa.w;qm.x = qa.x;qm.y = qa.y;qm.z = qa.z;
		return qm;
	}
	// Calculate temporary values.
	float halfTheta = acosf(cosHalfTheta);
	float sinHalfTheta = sqrtf((float)1.0 - cosHalfTheta*cosHalfTheta);
	// if theta = 180 degrees then result is not fully defined
	// we could rotate around any axis normal to qa or qb
	if (fabs(sinHalfTheta) < 0.001){ // fabs is floating point absolute
		qm.w = (qa.w * (float)0.5 + qb.w * (float)0.5);
		qm.x = (qa.x * (float)0.5 + qb.x * (float)0.5);
		qm.y = (qa.y * (float)0.5 + qb.y * (float)0.5);
		qm.z = (qa.z * (float)0.5 + qb.z * (float)0.5);
		return qm;
	}

	float ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
	float ratioB = sin(t * halfTheta) / sinHalfTheta; 
	//calculate Quaternion.
	qm.w = (qa.w * ratioA + qb.w * ratioB);
	qm.x = (qa.x * ratioA + qb.x * ratioB);
	qm.y = (qa.y * ratioA + qb.y * ratioB);
	qm.z = (qa.z * ratioA + qb.z * ratioB);
	return qm;
}

/// calculates the determinant of a 3x3 matrix whose row stride stride elements
template <class T> inline T matrixdet3(const T* pf, int stride)
{
    return pf[0*stride+2] * (pf[1*stride + 0] * pf[2*stride + 1] - pf[1*stride + 1] * pf[2*stride + 0]) +
        pf[1*stride+2] * (pf[0*stride + 1] * pf[2*stride + 0] - pf[0*stride + 0] * pf[2*stride + 1]) +
        pf[2*stride+2] * (pf[0*stride + 0] * pf[1*stride + 1] - pf[0*stride + 1] * pf[1*stride + 0]);
}

/// stride is in T
/// if pfdet is not NULL, fills it with the determinant of the source matrix
template <class T> inline T* _inv3(const T* pf, T* pfres, T* pfdet, int stride)
{
	T* pfres2;
	if( pfres == pf ) pfres2 = (T*)alloca(3 * stride * sizeof(T));
	else pfres2 = pfres;

	// inverse = C^t / det(pf) where C is the matrix of coefficients

	// calc C^t
	pfres2[0*stride + 0] = pf[1*stride + 1] * pf[2*stride + 2] - pf[1*stride + 2] * pf[2*stride + 1];
	pfres2[0*stride + 1] = pf[0*stride + 2] * pf[2*stride + 1] - pf[0*stride + 1] * pf[2*stride + 2];
	pfres2[0*stride + 2] = pf[0*stride + 1] * pf[1*stride + 2] - pf[0*stride + 2] * pf[1*stride + 1];
	pfres2[1*stride + 0] = pf[1*stride + 2] * pf[2*stride + 0] - pf[1*stride + 0] * pf[2*stride + 2];
	pfres2[1*stride + 1] = pf[0*stride + 0] * pf[2*stride + 2] - pf[0*stride + 2] * pf[2*stride + 0];
	pfres2[1*stride + 2] = pf[0*stride + 2] * pf[1*stride + 0] - pf[0*stride + 0] * pf[1*stride + 2];
	pfres2[2*stride + 0] = pf[1*stride + 0] * pf[2*stride + 1] - pf[1*stride + 1] * pf[2*stride + 0];
	pfres2[2*stride + 1] = pf[0*stride + 1] * pf[2*stride + 0] - pf[0*stride + 0] * pf[2*stride + 1];
	pfres2[2*stride + 2] = pf[0*stride + 0] * pf[1*stride + 1] - pf[0*stride + 1] * pf[1*stride + 0];

	T fdet = pf[0*stride + 2] * pfres2[2*stride + 0] + pf[1*stride + 2] * pfres2[2*stride + 1] +
		pf[2*stride + 2] * pfres2[2*stride + 2];
	
	if( pfdet != NULL )
		pfdet[0] = fdet;

    if( fabs(fdet) < 1e-12 ) {
        printf("inv3: det is small %f", fdet);
        return NULL;
    }

	fdet = 1 / fdet;
	//if( pfdet != NULL ) *pfdet = fdet;

	if( pfres != pf ) {
		pfres[0*stride+0] *= fdet;		pfres[0*stride+1] *= fdet;		pfres[0*stride+2] *= fdet;
		pfres[1*stride+0] *= fdet;		pfres[1*stride+1] *= fdet;		pfres[1*stride+2] *= fdet;
		pfres[2*stride+0] *= fdet;		pfres[2*stride+1] *= fdet;		pfres[2*stride+2] *= fdet;
		return pfres;
	}

	pfres[0*stride+0] = pfres2[0*stride+0] * fdet;
	pfres[0*stride+1] = pfres2[0*stride+1] * fdet;
	pfres[0*stride+2] = pfres2[0*stride+2] * fdet;
	pfres[1*stride+0] = pfres2[1*stride+0] * fdet;
	pfres[1*stride+1] = pfres2[1*stride+1] * fdet;
	pfres[1*stride+2] = pfres2[1*stride+2] * fdet;
	pfres[2*stride+0] = pfres2[2*stride+0] * fdet;
	pfres[2*stride+1] = pfres2[2*stride+1] * fdet;
	pfres[2*stride+2] = pfres2[2*stride+2] * fdet;
	return pfres;
}

inline float* inv3(const float* pf, float* pfres, float* pfdet, int stride) { return _inv3<float>(pf, pfres, pfdet, stride); }

// inverse if 92 mults and 39 adds
template <class T> inline T* _inv4(const T* pf, T* pfres)
{
	T* pfres2;
	if( pfres == pf ) pfres2 = (T*)alloca(16 * sizeof(T));
	else pfres2 = pfres;

	// inverse = C^t / det(pf) where C is the matrix of coefficients

	// calc C^t

	// determinants of all possibel 2x2 submatrices formed by last two rows
	T fd0, fd1, fd2;
	T f1, f2, f3;
	fd0 = pf[2*4 + 0] * pf[3*4 + 1] - pf[2*4 + 1] * pf[3*4 + 0];
	fd1 = pf[2*4 + 1] * pf[3*4 + 2] - pf[2*4 + 2] * pf[3*4 + 1];
	fd2 = pf[2*4 + 2] * pf[3*4 + 3] - pf[2*4 + 3] * pf[3*4 + 2];

	f1 = pf[2*4 + 1] * pf[3*4 + 3] - pf[2*4 + 3] * pf[3*4 + 1];
	f2 = pf[2*4 + 0] * pf[3*4 + 3] - pf[2*4 + 3] * pf[3*4 + 0];
	f3 = pf[2*4 + 0] * pf[3*4 + 2] - pf[2*4 + 2] * pf[3*4 + 0];

	pfres2[0*4 + 0] =   pf[1*4 + 1] * fd2 - pf[1*4 + 2] * f1 + pf[1*4 + 3] * fd1;
	pfres2[0*4 + 1] = -(pf[0*4 + 1] * fd2 - pf[0*4 + 2] * f1 + pf[0*4 + 3] * fd1);

	pfres2[1*4 + 0] = -(pf[1*4 + 0] * fd2 - pf[1*4 + 2] * f2 + pf[1*4 + 3] * f3);
	pfres2[1*4 + 1] =   pf[0*4 + 0] * fd2 - pf[0*4 + 2] * f2 + pf[0*4 + 3] * f3;

	pfres2[2*4 + 0] =   pf[1*4 + 0] * f1 -	pf[1*4 + 1] * f2 + pf[1*4 + 3] * fd0;
	pfres2[2*4 + 1] = -(pf[0*4 + 0] * f1 -	pf[0*4 + 1] * f2 + pf[0*4 + 3] * fd0);

	pfres2[3*4 + 0] = -(pf[1*4 + 0] * fd1 - pf[1*4 + 1] * f3 + pf[1*4 + 2] * fd0);
	pfres2[3*4 + 1] =   pf[0*4 + 0] * fd1 - pf[0*4 + 1] * f3 + pf[0*4 + 2] * fd0;

	// determinants of first 2 rows of 4x4 matrix
	fd0 = pf[0*4 + 0] * pf[1*4 + 1] - pf[0*4 + 1] * pf[1*4 + 0];
	fd1 = pf[0*4 + 1] * pf[1*4 + 2] - pf[0*4 + 2] * pf[1*4 + 1];
	fd2 = pf[0*4 + 2] * pf[1*4 + 3] - pf[0*4 + 3] * pf[1*4 + 2];

	f1 = pf[0*4 + 1] * pf[1*4 + 3] - pf[0*4 + 3] * pf[1*4 + 1];
	f2 = pf[0*4 + 0] * pf[1*4 + 3] - pf[0*4 + 3] * pf[1*4 + 0];
	f3 = pf[0*4 + 0] * pf[1*4 + 2] - pf[0*4 + 2] * pf[1*4 + 0];

	pfres2[0*4 + 2] =   pf[3*4 + 1] * fd2 - pf[3*4 + 2] * f1 + pf[3*4 + 3] * fd1;
	pfres2[0*4 + 3] = -(pf[2*4 + 1] * fd2 - pf[2*4 + 2] * f1 + pf[2*4 + 3] * fd1);

	pfres2[1*4 + 2] = -(pf[3*4 + 0] * fd2 - pf[3*4 + 2] * f2 + pf[3*4 + 3] * f3);
	pfres2[1*4 + 3] =   pf[2*4 + 0] * fd2 - pf[2*4 + 2] * f2 + pf[2*4 + 3] * f3;

	pfres2[2*4 + 2] =   pf[3*4 + 0] * f1 -	pf[3*4 + 1] * f2 + pf[3*4 + 3] * fd0;
	pfres2[2*4 + 3] = -(pf[2*4 + 0] * f1 -	pf[2*4 + 1] * f2 + pf[2*4 + 3] * fd0);

	pfres2[3*4 + 2] = -(pf[3*4 + 0] * fd1 - pf[3*4 + 1] * f3 + pf[3*4 + 2] * fd0);
	pfres2[3*4 + 3] =   pf[2*4 + 0] * fd1 - pf[2*4 + 1] * f3 + pf[2*4 + 2] * fd0;

	T fdet = pf[0*4 + 3] * pfres2[3*4 + 0] + pf[1*4 + 3] * pfres2[3*4 + 1] +
			pf[2*4 + 3] * pfres2[3*4 + 2] + pf[3*4 + 3] * pfres2[3*4 + 3];

	if( fabs(fdet) < 1e-9) return NULL;

	fdet = 1 / fdet;
	//if( pfdet != NULL ) *pfdet = fdet;

	if( pfres2 == pfres ) {
		mult(pfres, fdet, 16);
		return pfres;
	}

	int i = 0;
	while(i < 16) {
		pfres[i] = pfres2[i] * fdet;
		++i;
	}

	return pfres;
}

inline float* inv4(const float* pf, float* pfres) { return _inv4<float>(pf, pfres); }

template <class T> inline T* _transpose3(const T* pf, T* pfres)
{
	assert( pf != NULL && pfres != NULL );

	if( pf == pfres ) {
		rswap(pfres[1], pfres[3]);
		rswap(pfres[2], pfres[6]);
		rswap(pfres[5], pfres[7]);
		return pfres;
	}

	pfres[0] = pf[0];	pfres[1] = pf[3];	pfres[2] = pf[6];
	pfres[3] = pf[1];	pfres[4] = pf[4];	pfres[5] = pf[7];
	pfres[6] = pf[2];	pfres[7] = pf[5];	pfres[8] = pf[8];

	return pfres;
}

inline float* transpose3(const float* pf, float* pfres) { return _transpose3(pf, pfres); }
inline double* transpose3(const double* pf, double* pfres) { return _transpose3(pf, pfres); }

template <class T> inline T* _transpose4(const T* pf, T* pfres)
{
	assert( pf != NULL && pfres != NULL );

	if( pf == pfres ) {
		rswap(pfres[1], pfres[4]);
		rswap(pfres[2], pfres[8]);
		rswap(pfres[3], pfres[12]);
		rswap(pfres[6], pfres[9]);
		rswap(pfres[7], pfres[13]);
		rswap(pfres[11], pfres[15]);
		return pfres;
	}

	pfres[0] = pf[0];	pfres[1] = pf[4];	pfres[2] = pf[8];		pfres[3] = pf[12];
	pfres[4] = pf[1];	pfres[5] = pf[5];	pfres[6] = pf[9];		pfres[7] = pf[13];
	pfres[8] = pf[2];	pfres[9] = pf[6];	pfres[10] = pf[10];		pfres[11] = pf[14];
	pfres[12] = pf[3];	pfres[13] = pf[7];	pfres[14] = pf[11];		pfres[15] = pf[15];
	return pfres;
}

inline float* transpose4(const float* pf, float* pfres) { return _transpose4(pf, pfres); }
inline double* transpose4(const double* pf, double* pfres) { return _transpose4(pf, pfres); }

inline float dot2(const float* pf1, const float* pf2)
{
	assert( pf1 != NULL && pf2 != NULL );
	return pf1[0]*pf2[0] + pf1[1]*pf2[1];
}

inline float dot3(const float* pf1, const float* pf2)
{
	assert( pf1 != NULL && pf2 != NULL );
	return pf1[0]*pf2[0] + pf1[1]*pf2[1] + pf1[2]*pf2[2];
}

inline float dot4(const float* pf1, const float* pf2)
{
	assert( pf1 != NULL && pf2 != NULL );
	return pf1[0]*pf2[0] + pf1[1]*pf2[1] + pf1[2]*pf2[2] + pf1[3] * pf2[3];
}

inline float lengthsqr2(const float* pf)
{
	assert( pf != NULL );
	return pf[0] * pf[0] + pf[1] * pf[1];
}

inline float lengthsqr3(const float* pf)
{
	assert( pf != NULL );
	return pf[0] * pf[0] + pf[1] * pf[1] + pf[2] * pf[2];
}

inline float lengthsqr4(const float* pf)
{
	assert( pf != NULL );
	return pf[0] * pf[0] + pf[1] * pf[1] + pf[2] * pf[2] + pf[3] * pf[3];
}

inline float* normalize2(float* pfout, const float* pf)
{
	assert(pf != NULL);

	float f = pf[0]*pf[0] + pf[1]*pf[1];
	f = 1.0f / sqrtf(f);
	pfout[0] = pf[0] * f;
	pfout[1] = pf[1] * f;

	return pfout;
}

inline float* normalize3(float* pfout, const float* pf)
{
	assert(pf != NULL);

	float f = pf[0]*pf[0] + pf[1]*pf[1] + pf[2]*pf[2];

	f = 1.0f / sqrtf(f);
	pfout[0] = pf[0] * f;
	pfout[1] = pf[1] * f;
	pfout[2] = pf[2] * f;

	return pfout;
}

inline float* normalize4(float* pfout, const float* pf)
{
	assert(pf != NULL);

	float f = pf[0]*pf[0] + pf[1]*pf[1] + pf[2]*pf[2] + pf[3]*pf[3];

	f = 1.0f / sqrtf(f);
	pfout[0] = pf[0] * f;
	pfout[1] = pf[1] * f;
	pfout[2] = pf[2] * f;
	pfout[3] = pf[3] * f;

	return pfout;
}

inline float* cross3(float* pfout, const float* pf1, const float* pf2)
{
	assert( pfout != NULL && pf1 != NULL && pf2 != NULL );

    float temp[3];
	temp[0] = pf1[1] * pf2[2] - pf1[2] * pf2[1];
	temp[1] = pf1[2] * pf2[0] - pf1[0] * pf2[2];
	temp[2] = pf1[0] * pf2[1] - pf1[1] * pf2[0];

	pfout[0] = temp[0]; pfout[1] = temp[1]; pfout[2] = temp[2];
    return pfout;
}

inline float* transcoord3(float* pfout, const TransformMatrix* pmat, const float* pf)
{
	assert( pfout != NULL && pf != NULL && pmat != NULL );

    float dummy[3];
	float* pfreal = (pfout == pf) ? dummy : pfout;

	pfreal[0] = pf[0] * pmat->m[0] + pf[1] * pmat->m[1] + pf[2] * pmat->m[2] + pmat->trans.x;
    pfreal[1] = pf[0] * pmat->m[4] + pf[1] * pmat->m[5] + pf[2] * pmat->m[6] + pmat->trans.y;
    pfreal[2] = pf[0] * pmat->m[8] + pf[1] * pmat->m[9] + pf[2] * pmat->m[10] + pmat->trans.z;

    if( pfout ==pf ) {
        pfout[0] = pfreal[0];
        pfout[1] = pfreal[1];
        pfout[2] = pfreal[2];
    }

	return pfout;
}

inline float* transnorm3(float* pfout, const float* pfmat, const float* pf)
{
	assert( pfout != NULL && pf != NULL && pfmat != NULL );

    float dummy[3];
	float* pfreal = (pfout == pf) ? dummy : pfout;

	pfreal[0] = pf[0] * pfmat[0] + pf[1] * pfmat[1] + pf[2] * pfmat[2];
    pfreal[1] = pf[0] * pfmat[3] + pf[1] * pfmat[4] + pf[2] * pfmat[5];
    pfreal[2] = pf[0] * pfmat[6] + pf[1] * pfmat[7] + pf[2] * pfmat[8];

    if( pfout ==pf ) {
        pfout[0] = pfreal[0];
        pfout[1] = pfreal[1];
        pfout[2] = pfreal[2];
    }

	return pfout;
}

inline float* transnorm3(float* pfout, const TransformMatrix* pmat, const float* pf)
{
	assert( pfout != NULL && pf != NULL && pmat != NULL );

	float dummy[3];
    float* pfreal = (pfout == pf) ? dummy : pfout;

	pfreal[0] = pf[0] * pmat->m[0] + pf[1] * pmat->m[1] + pf[2] * pmat->m[2];
    pfreal[1] = pf[0] * pmat->m[4] + pf[1] * pmat->m[5] + pf[2] * pmat->m[6];
    pfreal[2] = pf[0] * pmat->m[8] + pf[1] * pmat->m[9] + pf[2] * pmat->m[10];

	if( pfreal != pfout ) {
        pfout[0] = pfreal[0];
        pfout[1] = pfreal[1];
        pfout[2] = pfreal[2];
    }

	return pfout;
}


// if the two triangles collide, returns true and fills contactpos with the intersection point
// assuming triangles are declared counter-clockwise!!
// contactnorm is the normal of the first triangle
bool TriTriCollision(const Vector& u1, const Vector& u2, const Vector& u3,
                   const Vector& v1, const Vector& v2, const Vector& v3,
                   Vector& contactpos, Vector& contactnorm);

template <class T> inline void mult(T* pf, T fa, int r)
{
	assert( pf != NULL );

	while(r > 0) {
		--r;
		pf[r] *= fa;
	}
}

template <class T, class S, class R>
inline T* mult(T* pf1, R* pf2, int r1, int c1, int c2, S* pfres, bool badd)
{
	assert( pf1 != NULL && pf2 != NULL && pfres != NULL);
	int j, k;

	if( !badd ) memset(pfres, 0, sizeof(S) * r1 * c2);

	while(r1 > 0) {
		--r1;

		j = 0;
		while(j < c2) {
			k = 0;
			while(k < c1) {
				pfres[j] += pf1[k] * pf2[k*c2 + j];
				++k;
			}
			++j;
		}

		pf1 += c1;
		pfres += c2;
	}

	return pfres;
}

template <class T, class S, class R>
inline T* multtrans(T* pf1, R* pf2, int r1, int c1, int c2, S* pfres, bool badd)
{
	assert( pf1 != NULL && pf2 != NULL && pfres != NULL);
	int i, j, k;

	if( !badd ) memset(pfres, 0, sizeof(S) * c1 * c2);

	i = 0;
	while(i < c1) {

		j = 0;
		while(j < c2) {

			k = 0;
			while(k < r1) {
				pfres[j] += pf1[k*c1] * pf2[k*c2 + j];
				++k;
			}
			++j;
		}

		pfres += c2;
		++pf1;

		++i;
	}

	return pfres;
}

template <class T, class S, class R>
inline T* multtrans_to2(T* pf1, R* pf2, int r1, int c1, int r2, S* pfres, bool badd)
{
	assert( pf1 != NULL && pf2 != NULL && pfres != NULL);
	int j, k;

	if( !badd ) memset(pfres, 0, sizeof(S) * r1 * r2);

	while(r1 > 0) {
		--r1;

		j = 0;
		while(j < r2) {
			k = 0;
			while(k < c1) {
				pfres[j] += pf1[k] * pf2[j*c1 + k];
				++k;
			}
			++j;
		}

		pf1 += c1;
        pfres += r2;
	}

	return pfres;
}

template <class T> inline T* multto1(T* pf1, T* pf2, int r, int c, T* pftemp)
{
	assert( pf1 != NULL && pf2 != NULL );

	int j, k;
	bool bdel = false;
	
	if( pftemp == NULL ) {
		pftemp = new T[c];
		bdel = true;
	}

	while(r > 0) {
		--r;

		j = 0;
		while(j < c) {
			
			pftemp[j] = 0.0;

			k = 0;
			while(k < c) {
				pftemp[j] += pf1[k] * pf2[k*c + j];
				++k;
			}
			++j;
		}

		memcpy(pf1, pftemp, c * sizeof(T));
		pf1 += c;
	}

	if( bdel ) delete[] pftemp;

	return pf1;
}

template <class T, class S> inline T* multto2(T* pf1, S* pf2, int r2, int c2, S* pftemp)
{
	assert( pf1 != NULL && pf2 != NULL );

	int i, j, k;
	bool bdel = false;
	
	if( pftemp == NULL ) {
		pftemp = new S[r2];
		bdel = true;
	}

	// do columns first
	j = 0;
	while(j < c2) {
		i = 0;
		while(i < r2) {
			
			pftemp[i] = 0.0;

			k = 0;
			while(k < r2) {
				pftemp[i] += pf1[i*r2 + k] * pf2[k*c2 + j];
				++k;
			}
			++i;
		}

		i = 0;
		while(i < r2) {
			*(pf2+i*c2+j) = pftemp[i];
			++i;
		}

		++j;
	}

	if( bdel ) delete[] pftemp;

	return pf1;
}

template <class T> inline void add(T* pf1, T* pf2, int r)
{
	assert( pf1 != NULL && pf2 != NULL);

	while(r > 0) {
		--r;
		pf1[r] += pf2[r];
	}
}

template <class T> inline void sub(T* pf1, T* pf2, int r)
{
	assert( pf1 != NULL && pf2 != NULL);

	while(r > 0) {
		--r;
		pf1[r] -= pf2[r];
	}
}

template <class T> inline T normsqr(const T* pf1, int r)
{
	assert( pf1 != NULL );

	T d = 0.0;
	while(r > 0) {
		--r;
		d += pf1[r] * pf1[r];
	}

	return d;
}

template <class T> inline T lengthsqr(const T* pf1, const T* pf2, int length)
{
	T d = 0;
	while(length > 0) {
		--length;
        T t = pf1[length] - pf2[length];
		d += t * t;
	}

	return d;
}

template <class T> inline T dot(T* pf1, T* pf2, int length)
{
	T d = 0;
	while(length > 0) {
		--length;
		d += pf1[length] * pf2[length];
	}

	return d;
}

template <class T> inline T sum(T* pf, int length)
{
	T d = 0;
	while(length > 0) {
		--length;
		d += pf[length];
	}

	return d;
}

template <class T> inline bool inv2(T* pf, T* pfres)
{
	T fdet = pf[0] * pf[3] - pf[1] * pf[2];

	if( fabs(fdet) < 1e-16 ) return false;

	fdet = 1 / fdet;
	//if( pfdet != NULL ) *pfdet = fdet;

	if( pfres != pf ) {
		pfres[0] = fdet * pf[3];		pfres[1] = -fdet * pf[1];
		pfres[2] = -fdet * pf[2];		pfres[3] = fdet * pf[0];
		return true;
	}

	float ftemp = pf[0];
	pfres[0] = pf[3] * fdet;
	pfres[1] *= -fdet;
	pfres[2] *= -fdet;
	pfres[3] = ftemp * pf[0];

	return true;
}

template <class T, class S>
void Tridiagonal3 (S* mat, T* diag, T* subd)
{
    T a, b, c, d, e, f, ell, q;

    a = mat[0*3+0];
    b = mat[0*3+1];
    c = mat[0*3+2];
    d = mat[1*3+1];
    e = mat[1*3+2];
    f = mat[2*3+2];

	subd[2] = 0.0;
    diag[0] = a;
    if ( fabs(c) >= g_fEpsilon ) {
        ell = (T)sqrt(b*b+c*c);
        b /= ell;
        c /= ell;
        q = 2*b*e+c*(f-d);
        diag[1] = d+c*q;
        diag[2] = f-c*q;
        subd[0] = ell;
        subd[1] = e-b*q;
        mat[0*3+0] = (S)1; mat[0*3+1] = (S)0; mat[0*3+2] = (T)0;
        mat[1*3+0] = (S)0; mat[1*3+1] = b; mat[1*3+2] = c;
        mat[2*3+0] = (S)0; mat[2*3+1] = c; mat[2*3+2] = -b;
    }
    else {
        diag[1] = d;
        diag[2] = f;
        subd[0] = b;
        subd[1] = e;
        mat[0*3+0] = (S)1; mat[0*3+1] = (S)0; mat[0*3+2] = (S)0;
        mat[1*3+0] = (S)0; mat[1*3+1] = (S)1; mat[1*3+2] = (S)0;
        mat[2*3+0] = (S)0; mat[2*3+1] = (S)0; mat[2*3+2] = (S)1;
    }
}

template <class T>
int Min(T* pts, int stride, int numPts)
{
    assert( pts != NULL && numPts > 0 && stride > 0 );

    int best = pts[0]; pts += stride;
    for(int i = 1; i < numPts; ++i, pts += stride) {
        if( best > pts[0] )
            best = pts[0];
    }

    return best;
}

template <class T>
int Max(T* pts, int stride, int numPts)
{
    assert( pts != NULL && numPts > 0 && stride > 0 );

    int best = pts[0]; pts += stride;
    for(int i = 1; i < numPts; ++i, pts += stride) {
        if( best < pts[0] )
            best = pts[0];
    }

    return best;
}

template <class U>
std::ostream& operator<<(std::ostream& O, const RaveVector<U>& v)
{
    return O << v.x << " " << v.y << " " << v.z << " " << v.w << " ";
}

template <class U>
std::istream& operator>>(std::istream& I, RaveVector<U>& v)
{
    return I >> v.x >> v.y >> v.z >> v.w;
}

template <class U>
std::ostream& operator<<(std::ostream& O, const RaveTransform<U>& v)
{
    return O << v.rot.x << " " << v.rot.y << " " << v.rot.z << " " << v.rot.w << std::endl
             << v.trans.x << " " << v.trans.y << " " << v.trans.z << " ";
}

template <class U>
std::istream& operator>>(std::istream& I, RaveTransform<U>& v)
{
    return I >> v.rot.x >> v.rot.y >> v.rot.z >> v.rot.w >> v.trans.x >> v.trans.y >> v.trans.z;
}

template <class U>
std::ostream& operator<<(std::ostream& O, const RaveTransformMatrix<U>& v)
{
    return O << v.m[0] << " " << v.m[1] << " " << v.m[2] << " " << v.trans.x << std::endl
             << v.m[4] << " " << v.m[5] << " " << v.m[6] << " " << v.trans.y << std::endl
             << v.m[8] << " " << v.m[9] << " " << v.m[10] << " " << v.trans.z << " ";
}

template <class U>
std::istream& operator>>(std::istream& I, RaveTransformMatrix<U>& v)
{
    return I >> v.m[0] >> v.m[1] >> v.m[2] >> v.trans.x
             >> v.m[4] >> v.m[5] >> v.m[6] >> v.trans.y
             >> v.m[8] >> v.m[9] >> v.m[10] >> v.trans.z;
}

#endif
