// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class, 3D math helper
// 6/5/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-05 - add 3D math helper file to DMP6 example sketch

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _HELPER_3DMATH_H_
#define _HELPER_3DMATH_H_

#include <Arduino.h>
//I would like to have here std::numeric_limits, but...
template <typename T>
struct epsilon_t
{
    inline static T epsilon();
};

template <>
struct epsilon_t<uint8_t>
{
    inline static uint8_t epsilon(){ return 0;}
};

template <>
struct epsilon_t<int>
{
    inline static int epsilon(){ return 0;}
};


template <>
struct  epsilon_t<float>
{
    inline static float epsilon(){ return 0.0001f;}
};

template <>
struct  epsilon_t<double >
{
    inline static double epsilon(){ return 0.00000001;}
};




template <typename T>
class QuaternionTempl
{
public:
    T w;
    T x;
    T y;
    T z;

    bool isZero(T v)
    {
        return (abs(v) <= epsilon_t<T>::epsilon());
    }

    QuaternionTempl()
    {
        zero();
    }

    QuaternionTempl(T nw, T nx, T ny, T nz)
    {
       set(nw, nx, ny, nz);
    }

    void zero()
    {
        w = static_cast<T>(1);
        x = static_cast<T>(0);
        y = static_cast<T>(0);
        z = static_cast<T>(0);
    }

    void set(T nw, T nx, T ny, T nz)
    {
        w = nw;
        x = nx;
        y = ny;
        z = nz;
    }

    QuaternionTempl(const QuaternionTempl<T>& c)               = default;
    QuaternionTempl<T>& operator=(const QuaternionTempl<T>& c) = default;

    QuaternionTempl<T> getProduct(const QuaternionTempl<T>& q) const
    {
        // Quaternion multiplication is defined by:
        //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
        //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
        //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
        //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2

        return QuaternionTempl<T>(w*q.w - x*q.x - y*q.y - z*q.z,  // new w
        w*q.x + x*q.w + y*q.z - z*q.y,  // new x
        w*q.y - x*q.z + y*q.w + z*q.x,  // new y
        w*q.z + x*q.y - y*q.x + z*q.w); // new z
    }

    QuaternionTempl<T> getConjugate() const
    {
        return QuaternionTempl<T>(w, -x, -y, -z);
    }

    T getMagnitude() const
    {
        return sqrt(w*w + x*x + y*y + z*z);
    }

    void scale(T val)
    {
        w *= val;
        x *= val;
        y *= val;
        z *= val;
    }

    QuaternionTempl<T>&operator *= (T val)
    {
        scale(val);
        return *this;
    }

    QuaternionTempl<T> operator * (T val) const
    {
        QuaternionTempl<T> copy(*this);
        copy.scale(val);
        return copy;
    }

    QuaternionTempl<T>& operator -= (const QuaternionTempl<T>& v)
    {
        w -= v.w;
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    QuaternionTempl<T>& operator += (const QuaternionTempl<T>& v)
    {
        w += v.w;
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    QuaternionTempl<T> operator - (const QuaternionTempl<T>& v) const
    {
        return  QuaternionTempl<T>(w - v.w, x-v.x, y-v.y, z-v.z);
    }

    QuaternionTempl<T> operator + (const QuaternionTempl<T>& v) const
    {
        return  QuaternionTempl<T>(w + v.w, x + v.x, y + v.y, z + v.z);
    }

    void normalize()
    {
        auto m = getMagnitude();
        scale(static_cast<decltype(m)>(1) / m);
    }

    QuaternionTempl<T> getNormalized() const
    {
        QuaternionTempl<T> r(*this);
        r.normalize();
        return r;
    }

    /**
     * Converts the quaternion into a float array consisting of: rotation angle
     * in radians, rotation axis x,y,z
     *
     * @return 4-element float array
     *
     * from https://github.com/postspectacular/toxiclibs/blob/master/src.core/toxi/geom/Quaternion.java
     */
    QuaternionTempl<T> toAxisAngle() const
    {
        //for integers it will have no sence actually
        T sa = sqrt(1. - w * w);
        if (isZero(sa))
            sa = static_cast<T>(1);
        else
            sa = static_cast<T>(1) / sa;
        return QuaternionTempl<T>(acos(w) * 2., x * sa, y * sa, z * sa);
    }
};

using Quaternion = QuaternionTempl<float>;

template <typename T>
class Vector
{
public:
    T x;
    T y;
    T z;

    Vector()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    Vector(T nx, T ny, T nz)
    {
        x = nx;
        y = ny;
        z = nz;
    }

    Vector(const Vector<T>&) = default;
    Vector<T>& operator=(const Vector<T>&) = default;
    ~Vector() = default;

    T getMagnitude() const
    {
        return static_cast<T>(sqrt(x*x + y*y + z*z));
    }

    void normalize()
    {
        auto m = getMagnitude();
        x /= m;
        y /= m;
        z /= m;
    }

    Vector<T> getNormalized() const
    {
        Vector<T> r(x, y, z);
        r.normalize();
        return r;
    }

    T getSProd(const Vector<T>& c) const
    {
        return static_cast<T>(x * c.x + y * c.y + z * c.z); //casting out overflows (for integers, take care of it)
    }

    Vector<T> operator * (T v) const
    {
        return Vector<T>(x * v, y * v, z * v);
    }

    Vector<T>& operator *= (T v)
    {
        x *= v;
        y *= v;
        z *= v;
        return *this;
    }

    Vector<T>& operator -= (const Vector<T>& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    Vector<T>& operator += (const Vector<T>& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    Vector<T> operator - (const Vector<T>& v) const
    {
        return  Vector<T>(x-v.x, y-v.y, z-v.z);
    }

    Vector<T> operator + (const Vector<T>& v) const
    {
        return  Vector<T>(x + v.x, y + v.y, z + v.z);
    }

    inline void rotate(const Quaternion* q)
    {
        return rotate(*q);
    }

    void rotate(const Quaternion& q)
    {
        // http://www.cprogramming.com/tutorial/3d/quaternions.html
        // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
        // http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
        // ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1

        // P_out = q * P_in * conj(q)
        // - P_out is the output vector
        // - q is the orientation quaternion
        // - P_in is the input vector (a*aReal)
        // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
        Quaternion p(0, x, y, z);

        // quaternion multiplication: q * p, stored back in p
        p = q.getProduct(p);

        // quaternion multiplication: p * conj(q), stored back in p

        p = p.getProduct(q.getConjugate().getNormalized());

        // p quaternion is now [0, x', y', z']
        x = static_cast<decltype(x)>(p.x);
        y = static_cast<decltype(y)>(p.y);
        z = static_cast<decltype(z)>(p.z);
    }

    Vector<T> getRotated(const Quaternion &q) const
    {
        Vector<T> r(x, y, z);
        r.rotate(q);
        return r;
    }
};

using VectorInt16 = Vector<int16_t>;
using VectorFloat = Vector<float>;


#endif /* _HELPER_3DMATH_H_ */