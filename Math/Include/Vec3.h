#pragma once
#include <cmath>

struct Vec3 {
    float x, y, z;

    Vec3() = default;
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    explicit Vec3(const float* v) : x(v[0]), y(v[1]), z(v[2]) {}

    float& operator[](int i)       { return (&x)[i]; }
    float  operator[](int i) const { return (&x)[i]; }

    Vec3 operator+(const Vec3& o) const { return Vec3(x+o.x, y+o.y, z+o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x-o.x, y-o.y, z-o.z); }
    Vec3 operator*(float s)        const { return Vec3(x*s,   y*s,   z*s);   }
    Vec3 operator/(float s)        const { return Vec3(x/s,   y/s,   z/s);   }
    Vec3 operator-()               const { return Vec3(-x,    -y,    -z);    }

    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x-=o.x; y-=o.y; z-=o.z; return *this; }
    Vec3& operator*=(float s)       { x*=s;   y*=s;   z*=s;   return *this; }
    Vec3& operator/=(float s)       { x/=s;   y/=s;   z/=s;   return *this; }

    bool operator==(const Vec3& o) const { return x==o.x && y==o.y && z==o.z; }
    bool operator!=(const Vec3& o) const { return !(*this == o); }

    float dot(const Vec3& o)   const { return x*o.x + y*o.y + z*o.z; }
    Vec3  cross(const Vec3& o) const { return Vec3(y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x); }
    float lengthSqr()          const { return x*x + y*y + z*z; }
    float length()             const { return sqrtf(lengthSqr()); }
    void  normalize()                { float l = length(); if (l > 0.f) *this /= l; }

    float*       ptr()       { return &x; }
    const float* ptr() const { return &x; }
};

inline Vec3 operator*(float s, const Vec3& v) { return v * s; }

inline float dist(const Vec3& a, const Vec3& b)    { return (a-b).length(); }
inline float distSqr(const Vec3& a, const Vec3& b) { return (a-b).lengthSqr(); }
inline Vec3  lerp(const Vec3& a, const Vec3& b, float t) { return a + (b-a)*t; }
inline Vec3  vmin(const Vec3& a, const Vec3& b) {
    return Vec3(a.x<b.x?a.x:b.x, a.y<b.y?a.y:b.y, a.z<b.z?a.z:b.z);
}
inline Vec3  vmax(const Vec3& a, const Vec3& b) {
    return Vec3(a.x>b.x?a.x:b.x, a.y>b.y?a.y:b.y, a.z>b.z?a.z:b.z);
}

// 2D (xz-plane) helpers
inline float dist2D(const Vec3& a, const Vec3& b)    { float dx=a.x-b.x, dz=a.z-b.z; return sqrtf(dx*dx+dz*dz); }
inline float dist2DSqr(const Vec3& a, const Vec3& b) { float dx=a.x-b.x, dz=a.z-b.z; return dx*dx+dz*dz; }
inline float dot2D(const Vec3& a, const Vec3& b)     { return a.x*b.x + a.z*b.z; }
inline float perp2D(const Vec3& a, const Vec3& b)    { return a.z*b.x - a.x*b.z; }
