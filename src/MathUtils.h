#pragma once

#include <cmath>  
#include <algorithm> 

/**
 * @class Vector3
 * @brief Represents a 3D vector for physics calculations.
 */
struct Vector3 {
    float x, y, z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vector3& operator+=(const Vector3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vector3& operator-=(const Vector3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vector3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
    Vector3& operator/=(float s) { x /= s; y /= s; z /= s; return *this; }

    Vector3 operator+(const Vector3& v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vector3 operator-(const Vector3& v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vector3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vector3 operator/(float s) const { return {x / s, y / s, z / s}; }

    float Length() const { return std::sqrt(x*x + y*y + z*z); }
    Vector3 Normalize() const { float len = Length(); return len > 0 ? *this / len : *this; }
    float Dot(const Vector3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vector3 Cross(const Vector3& v) const { return {y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x}; }
};

inline Vector3 operator*(float scalar, const Vector3& v) {
    return Vector3(v.x * scalar, v.y * scalar, v.z * scalar);
}

/**
 * @class Quaternion
 * @brief Represents a quaternion for 3D rotation.
 */
struct Quaternion {
    float w, x, y, z;

    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
    Quaternion(const Vector3& v, float scalar) : w(scalar), x(v.x), y(v.y), z(v.z) {}

    static Quaternion Identity() { return {1, 0, 0, 0}; }

    Quaternion operator*(const Quaternion& q) const {
        return {w * q.w - x * q.x - y * q.y - z * q.z,
                w * q.x + x * q.w + y * q.z - z * q.y,
                w * q.y - x * q.z + y * q.w + z * q.x,
                w * q.z + x * q.y - y * q.x + z * q.w};
    }

    Quaternion& operator+=(const Quaternion& q) {
        w += q.w; x += q.x; y += q.y; z += q.z;
        return *this;
    }

    Quaternion Conjugate() const { return {w, -x, -y, -z}; }

    void Normalize() {
        float length = std::sqrt(w * w + x * x + y * y + z * z);
        if (length > 0.0f) {
            float invLen = 1.0f / length;
            w *= invLen; x *= invLen; y *= invLen; z *= invLen;
        }
    }

    Quaternion operator*(float scalar) const {
        return {w * scalar, x * scalar, y * scalar, z * scalar};
    }

    Quaternion& operator*=(float scalar) {
        w *= scalar; x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }

    // Convert quaternion to rotation matrix
    Matrix3 ToMatrix() const;
};

/**
 * @class Matrix3
 * @brief Represents a 3x3 matrix for rotational inertia calculations.
 */
struct Matrix3 {
    float m[3][3];

    Matrix3() { std::fill(&m[0][0], &m[0][0] + 9, 0.0f); }
    Matrix3(float diag) { std::fill(&m[0][0], &m[0][0] + 9, 0.0f); m[0][0] = m[1][1] = m[2][2] = diag; }
    Matrix3(const Vector3& row1, const Vector3& row2, const Vector3& row3) {
        m[0][0] = row1.x; m[0][1] = row1.y; m[0][2] = row1.z;
        m[1][0] = row2.x; m[1][1] = row2.y; m[1][2] = row2.z;
        m[2][0] = row3.x; m[2][1] = row3.y; m[2][2] = row3.z;
    }

    Vector3 operator*(const Vector3& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }

    Matrix3 Transpose() const {
        return {
            {m[0][0], m[1][0], m[2][0]},
            {m[0][1], m[1][1], m[2][1]},
            {m[0][2], m[1][2], m[2][2]}
        };
    }

    Vector3 GetColumn(int index) const {
        return {m[0][index], m[1][index], m[2][index]};
    }

    Matrix3 Abs() const {
        return {
            {std::fabs(m[0][0]), std::fabs(m[0][1]), std::fabs(m[0][2])},
            {std::fabs(m[1][0]), std::fabs(m[1][1]), std::fabs(m[1][2])},
            {std::fabs(m[2][0]), std::fabs(m[2][1]), std::fabs(m[2][2])}
        };
    }
};

// Implement Quaternion → Matrix3 conversion
inline Matrix3 Quaternion::ToMatrix() const {
    return Matrix3(
        Vector3(1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y),
        Vector3(2 * x * y + 2 * w * z, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * w * x),
        Vector3(2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x * x - 2 * y * y)
    );
}
