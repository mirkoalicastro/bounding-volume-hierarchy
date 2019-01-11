#ifndef BVH_POINT3D_H
#define BVH_POINT3D_H

#include <cmath>
#include <cstdlib>
#include <algorithm>

namespace bvh {
    class Point3d {
        private:
            float v_[3];
        public:
            Point3d() : Point3d(0,0,0) {
            
            }

            Point3d(const Point3d& other) : Point3d(other.v_[0], other.v_[1], other.v_[2]) {

            }

            Point3d(float x, float y, float z) {
                v_[0] = x;
                v_[1] = y;
                v_[2] = z;
            }

            inline float operator [] (size_t i) const {
                return v_[i];
            }

            inline float& operator[] (size_t i) {
                return v_[i];
            }

            inline Point3d& operator += (const Point3d& other) {
                v_[0] += other.v_[0];
                v_[1] += other.v_[1];
                v_[2] += other.v_[2];
                return *this;
            }

            inline Point3d& operator -= (const Point3d& other) {
                v_[0] -= other.v_[0];
                v_[1] -= other.v_[1];
                v_[2] -= other.v_[2];
                return *this;
            }

            inline Point3d& operator *= (float t) {
                v_[0] *= t;
                v_[1] *= t;
                v_[2] *= t;
                return *this;
            }

            inline Point3d operator + (const Point3d& other) const {
                return Point3d(v_[0] + other.v_[0], v_[1] + other.v_[1], v_[2] + other.v_[2]);
            }

            inline Point3d operator - (const Point3d& other) const {
                return Point3d(v_[0] - other.v_[0], v_[1] - other.v_[1], v_[2] - other.v_[2]);
            }

            inline Point3d operator * (float t) const {
                return Point3d(v_[0] * t, v_[1] * t, v_[2] * t);
            }

            inline float dot(const Point3d& other) const {
                return v_[0] * other.v_[0] + v_[1] * other.v_[1] + v_[2] * other.v_[2];
            }

            inline bool normalize() {
                float len = length();
                v_[0] /= len;
                v_[1] /= len;
                v_[2] /= len;
            }

            inline float length() const {
               return sqrt(squareLength());
            }

            inline float squareLength() const {
                return v_[0] * v_[0] + v_[1] * v_[1] + v_[2] * v_[2];
            }

            inline Point3d& setValue(float x, float y, float z) {
                v_[0] = x;
                v_[1] = y;
                v_[2] = z;
                return *this;
            }

            static Point3d min(const Point3d& a, const Point3d& b) {
                Point3d ret(std::min(a[0], b[0]), std::min(a[1], b[1]), std::min(a[2], b[2]));
                return ret;
            }

            static Point3d max(const Point3d& a, const Point3d& b) {
                Point3d ret(std::max(a[0], b[0]), std::max(a[1], b[1]), std::max(a[2], b[2]));
                return ret;
            }

    };

}

#endif