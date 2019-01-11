#ifndef BVH_AABB_H
#define BVH_AABB_H

#include "point3d.h"

namespace bvh {

    /** AABB: box 3d stored as two diagonal points */
    class AABB {
        private:
            Point3d min_;
            Point3d max_;
            friend class Ray;
            
        public:

            /** AABB at v but empty */
            AABB(const Point3d& v) : min_(v), max_(v) {

            }

            /** AABB of a and b */
            AABB(const Point3d& a, const Point3d&b) {
                min_ = bvh::Point3d::min(a, b);
                max_ = bvh::Point3d::max(a, b);
            }

            /** Test if the AABB overlaps another AABB */
            inline bool overlap(const AABB& other) const {
                if(min_[0] > other.max_[0]) return false;
                if(min_[1] > other.max_[1]) return false;
                if(min_[2] > other.max_[2]) return false;

                if(max_[0] < other.min_[0]) return false;
                if(max_[1] < other.min_[1]) return false;
                if(max_[2] < other.min_[2]) return false;

                return true;
            }

            /** Test if the AABB contains a point */
            inline bool contain(const Point3d& p) const {
                if(p[0] < min_[0] || p[0] > max_[0]) return false;
                if(p[1] < min_[1] || p[1] > max_[1]) return false;
                if(p[2] < min_[2] || p[2] > max_[2]) return false;

                return true;
            }

            /** Merge the AABB and a point */
            inline AABB& operator += (const Point3d& p) {
                min_ = bvh::Point3d::min(min_, p);
                max_ = bvh::Point3d::max(max_, p);
                return *this;
            }

            /** Merge the AABB and another AABB */
            inline AABB& operator += (const AABB& other) {
                min_ = bvh::Point3d::min(min_, other.min_);
                max_ = bvh::Point3d::max(max_, other.max_);
                return *this;
            }

            /** Return the merged AABB of current AABB and the other one */
            inline AABB operator + (const AABB& other) const {
                AABB res(*this);
                return res += other;
            }

            /** Width of the AABB */
            inline float width() const {
                return max_[0] - min_[0];
            }

            /** Height of the AABB */
            inline float height() const {
                return max_[1] - min_[1];
            }

            /** Depth of the AABB */
            inline float depth() const {
                return max_[2] - min_[2];
            }

            /** Volume of the AABB */
            inline float volume() const {
                return width() * height() * depth();
            }

            /** Size of the AABB */
            inline float size() const {
                return (max_ - min_).squareLength();
            }

            /** Center of the AABB */
            inline  Point3d center() const {
                return (min_ + max_) * 0.5;
            }
    };

}

#endif
