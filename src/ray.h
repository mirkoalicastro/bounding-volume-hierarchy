#ifndef BVH_RAY_H
#define BVH_RAY_H

#include <cmath>
#include <cstdlib>
#include <algorithm>
#include "AABB.h"
#include "point3d.h"

namespace bvh {
    class Ray {
        private:
            Point3d o_, d_;

            bool solveQuadratic(float a, float b, float c, float &x0, float &x1) { 
                float discr = b*b - 4*a*c;
                if (discr < 0)
                    return false; 
                if (discr == 0)
                    x0 = x1 = - 0.5 * b / a;
                else {
                    float q;
                    if(b > 0)
                        q = -0.5 * (b + sqrt(discr));
                    else
                        q = -0.5 * (b - sqrt(discr));
                    x0 = q / a; 
                    x1 = c / q; 
                }

                return true; 
            }

        public:

            Ray() : Ray(0,0,0,0,0,0) {

            }

            /* Vectors must be already normalized */
            Ray(const Point3d& o, const Point3d& d) : o_(o), d_(d) {

            }

            /* Vectors must be already normalized */
            Ray(const float ox, const float oy, const float oz, const float dx, const float dy, const float dz) : o_(ox, oy, oz), d_(dx, dy, dz) {

            }

            void setOrigin(const Point3d& o) {
                o_ = o;
            }
            
            void setDirection(const Point3d& d) {
                d_ = d;
            }
            
            void setOrigin(const float ox, const float oy, const float oz) {
                o_.setValue(ox, oy, oz);
            }
            
            void setDirection(const float dx, const float dy, const float dz) {
                d_.setValue(dx, dy, dz);
            }
            
            bool overlapAABB(const AABB& other) {
                float tmin = (other.min_[0] - o_[0]) / d_[0]; 
                float tmax = (other.max_[0] - o_[0]) / d_[0];
            
                if (tmin > tmax)
                    std::swap(tmin, tmax); 

                float tymin = (other.min_[1] - o_[1]) / d_[1]; 
                float tymax = (other.max_[1] - o_[1]) / d_[1];

                if (tymin > tymax)
                    std::swap(tymin, tymax);

                if ((tmin > tymax) || (tymin > tmax))
                    return false;

                if (tymin > tmin)
                    tmin = tymin;

                if (tymax < tmax)
                    tmax = tymax;
            
                float tzmin = (other.min_[2] - o_[2]) / d_[2]; 
                float tzmax = (other.max_[2] - o_[2]) / d_[2];
            
                if (tzmin > tzmax)
                    std::swap(tzmin, tzmax);

                if ((tmin > tzmax) || (tzmin > tmax)) 
                    return false; 
                    
                if (tzmin > tmin) 
                    tmin = tzmin; 
            
                if (tzmax < tmax) 
                    tmax = tzmax; 
            
                return true; 

            }

            bool overlapSphere(float x, float y, float z, float radius) {
                float tmp = 0;
                for(int i=0; i<250; i++)
                    tmp += i*0;
                float t; //useless for this analysis

                float radius2 = radius*radius;

                float lx = o_[0] - x;
                float ly = o_[1] - y;
                float lz = o_[2] - z;

                float a = d_[0]*d_[0] + d_[1]*d_[1] + d_[2]*d_[2]; //1
                float b = 2 * (d_[0]*lx + d_[1]*ly + d_[2]*lz);
                float c = (lx*lx + ly*ly + lz*lz) - radius2;

                float t0, t1;

                if(!solveQuadratic(a,b,c, t0, t1))
                    return false;

                if(t0 > t1)
                    std::swap(t0, t1);

                if (t0 < 0) { 
                    t = t1;
                    if (t1 < 0)
                        return false; 
                } else
                    t = t0; 

                return true;
            }
    };

}

#endif