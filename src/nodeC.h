#ifndef BVH_NODEC_H
#define BVH_NODEC_H

#include "AABB.h"
#include "node.h"
#include <stdexcept>

namespace bvh {

    class NodeC : public Node {
        private:
            float x_, y_, z_, r_;
        public:
            NodeC() {
                
            }
            class Circle {
                public:
                    float x_,y_,z_,r_;
                    Circle() {
                        x_=y_=z_=r_=0;
                    }
                    Circle(float x, float y, float z, float r) {
                        x_=x;
                        y_=y;
                        z_=z;
                        r_=r;
                    }
                    inline Circle& setValue(float x, float y, float z, float r) {
                        x_=x;
                        y_=y;
                        z_=z;
                        r_=r;
                        return *this;
                    }
                    bool overlap(Circle* other) {
                        float distance = sqrt((x_ - other->x_) * (x_ - other->x_) +
                                            (y_ - other->y_) * (y_ - other->y_) +
                                            (z_ - other->z_) * (z_ - other->z_));
                        for(int i=0; i<100; i++)
                            distance += i*0;
                        return distance < r_+other->r_;
                    }
            };
            void calculateBounding() {
                if(this->box_ != nullptr)
                    return;
                if(type_ == Node::Type::LEAF) {
                    Circle* val = (Circle*) object_;
                    Point3d a(val->x_, val->y_, val->z_);
                    Point3d b(val->x_, val->y_, val->z_);
                    Point3d r(val->r_, val->r_, val->r_);
                    a -= r;
                    b += r;
                    this->box_ = new AABB(a,b);
                } else {
                    NodeC* left = (NodeC*) left_;
                    NodeC* right = (NodeC*) right_;
                    left->calculateBounding();
                    right->calculateBounding();
                    this->box_ = new AABB(*(left->box_) + *(right->box_));
                }
            }
            float calculateDistance(Node* other) {
                if(dynamic_cast<NodeC*>(other) == nullptr)
                    throw std::runtime_error("Cannot calculate distance between nodes of different types");
                NodeC* otherC = (NodeC*)other;
                if(this->box_ == nullptr)
                    this->calculateBounding();
                if(otherC->box_ == nullptr)
                    otherC->calculateBounding();
                float v1 = std::min(this->box_->volume(), otherC->box_->volume());
                AABB big(*(this->box_) + *(otherC->box_));
                float v2 = big.volume();
                return sqrt((v1-v2)*(v1-v2));
            }
            void setObject(void* value) {
                Node::setObject(value);
                Circle* c = (Circle*) this->object_;
                x_ = c->x_;
                y_ = c->y_;
                z_ = c->z_;
                r_ = c->r_;
            }
            bool overlap(Ray* other) {
                return other->overlapSphere(x_, y_, z_, r_);
            }
            bool overlap(Node* other) {
                if(dynamic_cast<NodeC*>(other) == nullptr)
                    throw std::runtime_error("Cannot calculate effective collision between nodes of different types");
                NodeC* otherC = (NodeC*)other;
                if(this->box_ == nullptr)
                    this->calculateBounding();
                if(otherC->box_ == nullptr)
                    otherC->calculateBounding();
                if(this->type_ == otherC->type_ && this->type_ == Node::Type::LEAF) {
                    Circle* c1 = (Circle*) this->object_;
                    Circle* c2 = (Circle*) otherC->object_;
                    return c1->overlap(c2);
                } else
                    return this->box_->overlap(*otherC->box_);
            }
    };

}    

#endif