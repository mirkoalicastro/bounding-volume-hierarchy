#ifndef BVH_BVHTREE_H
#define BVH_BVHTREE_H

#include <stdexcept>
#include "ray.h"
#include "nodeC.h"
#include "node.h"

namespace bvh {

    class BVHTree {
        private:
            float h_;
            float threshold_;
            Node* root_ = nullptr;
            int c_height(Node* el) {
                if (el == nullptr)
                    return 0;

                return std::max(c_height(el->left_), c_height(el->right_)) + 1;
            }
            void fill(std::vector<Node*> *ret, Node* cur, int l, int rl) {
                if(cur == nullptr)
                    return;
                if(l == rl || cur->type_ == Node::Type::LEAF) {
                    (*ret).push_back(cur);
                    return;
                }
                fill(ret, cur->left_, l+1, rl);
                fill(ret, cur->right_, l+1, rl);
            }
            void fillExactly(std::vector<Node*> *ret, Node* cur, int l, int rl) {
                if(cur == nullptr)
                    return;
                if(l == rl) {
                    (*ret).push_back(cur);
                    return;
                }
                fill(ret, cur->left_, l+1, rl);
                fill(ret, cur->right_, l+1, rl);
            }
            // 'Descend A' descent rule
            bool DescendA(Node* a, Node* b) {
                return a->type_ != Node::Type::LEAF;
            }
/*            // 'Descend B' descent rule
            bool DescendA(Node* a, Node* b) {
                return b->getType() == Node::Type::LEAF;
            }
            // 'Descend larger' descent rule
            bool DescendA(Node* a, Node* b) {
                return b->getType() == Node::Type::LEAF || (a->getType() != Node::Type::LEAF && sizeOfTree(a) >= sizeOfTree(b));
            }*/
            int BVHCollision(std::vector<int> *collisionResult, Node* a, Node* b) {
                if (!(a->box_->overlap(*b->box_)))
                    return 0;
                if (a->type_ == b->type_ && a->type_ == Node::Type::LEAF) {
                    if(a->overlap(b)) {
                        (*collisionResult).push_back(a->id_);
                        (*collisionResult).push_back(b->id_);
                        return 1;
                    }
                    return 0;
                } else {
                    if (DescendA(a, b))
                        return BVHCollision(collisionResult, a->left_, b) + BVHCollision(collisionResult, a->right_, b);
                    else
                        return BVHCollision(collisionResult, a, b->left_) + BVHCollision(collisionResult, a, b->right_);
                }
            }
            bool BVHCollision(Node* a, Node* b) {
                if (!(a->box_->overlap(*b->box_)))
                    return false;
                if (a->type_ == b->type_ && a->type_ == Node::Type::LEAF) {
                // At leaf nodes. Perform collision tests on leaf node contents
                    return a->overlap(b);
                } else {
                    if (DescendA(a, b))
                        return BVHCollision(a->left_, b) || BVHCollision(a->right_, b);
                    else
                        return BVHCollision(a, b->left_) || BVHCollision(a, b->right_);
                }
            }

            void findNodesToMerge(NodeC** nodes, int numObjects, int *i, int *j) {
                int a=0,b=1;
                int ta=0,tb=1;
                float bestDistance = INFINITY;
                do {
                    float tmp = nodes[ta]->calculateDistance(nodes[tb]);
                    if(tmp < bestDistance) {
                        bestDistance = tmp;
                        a = ta;
                        b = tb;
                        if(tmp < threshold_)
                            break;
                    }
                    tb++;
                    if(tb == numObjects) {
                        ta++;
                        tb = ta+1;
                    }
                    if(tb >= numObjects)
                        break;
                } while(true);
                *i = a;
                *j = b;
            }   

            int BVHRayCollision(std::vector<int> *collisionResult, Node *a, Ray *r) {
                if(!(r->overlapAABB(*a->box_)))
                    return 1;
                if (a->type_ == Node::Type::LEAF) {
                    if(a->overlap(r))
                        (*collisionResult).push_back(a->id_);
                    return 1;
                } else
                    return BVHRayCollision(collisionResult, a->left_, r) + BVHRayCollision(collisionResult, a->right_, r);
            }

        public:
            BVHTree(float threshold=0.5f) {
                threshold_ = threshold;
                h_ = 0;
            }
            Node* getRoot() {
                return root_;
            }
            int rayCollision(std::vector<int> *collisionResult, Ray *ray) {
                return BVHRayCollision(collisionResult, root_, ray);
            }
            int overlaps(std::vector<int> *collisionResult, BVHTree *other) {
                return BVHCollision(collisionResult, root_, other->root_);
            }
            bool overlaps(BVHTree *other) {
                return BVHCollision(root_, other->root_);
            }
            std::vector<Node*> getAtLevel(int i) {
                std::vector<Node*> ret;
                if(i < 0 || i >= height())
                    return ret;
                fill(&ret, root_, 0, i);
            }
            std::vector<Node*> getExactlyAtLevel(int i) {
                std::vector<Node*> ret;
                if(i < 0 || i >= height())
                    return ret;
                fillExactly(&ret, root_, 0, i);
            }

            void bottom_up(NodeC** nodes, int numObjects, bool print=false) {
                if(numObjects < 1)
                    return;
                int i, j;
                while (numObjects > 1) {
                    if(print)
                        std::cout << "Remaining objects: " << numObjects << "..." << std::endl;
                    findNodesToMerge(&nodes[0], numObjects, &i, &j);
                    NodeC *parent = new NodeC();
                    parent->type_ = Node::Type::INTERNAL;
                    parent->left_ = nodes[i];
                    parent->right_ = nodes[j];
                    parent->calculateBounding();
                    int min = i, max = j;
                    if (i > j)
                        min = j, max = i;
                    nodes[min] = parent;
                    nodes[max] = nodes[numObjects - 1];
                    numObjects--;
                }
                root_ = nodes[0];
                h_ = c_height(root_);
            }

            int height() {
                return h_;
            }


    };

}    

#endif