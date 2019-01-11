#ifndef BVH_NODE_H
#define BVH_NODE_H

#include <stdexcept>
#include "AABB.h"

namespace bvh {

    class Node {
        public:
            enum Type { LEAF, INTERNAL, UNKNOWN };
            inline Node::Type getType() const {
                return type_;
            }
            inline Node* getLeft() const {
                return left_;
            }
            inline Node* getRight() const {
                return right_;
            }
            inline void* getObject() const {
                return object_;
            }
            inline AABB* getAABB() const {
                return box_;
            }
            Node() {
                type_ = Node::Type::UNKNOWN;
                left_ = right_ = nullptr;
                object_ = (void*) 0;
                id_ = 0;
            }
            inline int getID() const {
                return id_;
            }
            inline void setID(int id) {
                id_ = id;
            }
            void setType(Node::Type type) {
                if(type_ != UNKNOWN)
                   throw std::invalid_argument("Nodes can change their type only once!");
                type_ = type;
            }
            void setObject(void* value) {
                if(type_ != LEAF)
                   throw std::invalid_argument("Only leaf nodes can have values!");
                object_ = value;
            }
            inline void setLeft(Node* node) {
                if(type_ != INTERNAL)
                   throw std::invalid_argument("Only internal nodes can have children!");
                left_ = node;
            }
            inline void setRight(Node* node) {
                if(type_ != INTERNAL)
                   throw std::invalid_argument("Only internal nodes can have children!");
                right_ = node;
            }
            virtual float calculateDistance(Node* other) { }
            virtual bool overlap(Node* other) { return false; }
            virtual bool overlap(Ray* other) { return false; }
        private:
            int id_;
            friend class BVHTree;
        protected:
            Node* left_; /* valid only if node type is internal */
            Node* right_; /* valid only if node type is internal */
            Node::Type type_;
            void* object_; /* valid only if node type is leaf */
            AABB* box_ = nullptr;
    };

}    

#endif