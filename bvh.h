#include <string>
#include <vector>
#include "parser.h"

using namespace parser;

class BVHNode {
public:
    BoundingBox box;    // The bounding box for this node
    BVHNode* left;      // Left child
    BVHNode* right;     // Right child
    std::vector<Face> faces;  // Leaf node stores objects

    BVHNode() {this->left = nullptr; this->right = nullptr;}
};