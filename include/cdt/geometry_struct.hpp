#ifndef _GEOMETRY_STRUCT_H_
#define _GEOMETRY_STRUCT_H_

#include <iostream>
#include <cmath>
#include <list>
#include <vector>
using namespace std;

struct Vertex;
struct Side;
struct Triangle;

struct Vertex { // vector struct. For easy vector calculation.
    float x; // normalized x 
    float y; // normalized y 
    list<Triangle*> tri; // Triangle pointer related to this point.

    Vertex() { x = y = 0; }
    Vertex(float x, float y) {
        this->x = x;
        this->y = y;
    }
    const float dist() const { // distance
        return sqrtf(x * x + y * y);
    }
    const float inner(const Vertex &a) const { // inner product
        return x * a.x + y * a.y;
    }
    const float cross(const Vertex &a) const { // cross product (determinant)
        return x * a.y - y * a.x;
    }
    const Vertex operator+ (const Vertex &a) const { // Vector summation
        return Vertex(x + a.x, y + a.y);
    }
    const Vertex operator- (const Vertex &a) const { // Vector subtraction
        return Vertex(x - a.x, y - a.y);
    }
    const Vertex operator* (const float &a) const { // scalar product
        return Vertex(a * x, a * y);
    }
    const Vertex proj(const Vertex &a) const { // projection vector
        return *this * (inner(*this) / inner(a));
    }
    const float get_cos(const Vertex &a) const { // cosine between two vectors.
        return inner(a) / (dist() * a.dist());
    }
};
struct Side { // edge
    int a; // starting point
    int b; // end point
    Side() { a = b = -1; };
    Side(int a_, int b_) { a = a_; b = b_;};
};
class Triangle {
public:
    Triangle* adj[3]; // same index of i means an adjacent Triangle not containing points_[idx[i]].
    int idx[3]; // point indexes. left-most (minimum x value) is '0-th' point. By an anticlockwise, ascending numbering.
    int id;
    // methods
    Triangle() {
        idx[0] = -1; idx[1] = -1; idx[2] = -1;
        adj[0] = nullptr, adj[1] = nullptr, adj[2] = nullptr;
        id = -1;
    }
    Triangle(int a, int b, int c, int id_) {
        idx[0] = a; idx[1] = b; idx[2] = c;
        adj[0] = nullptr, adj[1] = nullptr, adj[2] = nullptr;
        id = id_;
    }
    ~Triangle() {
#ifdef _VERBOSE_
        cout << "  delete Triangle [" << id << "]\n";
#endif
    }
};
#endif