#ifndef PROJECTMESH_H
#define PROJECTMESH_H

#include <vector>
#include "point3.h"
#include <map>

struct Vertex{
    point3d p;
    Vertex() {}
    Vertex(double x , double y , double z) : p(x,y,z) {}
    double & operator [] (unsigned int c) { return p[c]; }
    double operator [] (unsigned int c) const { return p[c]; }
    Vertex operator + (const Vertex & v) { return Vertex(
            p[0]+v[0],
            p[1]+v[1],
            p[2]+v[2]);}
    Vertex operator / (const float s) { return Vertex(p[0]/s, p[1]/s, p[2]/s);}
    Vertex operator += (const Vertex & v) { return Vertex(
            p[0]+=v[0],
            p[1]+=v[1],
            p[2]+=v[2]);}

};

Vertex operator * (const float s, const Vertex & v);

struct Triangle{
    unsigned int corners[3];
    Triangle () {}
    Triangle(unsigned int x , unsigned int y , unsigned int z) : corners{x, y, z} {}
    unsigned int & operator [] (unsigned int c) { return corners[c]; }
    unsigned int operator [] (unsigned int c) const { return corners[c]; }
    unsigned int size() const { return 3 ; }
};

class Uvec2{
    private:
        unsigned int corners[2];
    public:
        Uvec2(unsigned int x, unsigned int y) : corners{x, y} {}
        unsigned int & operator [] (unsigned int c) { return corners[c]; }
        unsigned int operator [] (unsigned int c) const { return corners[c]; }
        unsigned int size() const { return 2 ; }
        bool operator == (const Uvec2 & v) { return (corners[0] == v[0]) && (corners[1] == v[1]);}
};

struct coeff{
    unsigned int vertex;
    float lambda;
    coeff(unsigned int _vertex = 0, float _lambda = 0): vertex(_vertex), lambda(_lambda) {}
};

struct Mesh{
    std::vector< Vertex > vertices;
    std::vector< Vertex > basicVertices;
    std::vector< Triangle > triangles;
    std::vector< Triangle > basicTriangles;
    std::vector< std::map< unsigned int, float > > coeffs;
    std::vector< std::map< unsigned int, float > > newCoeffs;
    void subdivide();
    void addCoeff(unsigned int vertex, coeff k);
    void addNewCoeff(unsigned int vertex, coeff k);
    void redisplay();
    void basicDisplay();
};



#endif // PROJECTMESH_H
