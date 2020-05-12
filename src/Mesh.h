#ifndef PROJECTMESH_H
#define PROJECTMESH_H

#include <vector>
#include "point3.h"
#include <map>
#include <eigen3/Eigen/SparseCore>

typedef Eigen::SparseMatrix<float> SpMat;

struct Vertex{
    point3d p;
    Vertex() {}
    Vertex(float x , float y , float z) : p(x,y,z) {}
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
    friend Vertex operator * (const float s, const Vertex & v) {
        return Vertex(v[0]*s, v[1]*s, v[2]*s);
    }

};


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

struct GausCoeff {
    Vertex mean;
    float variance;
    GausCoeff(Vertex _mean = {0,0,0}, float _variance = 1) : mean(_mean), variance(_variance) {}
};

struct Mesh{
    std::vector< Vertex > vertices;
    std::vector< Vertex > basicVertices;
    std::vector< Triangle > triangles;
    std::vector< Triangle > basicTriangles;

    std::vector< std::map< unsigned int, float > > coeffs;

    std::vector<SpMat> Ap; //compute in subdivise
    SpMat A_1; // compute in subdivise Sparse ??
    std::vector<SpMat> Qi; //Sparse ??

    void subdivide();
    void redisplay(); //TO DO: integrate this function in transform
    void basicDisplay();
    void computeQi(const std::vector<GausCoeff>);
    void transform(const float **);
};
#endif // PROJECTMESH_H
