#ifndef PROJECTMESH_H
#define PROJECTMESH_H

#include <vector>
#include "point3.h"
#include <map>
#include <eigen3/Eigen/Dense>
#include <math.h>

struct Vertex{
    point3d p;
    Vertex() {}
    Vertex(float x , float y , float z) : p(x,y,z) {}
    double & operator [] (unsigned int c) { return p[c]; }
    double operator [] (unsigned int c) const { return p[c]; }
    Vertex operator + (const Vertex & v) const { return Vertex(
            p[0]+v[0],
            p[1]+v[1],
            p[2]+v[2]);}
    Vertex operator - (const Vertex & v) const { return Vertex(
            p[0]-v[0],
            p[1]-v[1],
            p[2]-v[2]);}
    Vertex operator / (const float s) const { return Vertex(p[0]/s, p[1]/s, p[2]/s);}
    Vertex operator += (const Vertex & v) { return Vertex(
            p[0]+=v[0],
            p[1]+=v[1],
            p[2]+=v[2]);}
    friend Vertex operator * (const float s, const Vertex & v) {
        return Vertex(v[0]*s, v[1]*s, v[2]*s);
    }

    float sqrnorm() const {
        return p.sqrnorm();
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
    std::vector< Vertex > resetVertices;
    std::vector< Triangle > triangles;
    std::vector< Triangle > basicTriangles;
    std::vector< std::map< unsigned int, float > > coeffs;
    std::vector<Eigen::MatrixXf> Qis;

    void subdivide();
    void reset();
    void computeQis(const std::vector<GausCoeff>);
    void transform(const std::vector<Eigen::MatrixXf> & T);
    void transform_Basic(const std::vector<Eigen::MatrixXf> & T, const std::vector<GausCoeff>gCoeffs);

    private:
    float area_d3 (unsigned int k) {
        const Triangle T = triangles[k];
        const Vertex AB = vertices[T[1]] - vertices[T[0]];
        const Vertex AC = vertices[T[2]] - vertices[T[0]];
        return sqrt(pow(AB[1]*AC[2] - AB[2]*AC[1],2) + pow(AB[2]*AC[0] - AB[0] * AC[2],2) + pow(AB[0]*AC[1] - AB[1] * AC[0],2))/6;
    }
};
#endif // PROJECTMESH_H
