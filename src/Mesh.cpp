#include "Mesh.h"
#include <map>
using namespace std;

Vertex operator * (const float s, const Vertex & v) {
    return Vertex(v[0]*s, v[1]*s, v[2]*s);
}
void Mesh::subdivide() {
    //split

    vector<Vertex> odd_vertexPositions;
    vector<Triangle> new_triangleIndices;
    vector<Uvec2> neighbours; //to know if the odd_vertex already exists

    vector<Uvec2> neighbour_triangles;// to move odd_vertices

    map<unsigned int, vector <unsigned int>> odd_n;

    vector <Uvec2>::iterator it;
    int counter = vertices.size();
    Vertex x, y ,z;
    Triangle new_triangle;

    int triangle_count = 0;


    auto create_odd_vertices=[&](unsigned int & odd_vertex, unsigned int even_vertex1, unsigned int even_vertex2) -> void {


        unsigned int tmp;
        if(even_vertex1 > even_vertex2) {
            tmp = even_vertex1;
            even_vertex1 = even_vertex2;
            even_vertex2 = tmp;
        }

        it = find(neighbours.begin(), neighbours.end(), Uvec2(even_vertex1, even_vertex2));

        if(it == neighbours.end()) {
            odd_vertexPositions.push_back((vertices[even_vertex1]+vertices[even_vertex2])/2.f);
            odd_vertex = counter;
            odd_n[even_vertex1].push_back(counter - vertices.size());
            odd_n[even_vertex2].push_back(counter - vertices.size());
            counter ++;
            neighbour_triangles.push_back(Uvec2(triangle_count, numeric_limits<unsigned int>::infinity()));
            neighbours.push_back(Uvec2(even_vertex1, even_vertex2));
        } else {
            neighbour_triangles[std::distance(neighbours.begin(), it)][1] = triangle_count;
            odd_vertex = std::distance(neighbours.begin(), it) + vertices.size();
        }


    };

    for(const Triangle & triangle : triangles) {

        create_odd_vertices(new_triangle[0], triangle[0], triangle[1]);
        create_odd_vertices(new_triangle[1], triangle[1], triangle[2]);
        create_odd_vertices(new_triangle[2], triangle[2], triangle[0]);


        new_triangleIndices.push_back(new_triangle);
        new_triangleIndices.push_back(Triangle(triangle[0], new_triangle[0], new_triangle[2]));
        new_triangleIndices.push_back(Triangle(triangle[1], new_triangle[1], new_triangle[0]));
        new_triangleIndices.push_back(Triangle(triangle[2], new_triangle[2], new_triangle[1]));
        triangle_count ++;

    }


    vector<Vertex> old_odd = odd_vertexPositions;

    //move odd
    int len = odd_vertexPositions.size();
    unsigned int triangle1[3];
    unsigned int triangle2[3];
    unsigned int A, B, C, D;

    for(int i = 0; i < len; i++) {
        if (neighbour_triangles[i][1] != numeric_limits<unsigned int>::infinity()) {
            triangle1[0] = triangles[neighbour_triangles[i][0]][0];
            triangle2[0] = triangles[neighbour_triangles[i][1]][0];
            ;
            triangle1[1] = triangles[neighbour_triangles[i][0]][1];
            triangle2[1] = triangles[neighbour_triangles[i][1]][1];
            ;
            triangle1[2] = triangles[neighbour_triangles[i][0]][2];
            triangle2[2] = triangles[neighbour_triangles[i][1]][2];

            sort(triangle1, triangle1+3);
            sort(triangle2, triangle2+3);

            if(triangle1[1] == triangle2[1]) {
                B = triangle1[1];
                if (triangle1[0] == triangle2[0]) {
                    C = triangle1[0];
                    A = triangle1[2];
                    D = triangle2[2];
                } else {
                    C = triangle1[2];
                    A = triangle1[0];
                    D = triangle2[0];
                }
            } else if(triangle1[1] == triangle2[0]) {
                B = triangle1[1];
                A = triangle1[0];
                C = triangle1[2];
                if (triangle1[2] == triangle2[1]) {
                    D = triangle2[2];
                } else {
                    D = triangle2[1];
                }
            } else if(triangle1[1] == triangle2[2]) {
                B = triangle1[1];
                A = triangle1[2];
                C = triangle1[0];

                if (triangle1[0] == triangle2[1]) {
                    D = triangle2[0];
                } else {
                    D = triangle2[1];
                }
            } else {
                A = triangle1[1];
                B = triangle1[0];
                C = triangle1[2];
                D = triangle2[1];
            }
            odd_vertexPositions[i] = (vertices[A] + vertices[D])/8.f +(3.f/8.f)*(vertices[B] + vertices[C]);
        }

    }
    // move even
    int n;
    Vertex res;
    float alpha;

    for(const Triangle & triangle : triangles) {
        n = odd_n[triangle[0]].size();
        alpha = 1.f/64.f * (40.f - pow(3.f + 2.f * cos(2*M_PI / (float)n), 2));

        res = Vertex(0.f, 0.f, 0.f);

        for(unsigned int i : odd_n[triangle[0]])
            res += alpha/n *  old_odd[i];
        res += (1-alpha)*vertices[triangle[0]];
        vertices[triangle[0]] = res;

        res = Vertex(0.f, 0.f, 0.f);

        n = odd_n[triangle[1]].size();
        alpha = 1.f/64.f * (40.f - pow(3.f + 2.f * cos(2*M_PI / (float)n), 2));

        for(unsigned int i : odd_n[triangle[1]])
            res += alpha/n *  old_odd[i];
        res += (1-alpha)*vertices[triangle[1]];
        vertices[triangle[1]] = res;

        res = Vertex(0.f, 0.f, 0.f);
        n = odd_n[triangle[2]].size();
        alpha = 1.f/64.f * (40.f - pow(3.f + 2.f * cos(2*M_PI / (float)n), 2));

        for(unsigned int i : odd_n[triangle[2]])
            res += alpha/n *  old_odd[i];
        res += (1-alpha)*vertices[triangle[2]];
        vertices[triangle[2]] = res;
    }
    vertices.insert(vertices.end(), odd_vertexPositions.begin(), odd_vertexPositions.end());

    triangles = new_triangleIndices;
    //this->recomputePerVertexNormals(false);

}
