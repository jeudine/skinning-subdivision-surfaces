#include "Mesh.h"

using namespace std;

Vertex operator * (const float s, const Vertex & v) {
    return Vertex(v[0]*s, v[1]*s, v[2]*s);
}

void Mesh::subdivide() {
    //split

    vector<Triangle> new_triangleIndices;
    vector<Uvec2> neighbours; //to know if the odd_vertex already exists

    vector<Uvec2> neighbour_triangles;// to move odd_vertices

    map<unsigned int, vector <unsigned int>> odd_n;
    vector <Uvec2>::iterator it;
    int counter = vertices.size();
    int len_vertices = counter;
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


    //move odd
    coeffs.resize(counter);
    vector<Vertex> odd_vertexPositions;
    unsigned int len = counter - len_vertices;
    unsigned int triangle1[3];
    unsigned int triangle2[3];
    unsigned int A, B, C, D;

    for(unsigned int i = 0; i < len; i++) {
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
            odd_vertexPositions.push_back((vertices[A] + vertices[D])/8.f +(3.f/8.f)*(vertices[B] + vertices[C]));
            addCoeff( len_vertices + i, {A, 1.f/8.f});
            addCoeff( len_vertices + i, {D, 1.f/8.f});
            addCoeff( len_vertices + i, {B, 3.f/8.f});
            addCoeff( len_vertices + i, {C, 3.f/8.f});
        } else {
            odd_vertexPositions.push_back((vertices[neighbours[i][0]] + vertices[neighbours[i][1]])/2.f);
            addCoeff(len_vertices + i, {neighbours[i][0], 1.f/2.f});
            addCoeff(len_vertices + i, {neighbours[i][1], 1.f/2.f});
        }

    }

    newCoeffs.clear();
    newCoeffs.resize(len_vertices);
    // move even
    int n;
    Vertex res;
    len = vertices.size();
    float alphan = 3.f/8.f;
    float alpha3 = 3.f/16.f;
    for (unsigned int i = 0; i<len ; i++) {
        n = odd_n[i].size();
        if (n == 2) {
            vertices[i] = 1.f/8.f*(odd_vertexPositions[odd_n[i][0]] + odd_vertexPositions[odd_n[i][1]]) + 3.f/4.f*vertices[i];
            addNewCoeff(i, {len_vertices + odd_n[i][0], 1.f/8.f});
            addNewCoeff(i, {len_vertices + odd_n[i][1], 1.f/8.f});
            addNewCoeff(i, {i, 3.f/4.f});

        } else if (n == 3) {
            res = Vertex(0.f,0.f,0.f);
            for(unsigned int j : odd_n[i]) {
                res += alpha3 *  odd_vertexPositions[j];
                addNewCoeff(i, {len_vertices + j, alpha3});
            }
            res += (1 - 3.f*alpha3) * vertices[i];
            addNewCoeff(i, {i,1 - 3.f*alpha3});
            vertices[i] = res;
        }
        else {
            res = Vertex(0.f,0.f,0.f);
            for(unsigned int j : odd_n[i]) {
                res += alphan/(float)n *  odd_vertexPositions[j];
                addNewCoeff(i, {len_vertices + j, alphan/(float)n});
            }
            res += (1-alphan) * vertices[i];
            addNewCoeff(i, {i,1 - alphan});
            vertices[i] = res;
        }
    }

    for(int i = 0; i < len_vertices; i++) {
        coeffs[i] = newCoeffs[i];
    }

    vertices.insert(vertices.end(), odd_vertexPositions.begin(), odd_vertexPositions.end());
    triangles = new_triangleIndices;
}

void Mesh::addNewCoeff(unsigned int vertex, coeff k) {
    for(auto const & it : coeffs[k.vertex]) {
        newCoeffs[vertex][it.first] += it.second * k.lambda;
    }
}


void Mesh::addCoeff(unsigned int vertex, coeff k) {
    for(auto const & it : coeffs[k.vertex]) {
        coeffs[vertex][it.first] += it.second * k.lambda;
    }
}

void Mesh::redisplay() {
    unsigned int n = vertices.size();
    Vertex tmp;
    for(unsigned int i = 0; i < n; i++) {
        tmp = {0, 0, 0};
        for (auto const & it : coeffs[i]) {
            tmp += it.second * basicVertices[it.first];
        }
        vertices[i] = tmp;
    }
}

void Mesh::basicDisplay() {
    vertices = basicVertices;
    triangles = basicTriangles;
    coeffs.resize(basicVertices.size());
    for(unsigned int i = 0; i<basicVertices.size(); i++)
        coeffs[i][i] = 1;
}
