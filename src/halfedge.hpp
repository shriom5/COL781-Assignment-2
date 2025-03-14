//Half edge data structure
#include "viewer.hpp"
#include <iostream>
#include <glm/glm.hpp>
#include <vector>
#include <map>
#include <fstream>
#include <string>

using namespace glm;

class HalfEdge{
    public:
        HalfEdge *next;
        HalfEdge *twin;
        int vertexIndex;
};

class MeshVertex{
    public:
        HalfEdge *edge;
};

class MeshFace{
    public:
        HalfEdge *edge;
        vec3 normal;
};

class Mesh{
    public:
    std::vector<HalfEdge> halfEdges;
    std::vector<MeshVertex> vertices;
    std::vector<MeshFace> faces;

    void getEdges();
    void triangulate();
    void viewMesh(COL781::Viewer::Viewer &viewer);
};
