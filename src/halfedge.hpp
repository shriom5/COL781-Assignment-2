//Half edge data structure
#include "viewer.hpp"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/polar_coordinates.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/epsilon.hpp>
#include <vector>
#include <map>
#include <fstream>
#include <string>
#include <algorithm>

using namespace glm;

class HalfEdge;
class MeshFace;
class MeshVertex;

class HalfEdge{
    public:
        HalfEdge *next;
        HalfEdge *twin;
        int vertexIndex;
        MeshFace* face;
        HalfEdge(int);
};

class MeshFace{
    public:
        HalfEdge *edge;
        vec3 normal;
        MeshFace(HalfEdge*,vec3);
        std::vector<int> getFaceVertices();
};

class MeshVertex{
    public:
        HalfEdge *edge;
        vec3 position;
        //boundary check
        MeshVertex(HalfEdge*,vec3);
        std::vector<HalfEdge*> getAdjacentFaces();
};

class Mesh{
    public:
    std::vector<HalfEdge*> halfEdges;
    std::vector<MeshVertex> vertices;
    std::vector<MeshFace> faces;

    std::vector<ivec3> triangles;
    std::vector<ivec2> edges;
    std::vector<vec3> normals;
    std::vector<std::vector<int>> vertexPerFace;

    Mesh(std::vector<HalfEdge*>, std::vector<MeshVertex>, std::vector<MeshFace>);
    void getEdges();
    void triangulate();
    void viewMesh(COL781::Viewer::Viewer &viewer);
    void viewMesh2(COL781::Viewer::Viewer &viewer);
    void extrudeFace(int faceIndex, float distance);
    void extrudeFace(vec3 point, float distance);
};


float pointToSegmentDistance(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b);
float pointToTriangleDistance(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);