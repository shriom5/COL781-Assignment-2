#include "viewer.hpp"
#include "halfedge.hpp"

namespace V = COL781::Viewer;
using namespace glm;

int main() {

    vec3 vertices[] = {
        vec3(0.0, 0.0, 0.0),
        vec3(0.5, 0.0, 0.0),
        vec3(0.5, 0.5, 0.0),
        vec3(0.0, 0.5, 0.0),

    };

    vec3 normals[] = {
        vec3(0.0, 0.0, 1.0),
        vec3(0.0, 0.0, 1.0),
        vec3(0.0, 0.0, 1.0),
        vec3(0.0, 0.0, 1.0),
    };

    ivec3 triangles[] = {
        ivec3(0, 1, 2),
        ivec3(0, 2, 3),
    };

    ivec2 edges[] = {
        ivec2(0, 1),
        ivec2(1, 2),
        ivec2(3, 2),
        ivec2(3, 0)
    };

    std::vector<vec3> vertexes{vec3(0.0,0.0,0.0),vec3(0.5,0.0,0.0),vec3(0.5,0.5,0.0),vec3(0.0,0.5,0.0)};
    HalfEdge *edge1 = new HalfEdge(0);
    HalfEdge *edge2 = new HalfEdge(1);
    HalfEdge *edge3 = new HalfEdge(2);
    HalfEdge *edge4 = new HalfEdge(3);

    std::vector<HalfEdge*> hes{edge1, edge2, edge3, edge4};

    edge1->next = edge2;
    edge2->next = edge3;
    edge3->next = edge4;
    edge4->next = edge1;

    MeshFace face1 = MeshFace(edge1, vec3(0.0,0.0,1.0));
    std::vector<MeshVertex> mvs{MeshVertex(edge1,vertexes[0]),MeshVertex(edge2,vertexes[1]),MeshVertex(edge3,vertexes[2]),MeshVertex(edge4,vertexes[3])};

    Mesh check = Mesh(hes,mvs,{face1});

    check.triangulate();

    V::Viewer v;
    if (!v.initialize("Mesh viewer", 640, 480)) {
        return EXIT_FAILURE;
    }

    check.viewMesh(v);

    // v.setMesh(4, 2, 4, vertices, triangles, edges, normals);
    // v.view();


    delete(edge1);
    delete(edge2);
    delete(edge3);
    delete(edge4);
}
