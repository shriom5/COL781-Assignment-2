#include "viewer.hpp"
#include "halfedge.hpp"

namespace V = COL781::Viewer;
using namespace glm;

Mesh unitSquare(int m, int n)
{
    std::vector<MeshVertex> vertices;
    std::vector<MeshFace> faces;
    std::vector<HalfEdge*> halfEdges;

    for(int i=0;i<(m+1)*(n+1);i++)
    {
        int x = i/(n+1), y = i%(n+1);
        float x_cord = (float)y/(float)n;
        float y_cord = (float)x/(float)m;
        vertices.emplace_back(MeshVertex(nullptr,vec3(x_cord,y_cord,0.0)));
    }

    for(int i=0;i<m*n;i++)
    {
        int x = i/n, y = i%n;

        int l1=x*(n+1)+y, l2=x*(n+1)+y+1, l3=l2+(n+1), l4=l1+(n+1);

        HalfEdge * e1=new HalfEdge(l1);
        HalfEdge * e2=new HalfEdge(l2);
        HalfEdge * e3=new HalfEdge(l3);
        HalfEdge * e4=new HalfEdge(l4);

        vertices[l1].edge=e1;
        vertices[l2].edge=e2;
        vertices[l3].edge=e3;
        vertices[l4].edge=e4;

        e1->next=e2;
        e2->next=e3;
        e3->next=e4;
        e4->next=e1;
        MeshFace currFace = MeshFace(e1,vec3(0.0,0.0,1.0));
        faces.emplace_back(currFace);
        if(y>0)
        {
            faces[i-1].edge->next->twin=e4;
        }
        if(x>0)
        {
            faces[i-n].edge->next->next->twin=e1;
        }
        halfEdges.emplace_back(e1);
        halfEdges.emplace_back(e2);
        halfEdges.emplace_back(e3);
        halfEdges.emplace_back(e4);
    }

    return Mesh(halfEdges,vertices,faces);
}

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

    // MeshFace face1 = MeshFace(edge1, vec3(0.0,0.0,1.0));
    std::vector<MeshVertex> mvs{MeshVertex(edge1,vertexes[0]),MeshVertex(edge2,vertexes[1]),MeshVertex(edge3,vertexes[2]),MeshVertex(edge4,vertexes[3])};

    Mesh check=unitSquare(20,10);

    check.triangulate();

    V::Viewer v;
    if (!v.initialize("Mesh viewer", 640, 480)) {
        return EXIT_FAILURE;
    }

    check.viewMesh(v);

    v.setMesh(4, 2, 4, vertices, triangles, edges, normals);
    v.view();

    //Memory cleanup
    delete(edge1);
    delete(edge2);
    delete(edge3);
    delete(edge4);

    for(int i=0;i<(int)check.halfEdges.size();i++)
    {
        delete(check.halfEdges[i]);
    }
}
