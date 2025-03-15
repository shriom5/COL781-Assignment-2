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
            e4->twin=faces[i-1].edge->next;
        }
        if(x>0)
        {
            faces[i-n].edge->next->next->twin=e1;
            e1->twin=faces[i-n].edge->next->next;
        }
        halfEdges.emplace_back(e1);
        halfEdges.emplace_back(e2);
        halfEdges.emplace_back(e3);
        halfEdges.emplace_back(e4);
    }

    return Mesh(halfEdges,vertices,faces);
}

int main() {
    //Create a unit square mesh
    Mesh check=unitSquare(25,25);

    // check.triangulate();

    V::Viewer v;
    if (!v.initialize("Mesh viewer", 640, 480)) {
        return EXIT_FAILURE;
    }

    check.viewMesh2(v);

    //Memory cleanup

    for(int i=0;i<(int)check.halfEdges.size();i++)
    {
        delete(check.halfEdges[i]);
    }
}
