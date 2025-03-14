#include "viewer.hpp"
#include "halfedge.hpp"

namespace V = COL781::Viewer;
using namespace glm;

Mesh unitCube(int m, int n, int o)
{
    //unit cube with m,n,o divisions across x,y,z axis
    std::vector<MeshVertex> vertices;
    std::vector<MeshFace> faces;
    std::vector<HalfEdge*> halfEdges;
    //faces except those along z axis

    //vertices  
    for(int i=0;i<(o+1);i++)
    {
        vec3 position = vec3(0.0,0.0,(float)i/(float)o);
        for(int j=0;j<2*m+2*n;j++)
        {
            vertices.emplace_back(MeshVertex(nullptr,position));
            if(j<m) position.x+=1.0/(float)m;
            else if(j<m+n) position.y+=1.0/(float)n;
            else if(j<2*m+n) position.x-=1.0/(float)m;
            else position.y-=1.0/(float)n;
            // std::cout<<position.x<<" "<<position.y<<" "<<position.z<<std::endl;
        }
    }
    //faces
    for(int i=0;i<o;i++)
    {
        for(int j=0;j<2*m+2*n;j++)
        {
            int l1=i*(2*m+2*n)+j,l2=l1+1,l4=l1+(2*m+2*n),l3=l4+1;
            if(j==(2*m+2*n-1)) {l2=i*(2*m+2*n);l3=l2+(2*m+2*n);}
            HalfEdge *e1 = new HalfEdge(l1);
            HalfEdge *e2 = new HalfEdge(l2);
            HalfEdge *e3 = new HalfEdge(l3);
            HalfEdge *e4 = new HalfEdge(l4);
            e1->next=e2;
            e2->next=e3;
            e3->next=e4;
            e4->next=e1;
            vertices[l1].edge=e1;
            vertices[l2].edge=e2;
            vertices[l3].edge=e3;
            vertices[l4].edge=e4;
            halfEdges.emplace_back(e1);
            halfEdges.emplace_back(e2);
            halfEdges.emplace_back(e3);
            halfEdges.emplace_back(e4);
            
            MeshFace currFace = MeshFace(e1,vec3(0.0,0.0,1.0));
            if(j<m) currFace.normal=vec3(0.0,-1.0,0.0);
            else if (j<m+n) currFace.normal=vec3(1.0,0.0,0.0);
            else if (j<2*m+n) currFace.normal=vec3(0.0,1.0,0.0);
            else currFace.normal=vec3(-1.0,0.0,0.0);
            faces.emplace_back(currFace);
            if(i>0)
            {
                faces[(i-1)*(2*m+2*n)+j].edge->next->next->twin=e1;
                e1->twin=faces[(i-1)*(2*m+2*n)+j].edge->next->next;
            }
            if(j>0)
            {
                faces[i*(2*m+2*n)+j-1].edge->next->twin=e4;
                e4->twin=faces[i*(2*m+2*n)+j-1].edge->next;
            }
        }
        faces[i*(2*m+2*n)].edge->next->next->next->twin=faces[i*(2*m+2*n)+(2*m+2*n)-1].edge->next;
        faces[i*(2*m+2*n)+(2*m+2*n)-1].edge->next->twin=faces[i*(2*m+2*n)].edge->next->next->next;
    }

    //bottom face
    int sz=vertices.size();
    vec3 position = vec3(0.5,0.5,0.0);
    for(int i=0;i<n-1;i++)
    {
        for(int j=0;j<m-1;j++)
        {
            vertices.emplace_back(MeshVertex(nullptr,position));
            position.x+=1.0/(float)m;
        }
        position.y+=1.0/(float)n;
    }
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<m;j++)
        {
            int l1,l2,l3,l4;
            if(i==0)
            {
                l1=j;l2=j+1;
            }
            else if (i==n-1)
            {
                l3=2*m+n-j-1;l4=2*m+n-j;
            }
            else
            {
                l1=sz+(i-1)*m+j;
            }
        }
    }



    return Mesh(halfEdges,vertices,faces);
}

int main() {
    //Create a unit square mesh
    Mesh check=unitCube(1,1,1);

    check.triangulate();

    V::Viewer v;
    if (!v.initialize("Mesh viewer", 640, 480)) {
        return EXIT_FAILURE;
    }

    check.viewMesh(v);

    //Memory cleanup

    for(int i=0;i<(int)check.halfEdges.size();i++)
    {
        delete(check.halfEdges[i]);
    }
}
