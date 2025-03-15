#include "viewer.hpp"
#include "halfedge.hpp"

namespace V = COL781::Viewer;
using namespace glm;

vec3 euclideanPos(vec3 polar_position)
{
    polar_position.y=radians(polar_position.y);
    polar_position.z=radians(polar_position.z);
    return vec3(
        polar_position.x * sin(polar_position.y) * cos(polar_position.z),
        polar_position.x * sin(polar_position.y) * sin(polar_position.z),
        polar_position.x * cos(polar_position.y)
    );
}

Mesh unitSphere(int m, int n)
{
    std::vector<MeshVertex> vertices;
    std::vector<MeshFace> faces;
    std::vector<HalfEdge*> halfEdges;

    float alpha = float(180)/float(n);
    float beta = float(360)/float(m);

    //quadrilateral faces
    for(int i=0;i<n-1;i++)
    {
        // std::cout<<"printing positions "<<std::endl;
        for(int j=0;j<m;j++)
        {
            float theta = 180-alpha*(i+1);
            float phi = beta*j;
            vec3 position = euclideanPos(vec3(1.0,theta,phi));
            // std::cout<<position.x<<" "<<position.y<<" "<<position.z<<std::endl;
            vertices.emplace_back(MeshVertex(nullptr,position));
        }
        // std::cout<<"done printing positions"<<std::endl;
    }
    for(int i=0;i<(n-2);i++)
    {
        for(int j=0;j<m;j++)
        {
            int l1,l2,l3,l4;
            if(j<m-1)
            {
                l1=i*m+j;l2=i*m+j+1;l3=l2+m;l4=l1+m;
            }
            else
            {
                l1=i*m+j;l2=i*m;l3=l2+m;l4=l1+m;
                // std::cout<<l1<<" "<<l2<<" "<<l3<<" "<<l4<<std::endl;
            }
            // std::cout<<l1<<" "<<l2<<" "<<l3<<" "<<l4<<std::endl;
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
            float theta = 180-alpha*(i+1)-alpha/(float)(2);
            float phi = beta*(j+0.5);
            vec3 currNormal=euclideanPos(vec3(1.0,theta,phi));
            currNormal=normalize(currNormal);
            MeshFace currFace = MeshFace(e1,currNormal);
            faces.emplace_back(currFace);
            halfEdges.emplace_back(e1);
            halfEdges.emplace_back(e2);
            halfEdges.emplace_back(e3);
            halfEdges.emplace_back(e4);
            if(j>0)
            {
                faces[i*m+j-1].edge->next->twin=e4;
                e4->twin=faces[i*m+j-1].edge->next;
            }
            if(i>0)
            {
                // std::cout<<"here"<<std::endl;
                faces[(i-1)*(m)+j].edge->next->next->twin=e1;
                e1->twin=faces[(i-1)*m+j].edge->next->next;
            }
        }
        faces[i*m].edge->next->next->next->twin=faces[i*m+m-1].edge->next;
        faces[i*m+m-1].edge->next->twin=faces[i*m].edge->next->next->next;
    }

    //pole
    vec3 position = vec3(0.0,0.0,-1.0);
    vertices.emplace_back(MeshVertex(nullptr,position));
    position = vec3(0.0,0.0,1.0);
    vertices.emplace_back(MeshVertex(nullptr,position));

    int sz=faces.size();

    for(int i=0;i<m;i++)
    {
        //bottom pole
        int l1,l2,l3;
        if(i<m-1)
        {
            l1=i;l2=i+1;l3=vertices.size()-2;
        }
        else
        {
            l1=i;l2=0;l3=vertices.size()-2;
        }
        HalfEdge *e1 = new HalfEdge(l1);
        HalfEdge *e2 = new HalfEdge(l2);
        HalfEdge *e3 = new HalfEdge(l3);
        e2->next=e1;
        e1->next=e3;
        e3->next=e2;
        vertices[l3].edge=e3;
        float theta = 180 - alpha/(float)(2);
        float phi = beta*(i+0.5);
        vec3 currNormal=euclideanPos(vec3(1.0,theta,phi));
        currNormal=normalize(currNormal);
        MeshFace currFace = MeshFace(e1,currNormal);
        faces.emplace_back(currFace);
        halfEdges.emplace_back(e1);
        halfEdges.emplace_back(e2);
        halfEdges.emplace_back(e3);
        if(i>0)
        {
            faces[sz+i-1].edge->next->twin=e1;
            e3->twin=faces[sz+i-1].edge->next;
        }
        if(i==m-1)
        {
            faces[sz].edge->twin=e3;
            e3->twin=faces[sz].edge;
        }
        if(n>2)
        {
            faces[i].edge->twin=e2;
            e2->twin=faces[i].edge;
        }
    }
    sz=faces.size();
    for(int i=0;i<m;i++)
    {
        //top pole
        int l1,l2,l3;
        if(i<m-1)
        {
            l1=(n-2)*m+i;l2=l1+1;l3=vertices.size()-1;
        }
        else
        {
            l1=(n-2)*m+i;l2=(n-2)*m;l3=vertices.size()-1;
        }
        HalfEdge *e1 = new HalfEdge(l1);
        HalfEdge *e2 = new HalfEdge(l2);
        HalfEdge *e3 = new HalfEdge(l3);
        e1->next=e2;
        e2->next=e3;
        e3->next=e1;
        vertices[l3].edge=e3;
        float theta = alpha/(float)(2);
        float phi = beta*(i+0.5);
        vec3 currNormal=euclideanPos(vec3(1.0,theta,phi));
        currNormal=normalize(currNormal);
        MeshFace currFace = MeshFace(e1,currNormal);
        faces.emplace_back(currFace);
        halfEdges.emplace_back(e1);
        halfEdges.emplace_back(e2);
        halfEdges.emplace_back(e3);
        if(i>0)
        {
            faces[sz+i-1].edge->next->twin=e3;
            e3->twin=faces[sz+i-1].edge->next;
        }
        if(i==m-1)
        {
            faces[sz].edge->next->next->twin=e2;
            e2->twin=faces[sz].edge->next->next;
        }
        if(n>2)
        {
            faces[(n-3)*m+i].edge->next->next->twin=e1;
            e1->twin=faces[(n-3)*m+i].edge->next->next;
        }
        else
        {
            if(n<2)
            {
                std::cout<<"Need at least two stacks"<<std::endl;
            }
            else
            {
                faces[i].edge->next->next->twin=e1;
                e1->twin=faces[i].edge->next->next;
            }
        }
    }

    return Mesh(halfEdges,vertices,faces);
}

int main() {
    //Create a unit square mesh
    Mesh check=unitSphere(20,20);

    // check.triangulate();

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
