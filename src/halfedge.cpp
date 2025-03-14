#include "halfedge.hpp"

/*
Functions for half edge data structure
*/

HalfEdge::HalfEdge(int index)
{
    this->next=NULL;
    this->twin=NULL;
    this->vertexIndex=index;
    this->face=NULL;
}

/*
Functions for MeshFace
*/

MeshFace::MeshFace(HalfEdge *edge, vec3 normal)
{
    this->edge=edge;
    this->normal=normal;
}

std::vector<int> MeshFace::getFaceVertices()
{
    std::vector<int> faceVertices;
    HalfEdge *start = this->edge;
    HalfEdge *current = start;

    do
    {
        faceVertices.emplace_back(current->vertexIndex);
        current=current->next;
    } while (current!=start);

    return faceVertices;
}

/*
Functions for MeshVertex
*/

MeshVertex::MeshVertex(HalfEdge *edge, vec3 position)
{
    this->edge=edge;
    this->position=position;
}

std::vector<HalfEdge*> MeshVertex::getAdjacentFaces()
{
    std::vector<HalfEdge*> adjacentFaces;
    HalfEdge *start = this->edge;
    HalfEdge *current = start;

    do
    {
        adjacentFaces.emplace_back(current);
        if(current->twin==NULL)
        {
            //boundary face encountered
            break;
        }
        else
        {
            current = current->twin->next;
        }

    }while(current!=start);

    return adjacentFaces;
}

/*
Functions for Mesh
*/

Mesh::Mesh(std::vector<HalfEdge*> halfEdges, std::vector<MeshVertex> vertices, std::vector<MeshFace> faces)
{
    this->halfEdges=halfEdges;
    this->vertices=vertices;
    this->faces=faces;
}

void Mesh::getEdges()
{
    edges.clear();
    for(auto &edge:halfEdges)
    {
        if(edge->twin)
        {
            ivec2 curredge;
            curredge.x = edge->vertexIndex;
            curredge.y = edge->twin->vertexIndex;
            edges.emplace_back(curredge);
        }
        else
        {
            ivec2 curredge;
            curredge.x = edge->vertexIndex;
            curredge.y = edge->next->vertexIndex;
            edges.emplace_back(curredge);
        }
    }
}

void Mesh::triangulate()
{
    triangles.clear();
    normals.clear();
    vertexPerFace.clear();
    for(auto face:faces)
    {
        std::vector<int> faceVertices = face.getFaceVertices();
        vertexPerFace.emplace_back(faceVertices);
        normals.emplace_back(face.normal);
        int n = faceVertices.size();
        for(int i=0;i<n-2;i++)
        {
            ivec3 triangle;
            triangle.x = faceVertices[0];
            triangle.y = faceVertices[i+1];
            triangle.z = faceVertices[i+2];
            triangles.emplace_back(triangle);
        }
    }
}

void Mesh::viewMesh(COL781::Viewer::Viewer &viewer)
{
    this->getEdges();
    this->triangulate();

    int totalVertices=0, numberOfTriangles=this->triangles.size(), numberofEdges=this->edges.size();
    for(auto const x:vertexPerFace)
    {
        totalVertices+=x.size();
    }

    // std::cout<<totalVertices<<std::endl;

    std::map<int,int> alias;
    int count=0;
    for(auto const x:vertexPerFace)
    {
        for(auto const y:x)
        {
            if(alias.find(y)==alias.end())
            {
                alias[y]=count;
            }
            count++;
        }
    }

    // for(auto x:alias)
    // {
    //     std::cout<<x.first<<" "<<x.second<<std::endl;
    // }

    if(count!=totalVertices)
    {
        std::cerr<<"Error in traingulating\n";
        return;
    }

    vec3 renderVertices[totalVertices*sizeof(vec3)];
    vec3 renderNormals[totalVertices*sizeof(vec3)];
    ivec3 renderTriangles[numberOfTriangles*sizeof(ivec3)];
    ivec2 renderEdges[numberofEdges*sizeof(ivec2)];

    count=0;
    int faceCount=0,triangleCount=0;
    for(auto const x:vertexPerFace)
    {
        vec3 normal = this->normals[faceCount];
        int n = x.size();
        for(int i=0;i<n-2;i++)
        {
            ivec3 currTriangle;
            currTriangle.x=count;
            currTriangle.y=count+i+1;
            currTriangle.z=count+i+2;
            renderTriangles[triangleCount]=currTriangle;
            triangleCount++;
        }
        for(auto const y:x)
        {
            renderVertices[count]=this->vertices[y].position;
            renderNormals[count]=normal;
            count++;
        }
        faceCount++;
    }
    for(int i=0;i<numberofEdges;i++)
    {
        ivec2 currEdge;
        currEdge.x=alias[this->edges[i].x];
        currEdge.y=alias[this->edges[i].y];
        renderEdges[i]=currEdge;
    }

    //Debug code

    // std::cout<<totalVertices<<" "<<numberOfTriangles<<" "<<numberofEdges<<std::endl;

    // std::cout<<"vertices"<<std::endl;

    // for(int i=0;i<totalVertices;i++)
    // {
    //     std::cout<<renderVertices[i].x<<" "<<renderVertices[i].y<<" "<<renderVertices[i].z<<std::endl;
    // }

    // std::cout<<"triangles"<<std::endl;

    // for(int i=0;i<numberOfTriangles;i++)
    // {
    //     std::cout<<renderTriangles[i].x<<" "<<renderTriangles[i].y<<" "<<renderTriangles[i].z<<std::endl;
    // }

    // std::cout<<"normals"<<std::endl;

    // for(int i=0;i<totalVertices;i++)
    // {
    //     std::cout<<renderNormals[i].x<<" "<<renderNormals[i].y<<" "<<renderNormals[i].z<<std::endl;
    // }

    viewer.setMesh(totalVertices,numberOfTriangles,numberofEdges,renderVertices,renderTriangles,renderEdges,renderNormals);

    viewer.view();
    
}