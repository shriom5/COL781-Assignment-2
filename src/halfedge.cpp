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
            // std::cout<<curredge.x<<" "<<curredge.y<<std::endl;
        }
        else
        {
            ivec2 curredge;
            curredge.x = edge->vertexIndex;
            curredge.y = edge->next->vertexIndex;
            // edges.emplace_back(curredge);
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
            // std::cout<<triangle.x<<" "<<triangle.y<<" "<<triangle.z<<std::endl;
        }
    }
    // std::cout<<"the number of faces is "<<faces.size()<<std::endl;
}

void Mesh::viewMesh(COL781::Viewer::Viewer &viewer)
{
    this->getEdges();
    this->triangulate();

    // std::cout<<"checking original vertices"<<std::endl;
    // for(auto x:this->vertices)
    // {
    //     std::cout<<x.position.x<<" "<<x.position.y<<" "<<x.position.z<<std::endl;
    // }

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
    // std::cout<<"checking for vertices"<<std::endl;
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
            // std::cout<<renderVertices[count].x<<" "<<renderVertices[count].y<<" "<<renderVertices[count].z<<std::endl;
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

    // std::cout<<"edges"<<std::endl;
    // for(int i=0;i<numberofEdges;i++)
    // {
    //     std::cout<<renderEdges[i].x<<" "<<renderEdges[i].y<<std::endl;
    // }

    viewer.setMesh(totalVertices,numberOfTriangles,numberofEdges,renderVertices,renderTriangles,renderEdges,renderNormals);

    viewer.view();
    
}

void Mesh::viewMesh2(COL781::Viewer::Viewer &viewer)
{
    // std::cout<<"original vertices"<<std::endl;
    // for(auto x:this->vertices)
    // {
    //     std::cout<<x.position.x<<" "<<x.position.y<<" "<<x.position.z<<std::endl;
    // }

    this->getEdges();
    this->triangulate();

    int totalVertices=vertices.size(), numberOfTriangles=this->triangles.size(), numberofEdges=this->edges.size();

    vec3 renderVertices[totalVertices*sizeof(vec3)];
    vec3 renderNormals[totalVertices*sizeof(vec3)];
    ivec3 renderTriangles[numberOfTriangles*sizeof(ivec3)];
    ivec2 renderEdges[numberofEdges*sizeof(ivec2)];

    // std::cout<<totalVertices<<" "<<numberOfTriangles<<" "<<numberofEdges<<std::endl;

    int count=0;
    int triangleCount=0;
    for(auto x:this->triangles)
    {
        ivec3 currTriangle;
        currTriangle.x=x.x;
        currTriangle.y=x.y;
        currTriangle.z=x.z;
        renderTriangles[triangleCount]=currTriangle;
        // std::cout<<renderTriangles[triangleCount].x<<" "<<renderTriangles[triangleCount].y<<" "<<renderTriangles[triangleCount].z<<std::endl;
        triangleCount++;
    }

    //hashmap such that we only store one face normal for each vertex
    std::map<int,int> done;
    //make a vector for normals corresponding to each vertex
    std::vector<vec3> currNormals(totalVertices);

    int currFace=0;
    for(auto x:vertexPerFace)
    {
        for(auto y:x)
        {
            if(done.find(y)==done.end())
            {
                done[y]=count;
                currNormals[count]=faces[currFace].normal;
                count++;
            }
        }
        currFace++;
    }

    //sanity check
    // std::cout<<"the size of normals is "<<count<<std::endl;
    for(int i=0;i<totalVertices;i++)
    {
        if(done.find(i)==done.end())
        {
            std::cerr<<"Error in normal allotment\n";
            return;
        }
    }
    // std::cout<<"checking for vertices"<<std::endl;
    count=0;
    for(auto x:this->vertices)
    {
        renderVertices[count]=x.position;
        renderNormals[count]=currNormals[count];
        // std::cout<<renderVertices[count].x<<" "<<renderVertices[count].y<<" "<<renderVertices[count].z<<std::endl;
        count++;
    }
    for(int i=0;i<numberofEdges;i++)
    {
        ivec2 currEdge;
        currEdge.x=this->edges[i].x;
        currEdge.y=this->edges[i].y;
        renderEdges[i]=currEdge;
    }

    //Debug code
    // std::cout<<"debugging"<<std::endl;

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

/*
Mesh extrusion
*/

void Mesh::extrudeFace(int idx, float distance)
{
    std::vector<int> faceVertices = this->faces[idx].getFaceVertices();
    vec3 normal = this->faces[idx].normal;

    int n = faceVertices.size();
    // std::vector<MeshVertex> new_vertices(n);
    for(int i=0;i<n;i++)
    {
        vec3 currPosition = this->vertices[faceVertices[i]].position;
        vec3 normal = this->faces[idx].normal;
        MeshVertex curr =MeshVertex(nullptr,currPosition+distance*normal);
        this->vertices.emplace_back(curr);
    }

    int sz=faces.size();
    HalfEdge *currEdge = this->faces[idx].edge;

    for(int i=0;i<n;i++)
    {
        int l1=faceVertices[i], l2=faceVertices[(i+1)%n], l3=vertices.size()-n+(i+1)%n, l4=vertices.size()-n+i;
        HalfEdge *e1 = new HalfEdge(l1);
        HalfEdge *e2 = new HalfEdge(l2);
        HalfEdge *e3 = new HalfEdge(l3);
        HalfEdge *e4 = new HalfEdge(l4);

        e1->next=e2;
        e2->next=e3;
        e3->next=e4;
        e4->next=e1;
        
        vec3 sideDirection = this->vertices[l2].position-this->vertices[l1].position;
        vec3 sideNormal = cross(normalize(sideDirection),normal);

        MeshFace currFace = MeshFace(e1,sideNormal);
        this->faces.emplace_back(currFace);

        this->halfEdges.emplace_back(e1);
        this->halfEdges.emplace_back(e2);
        this->halfEdges.emplace_back(e3);
        this->halfEdges.emplace_back(e4);

        e1->face=&this->faces[this->faces.size()-1];
        e2->face=&this->faces[this->faces.size()-1];
        e3->face=&this->faces[this->faces.size()-1];
        e4->face=&this->faces[this->faces.size()-1];

        vertices[l3].edge=e3;

        //bottom twin e1
        e1->twin=currEdge->twin;
        currEdge->twin->twin=e1;
        currEdge=currEdge->next;

        //left right twins (e2 and e4)
        if(i>0)
        {
            e4->twin=faces[sz+i-1].edge->next;
            faces[sz+i-1].edge->next->twin=e4;
        }
        if(i==n-1)
        {
            e2->twin=faces[sz].edge->next->next->next;
            faces[sz].edge->next->next->next->twin=e2;
        }
    }

    // we now make the new extruded face
    HalfEdge* first=new HalfEdge(vertices.size()-n);
    HalfEdge* curr=first;
    for(int i=0;i<n;i++)
    {
        curr->twin=faces[sz+i].edge->next->next;
        faces[sz+i].edge->next->next->twin=curr;
        if(i<n-1)
        {
            curr->next=new HalfEdge(vertices.size()-n+i+1);
            curr=curr->next;
        }
        else
        {
            curr->next=first;
        }
    }
    // //assign new half edge to the face
    faces[idx].edge=first;
}

void Mesh::extrudeFace(vec3 point, float distance)
{
    int idx=-1;
    float minDist=1e9;
    for(int i=0;i<faces.size();i++)
    {
        std::vector<int> faceVertices = faces[i].getFaceVertices();
        float currDist=1e9;
        for(int j=0;j<faceVertices.size()-2;j++)
        {
            vec3 a = this->vertices[faceVertices[0]].position;
            vec3 b = this->vertices[faceVertices[j+1]].position;
            vec3 c = this->vertices[faceVertices[j+2]].position;
            currDist=min(currDist,pointToTriangleDistance(point,a,b,c));
        }
        if(currDist<minDist)
        {
            minDist=currDist;
            idx=i;
        }
    }
    if(idx==-1)
    {
        std::cerr<<"No face found\n";
        return;
    }
    std::cout<<"the closest face has index "<<idx<<std::endl;
    this->extrudeFace(idx,distance);
}

/* The below functions are AI generated*/

float pointToSegmentDistance(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b) {
    glm::vec3 ab = b - a;
    glm::vec3 ap = p - a;
    float t = glm::dot(ap, ab) / glm::dot(ab, ab);
    t = clamp(t, 0.0f, 1.0f);  // Clamp projection to segment
    glm::vec3 closest = a + t * ab;
    return glm::length(p - closest);
}

float pointToTriangleDistance(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
    // Compute triangle normal
    glm::vec3 normal = glm::normalize(glm::cross(b - a, c - a));

    // Compute perpendicular projection onto triangle plane
    float d = glm::dot(normal, a);
    float signedDist = glm::dot(normal, p) - d;
    glm::vec3 projectedPoint = p - signedDist * normal;

    // Check if projected point is inside the triangle using Barycentric coordinates
    glm::vec3 v0 = b - a, v1 = c - a, v2 = projectedPoint - a;
    float d00 = glm::dot(v0, v0);
    float d01 = glm::dot(v0, v1);
    float d11 = glm::dot(v1, v1);
    float d20 = glm::dot(v2, v0);
    float d21 = glm::dot(v2, v1);
    float denom = d00 * d11 - d01 * d01;

    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    float u = 1.0f - v - w;

    if (u >= 0.0f && v >= 0.0f && w >= 0.0f) {
        // Projection falls inside the triangle
        return std::abs(signedDist);
    }

    // Otherwise, find the minimum distance to edges and vertices
    float edgeDist1 = pointToSegmentDistance(p, a, b);
    float edgeDist2 = pointToSegmentDistance(p, b, c);
    float edgeDist3 = pointToSegmentDistance(p, c, a);

    return std::min({ edgeDist1, edgeDist2, edgeDist3 });
}