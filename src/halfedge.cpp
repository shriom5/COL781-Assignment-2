#include "halfedge.hpp"

/*
Functions for half edge data structure
*/

#define _AREA_WEIGHTED_

HalfEdge::HalfEdge(int index)
{
    this->next=nullptr;
    this->twin=nullptr;
    this->vertexIndex=index;
    this->face=nullptr;
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

std::vector<HalfEdge *> MeshFace::getFaceEdges()
{
    std::vector<HalfEdge *> faceVertices;
    HalfEdge *start = this->edge;
    HalfEdge *current = start;

    do
    {
        faceVertices.emplace_back(current);
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

    if(!current)
    {
        std::cout<<"No edge associated with the vertex"<<std::endl;
    }

    do
    {
        adjacentFaces.emplace_back(current);
        if(current->twin==NULL)
        {
            //boundary face encountered
            std::cout<<"some issue with the code please check"<<std::endl;
            break;
        }
        else
        {
            current = current->twin->next;
        }

    }while(current!=start);
    return adjacentFaces;
}

std::vector<int> MeshVertex::getAdjacentVertices()
{
    std::vector<int> adjacentVertices;
    HalfEdge *start = this->edge;
    HalfEdge *current = start;

    if(!current)
    {
        std::cout<<"No edge associated with the vertex"<<std::endl;
    }

    do
    {
        if(current->twin==NULL)
        {
            //boundary face encountered
            std::cout<<"some issue with the code please check"<<std::endl;
            break;
        }
        else
        {
            adjacentVertices.emplace_back(current->twin->vertexIndex);
            current = current->twin->next;
        }

    }while(current!=start);
    return adjacentVertices;   
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
            if(curredge.x!=curredge.y) edges.emplace_back(curredge);
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
        }
    }
}

void Mesh::viewMesh(COL781::Viewer::Viewer &viewer)
{
    this->getEdges();
    this->triangulate();

    for(int i = 0; i < faces.size(); ++i) {
        faces[i].normal = getFaceNormal(i);
    }

    int totalVertices=0, numberOfTriangles=this->triangles.size(), numberofEdges=this->edges.size();
    for(auto const x:vertexPerFace)
    {
        totalVertices+=x.size();
    }

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

    if(count!=totalVertices)
    {
        std::cerr<<"Error in traingulating\n";
        return;
    }

    vec3 renderVertices[totalVertices*sizeof(vec3)];
    vec3 renderNormals[totalVertices*sizeof(vec3)];
    ivec3 renderTriangles[numberOfTriangles*sizeof(ivec3)];
    ivec2 renderEdges[numberofEdges*sizeof(ivec2)];

    #ifdef _AREA_WEIGHTED_
    std::map<int, vec3> wtnormals;
    for(int i = 0; i < this->faces.size(); ++i) {
        std::vector<int> verts = this->faces[i].getFaceVertices();
        int vsz = verts.size();
        for(int j = 0; j < vsz; ++j) {
            vec3 vi = this->vertices[verts[j]].position - this->vertices[verts[(j + 1) % vsz]].position;
            vec3 vin = this->vertices[verts[(j + 2) % vsz]].position - this->vertices[verts[(j + 1) % vsz]].position;
            float denominator = length(vi) * length(vin);
            denominator = denominator * denominator;
            wtnormals[verts[(j + 1) % vsz]] += cross(vin, vi) / denominator;
        }
    }
    #else
    std::map<int, std::vector<int>> mp;
    for(int i = 0; i < this->faces.size(); ++i) {
        for(auto p: this->faces[i].getFaceVertices()) {
            mp[p].emplace_back(i);
        }
    }
    #endif

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
            // renderNormals[count]=normal;
            #ifdef _AREA_WEIGHTED_
            renderNormals[count] = wtnormals[count];
            #else 
            renderNormals[count] = getVertexNormal(mp[count]);
            #endif
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

    viewer.setMesh(totalVertices,numberOfTriangles,numberofEdges,renderVertices,renderTriangles,renderEdges,renderNormals);

    viewer.view();
    
}

void Mesh::viewMesh2(COL781::Viewer::Viewer &viewer)
{
    this->getEdges();
    this->triangulate();

    for(int i = 0; i < faces.size(); ++i) {
        this->faces[i].normal = getFaceNormal(i);
    }

    int totalVertices=vertices.size(), numberOfTriangles=this->triangles.size(), numberofEdges=this->edges.size();

    vec3 renderVertices[totalVertices*sizeof(vec3)];
    vec3 renderNormals[totalVertices*sizeof(vec3)];
    ivec3 renderTriangles[numberOfTriangles*sizeof(ivec3)];
    ivec2 renderEdges[numberofEdges*sizeof(ivec2)];

    int count=0;
    int triangleCount=0;
    for(auto x:this->triangles)
    {
        ivec3 currTriangle;
        currTriangle.x=x.x;
        currTriangle.y=x.y;
        currTriangle.z=x.z;
        renderTriangles[triangleCount]=currTriangle;
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
    // for(int i=0;i<totalVertices;i++)
    // {
    //     if(done.find(i)==done.end())
    //     {
    //         std::cerr<<"Error in normal allotment\n";
    //         // return;
    //     }
    // }

    std::map<int, std::vector<int>> mp;

    for(int i = 0; i < this->faces.size(); ++i) {
        for(auto p: this->faces[i].getFaceVertices()) {
            mp[p].emplace_back(i);
        }
    }

    #ifdef _AREA_WEIGHTED_
    std::map<int, vec3> wtnormals;
    for(int i = 0; i < this->faces.size(); ++i) {
        std::vector<int> verts = this->faces[i].getFaceVertices();
        int vsz = verts.size();
        for(int j = 0; j < vsz; ++j) {
            vec3 vi = this->vertices[verts[j]].position - this->vertices[verts[(j + 1) % vsz]].position;
            vec3 vin = this->vertices[verts[(j + 2) % vsz]].position - this->vertices[verts[(j + 1) % vsz]].position;
            float denominator = length(vi) * length(vin);
            denominator = denominator * denominator;
            wtnormals[verts[(j + 1) % vsz]] += cross(vin, vi) / denominator;
        }
    }
    #else
    std::map<int, std::vector<int>> mp;
    for(int i = 0; i < this->faces.size(); ++i) {
        for(auto p: this->faces[i].getFaceVertices()) {
            mp[p].emplace_back(i);
        }
    }
    #endif

    count=0;
    for(auto x:this->vertices)
    {
        renderVertices[count]=x.position;
        #ifdef _AREA_WEIGHTED_
        renderNormals[count] = wtnormals[count];
        #else 
        renderNormals[count] = getVertexNormal(mp[count]);
        #endif
        count++;
    }
    for(int i=0;i<numberofEdges;i++)
    {
        ivec2 currEdge;
        currEdge.x=this->edges[i].x;
        currEdge.y=this->edges[i].y;
        renderEdges[i]=currEdge;
    }

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
    this->extrudeFace(idx,distance);
}

vec3 Mesh::getFaceNormal(int idx)
{
    std::vector<int> faceVertices = (this->faces[idx]).getFaceVertices();

    vec3 normal = vec3(0.0, 0.0, 0.0);
    for(int i = 1; i < faceVertices.size() - 1; ++i) {
        normal += (0.5f) * cross(
            this->vertices[faceVertices[i]].position - this->vertices[faceVertices[0]].position,
            this->vertices[faceVertices[i + 1]].position - this->vertices[faceVertices[0]].position
            );
    }

    normal = normalize(normal);
    return normal;

    // vec3 d1, d2;
    // HalfEdge *start = this->faces[idx].edge;
    // d1=this->vertices[start->next->vertexIndex].position-this->vertices[start->vertexIndex].position;
    // d2=this->vertices[start->next->next->vertexIndex].position-this->vertices[start->next->vertexIndex].position;
    // vec3 normal = cross(d1,d2);
    // return normal;
}

vec3 Mesh::getVertexNormal(std::vector<int> adj) {
    vec3 currnormal(0.0, 0.0, 0.0);
    for(auto x: adj) {
        currnormal += this->faces[x].normal;
    }
    currnormal = normalize(currnormal);
    return currnormal;
}

// vec3 Mesh::getVertexNormalWeighted(int vertIndex, std::vector<int> adj) {
//     vec3 normal(0.0, 0.0, 0.0);
//     vec3 vo = this->vertices[vertIndex].position;
//     for(auto face:adj) {
//         vec3 vto, vfrom;
//         HalfEdge* curr = this->faces[face].edge;
//         while(curr->next->vertexIndex != vertIndex) {
//             curr = curr->next;
//         }
//         vto = this->vertices[curr->vertexIndex].position;

//     }
// }

void Mesh::extrudeMultiple(std::vector<int> &indices, float dist)
{
    std::map<HalfEdge *,int> currEdges;
    std::map<int,std::vector<vec3>> faceNormals; 
    for(auto x:indices)
    {
        std::vector<HalfEdge*> faceEdges = this->faces[x].getFaceEdges();
        for(auto y:faceEdges)
        {
            currEdges[y]=1;
        }
        std::vector<int> faceVertices = this->faces[x].getFaceVertices();
        for(auto y:faceVertices)
        {
            faceNormals[y].emplace_back(getFaceNormal(x));
        }
    }
    std::map<int,vec3> newPositions;
    std::map<int,int> newIndices;
    std::map<int,vec3> newNormals;
    for(auto x:faceNormals)
    {
        vec3 normal(0.0f);
        for(auto y:x.second)
        {
            normal+=y;
        }
        normal=normalize(normal);
        newNormals[x.first]=normal;
        newPositions[x.first]=this->vertices[x.first].position+dist*normal;
        vertices.emplace_back(MeshVertex(nullptr,newPositions[x.first]));
        newIndices[x.first]=vertices.size()-1;
    }
    std::map<int,int> twinHelp;
    //make new quads
    for(auto x:currEdges)
    {
        HalfEdge *currEdge = x.first;
        //ignore the internal edges
        if(currEdges.find(currEdge->twin)!=currEdges.end()) continue;
        int l1=currEdge->vertexIndex, l2=currEdge->next->vertexIndex, l3=newIndices[l2], l4=newIndices[l1];
        HalfEdge *e1 = new HalfEdge(l1);
        HalfEdge *e2 = new HalfEdge(l2);
        HalfEdge *e3 = new HalfEdge(l3);
        HalfEdge *e4 = new HalfEdge(l4);

        e1->next=e2;
        e2->next=e3;
        e3->next=e4;
        e4->next=e1;

        this->halfEdges.emplace_back(e1);
        this->halfEdges.emplace_back(e2);
        this->halfEdges.emplace_back(e3);
        this->halfEdges.emplace_back(e4);

        //release the twin
        x.first->twin->twin=e1;
        e1->twin=x.first->twin;

        //attach
        e3->twin=x.first;
        x.first->twin=e3;

        vec3 d1=this->vertices[l2].position-this->vertices[l1].position;
        vec3 normal = cross(d1,newNormals[l1]);
        MeshFace currFace = MeshFace(e1,normal);
        this->faces.emplace_back(currFace);
        twinHelp[l1]=this->faces.size()-1;
    }
    // assign twins
    for(auto x:currEdges)
    {
        HalfEdge *currEdge = x.first;
        if(currEdges.find(currEdge->twin)==currEdges.end())
        {
            int f1 = twinHelp[currEdge->vertexIndex];
            int f2 = twinHelp[currEdge->next->vertexIndex];
            this->faces[f1].edge->next->twin=this->faces[f2].edge->next->next->next;
            this->faces[f2].edge->next->next->next->twin=this->faces[f1].edge->next;
        }
    }

    //make new top face
    for(auto x:indices)
    {
        std::vector<HalfEdge *> faceEdges = this->faces[x].getFaceEdges();
        for(auto y:faceEdges)
        {
            y->vertexIndex=newIndices[y->vertexIndex];
        }
    }
}

/*Addding random noise to the mesh*/
void Mesh::addNoise(float maxnoise)
{
    for(int i=0;i<vertices.size();i++)
    {
        vec3 noise = vec3(randomFloat(-maxnoise,maxnoise),randomFloat(-maxnoise,maxnoise),randomFloat(-maxnoise,maxnoise));
        vertices[i].position+=noise;
    }
}

/*Getting neighbors and performing smoothing by umbrella operator*/
void Mesh::printAdjacentVertices()
{
    for(int i=0;i<vertices.size();i++)
    {
        std::cout<<"Neighbors of vertex "<<i<<std::endl;
        vertices[i].getAdjacentFaces();
    }
}

void Mesh::umbrellaOperator(float lambda, int iterations)
{
    for(int i=0;i<iterations;i++)
    {
        int n=vertices.size();
        std::vector<vec3> deltas(n);
        for(int j=0;j<n;j++)
        {
            vec3 myPos=this->vertices[j].position;
            std::vector<int> nbr=this->vertices[j].getAdjacentVertices();
            vec3 sum=vec3(0.0f);
            for(auto x:nbr)
            {
                sum+=this->vertices[x].position-myPos;
            }
            float k=nbr.size();
            sum=(sum*lambda)/k;
            deltas[j]=sum;
        }
        for(int j=0;j<n;j++) this->vertices[j].position+=deltas[j];
    }
}

/*Catmull clark subdivision*/
void Mesh::catmullClarkSubdivision()
{
    std::map<int,std::vector<vec3>> facePoints;
    int numberofFaces=this->faces.size(), numberofVertices=this->vertices.size();
    std::vector<vec3> faceAvg(this->faces.size());
    for(int i=0;i<numberofFaces;i++)
    {
        //Compute the facepoint, the normals we will implement in the viewMesh code
        std::vector<int> currFaceVertices = this->faces[i].getFaceVertices();
        int numberOfVertices = currFaceVertices.size();
        vec3 facePoint(0.0f);
        for(int vertex:currFaceVertices) facePoint+=this->vertices[vertex].position;
        facePoint/=(float)numberOfVertices;

        for(auto x:currFaceVertices)
        {
            facePoints[x].push_back(facePoint);
        }
        faceAvg[i]=facePoint;
    }
    //We now compute the new vertex positions
    std::vector<vec3> newPos(numberofVertices);

    for(int i=0;i<numberofVertices;i++)
    {
        std::vector<int> neighborVertices = this->vertices[i].getAdjacentVertices();

        int n = neighborVertices.size();

        if(facePoints.find(i)==facePoints.end()) std::cout<<"Error in allotting facepoints"<<std::endl;

        if(n!=facePoints[i].size())
        {
            std::cout<<"Error in mesh connectivity"<<std::endl;
        }

        if(n==0)
        {
            std::cout<<"No faces adjacent to the vertex "<<i<<std::endl;
            return;
        }

        float m1=(float)(n-3)/(float)n;
        float m2=(float)(1)/float(n);
        float m3=(float)(2)/float(n);

        //calculate the average of midedges
        vec3 midEdge(0.0f);
        for(auto x:neighborVertices)
        {
            midEdge+=(this->vertices[i].position+this->vertices[x].position)/(float)2;
        }
        midEdge/=(float)n;
        //get the average of the adjacent facePoints
        vec3 adjFacepoints(0.0f);
        for(vec3 point:facePoints[i])
        {
            adjFacepoints+=point;
        }
        adjFacepoints/=(float)(n);
        
        newPos[i]=m1*this->vertices[i].position+m2*adjFacepoints+m3*midEdge;
    }

    for(int i=0;i<numberofVertices;i++) this->vertices[i].position=newPos[i];

    //Store the edge points for each halfEdge
    std::map<HalfEdge *,int> edgePoint;
    for(int i=0;i<this->halfEdges.size();i++)
    {
        vec3 p1 = this->vertices[this->halfEdges[i]->vertexIndex].position;
        if(this->halfEdges[i]->twin==nullptr)
        {
            std::cout<<"Error with the mesh"<<std::endl;
        }
        vec3 p2 = this->vertices[this->halfEdges[i]->twin->vertexIndex].position;
        if(this->halfEdges[i]->vertexIndex==this->halfEdges[i]->twin->vertexIndex)
        {
            std::cout<<"Error in twin allotment"<<std::endl;
        }
        p1+=p2;
        p1/=(float)2;
        if(edgePoint.find(this->halfEdges[i])==edgePoint.end())
        {
            MeshVertex ep = MeshVertex(nullptr,p1);
            this->vertices.emplace_back(ep);
            edgePoint[this->halfEdges[i]]=this->vertices.size()-1;
            edgePoint[this->halfEdges[i]->twin]=this->vertices.size()-1;
        }
    }

    //We will now make the new faces
    std::vector<MeshFace> newFaces;
    std::vector<HalfEdge* >newEdges;
    std::map<HalfEdge*,std::pair<HalfEdge*,HalfEdge*>> twinInfo;
    for(int i=0;i<this->halfEdges.size();i++)
    {
        twinInfo[this->halfEdges[i]]=std::make_pair(nullptr,nullptr);
    }
    for(int i=0;i<numberofFaces;i++)
    {
        vec3 facePoint = faceAvg[i];
        MeshVertex currFaceVertex = MeshVertex(nullptr,facePoint);

        this->vertices.emplace_back(currFaceVertex);
        int idx = this->vertices.size()-1;

        //Get all vertices of this face
        std::vector<int> currVertices = this->faces[i].getFaceVertices();
        //Get all HalfEdges associated with this face
        std::vector<HalfEdge *> currEdges = this->faces[i].getFaceEdges();

        int n = currVertices.size(); //This is the number of new quads that will be formed;
        std::vector<MeshFace> currNewFaces;
        for(int j=0;j<n;j++)
        {
            int l1=currVertices[j];
            if(edgePoint.find(currEdges[j])==edgePoint.end())
            {
                std::cout<<"Error in finding edgepoints"<<std::endl;
            }
            int l2=edgePoint[currEdges[j]];
            int l3=idx;
            int l4=edgePoint[currEdges[(n+j-1)%n]];

            HalfEdge *e1=new HalfEdge(l1);
            HalfEdge *e2=new HalfEdge(l2);
            HalfEdge *e3=new HalfEdge(l3);
            HalfEdge *e4=new HalfEdge(l4);

            e1->next=e2;
            e2->next=e3;
            e3->next=e4;
            e4->next=e1;
            newEdges.emplace_back(e1);
            newEdges.emplace_back(e2);
            newEdges.emplace_back(e3);
            newEdges.emplace_back(e4);

            MeshFace currFace = MeshFace(e1,this->faces[i].normal);
            currNewFaces.emplace_back(currFace);

            this->vertices[l1].edge=e1;
            this->vertices[l2].edge=e2;
            this->vertices[l3].edge=e3;
            this->vertices[l4].edge=e4;

            //twin allotment
            if(j>0)
            {
                e3->twin=currNewFaces[j-1].edge->next;
                currNewFaces[j-1].edge->next->twin=e3;
            }
            if(j==n-1)
            {
                e2->twin=currNewFaces[0].edge->next->next;
                currNewFaces[0].edge->next->next->twin=e2;
            }
            //twins for the old edges
            twinInfo[currEdges[j]].first=e1;
            twinInfo[currEdges[(j+n-1)%n]].second=e4;
        }
        for(auto f:currNewFaces) newFaces.emplace_back(f);
    }
    for(auto x:twinInfo)
    {
        HalfEdge *old = x.first;
        HalfEdge *bhai = x.first->twin;
        if(twinInfo[old].first==NULL || twinInfo[bhai].second==NULL)
        {
            std::cout<<"error in making new edges"<<std::endl;
        }
        twinInfo[old].first->twin=twinInfo[bhai].second;
        twinInfo[bhai].second->twin=twinInfo[old].first;
        if(twinInfo[old].second==NULL || twinInfo[bhai].first==NULL)
        {
            std::cout<<"error in making new edges"<<std::endl;
        }
        twinInfo[old].second->twin=twinInfo[bhai].first;
        twinInfo[bhai].first->twin=twinInfo[old].second;
    }
    this->halfEdges=newEdges;
    this->faces=newFaces;   
}

void Mesh::addMesh(Mesh &m)
{
    int n=this->vertices.size();
    for(auto x:m.vertices)
    {
        this->vertices.emplace_back(x);
    }
    for(auto x:m.faces)
    {
        this->faces.emplace_back(x);
    }
    for(auto x:m.halfEdges)
    {
        x->vertexIndex+=n;
        this->halfEdges.emplace_back(x);
    }
}

void Mesh::moveMesh(vec3 direction)
{
    for(auto &x:this->vertices)
    {
        x.position+=direction;
    }
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

float randomFloat(float min, float max) {
    static std::random_device rd;  // Non-deterministic random number generator
    static std::mt19937 gen(rd()); // Mersenne Twister PRNG
    std::uniform_real_distribution<float> dist(min, max);
    return dist(gen);
}


Mesh parseObjFile(const std::string &filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return Mesh{{}, {}, {}};
    }

    std::vector<HalfEdge*> hedges;
    std::vector<MeshVertex> verts;
    std::vector<MeshFace> faces;

    std::vector<glm::vec3> vertices,  normals;
    std::vector<glm::vec2> texCoords;

    std::map<std::pair<int, int>, HalfEdge*> settings;

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string op;
        iss >> op;

        if (op == "v") {
            glm::vec3 v;
            iss >> v.x >> v.y >> v.z;
            verts.push_back(MeshVertex(nullptr, v));
            // vertices.push_back(v);
        } else if (op == "vt") {
            glm::vec3 vt;
            iss >> vt.x >> vt.y;
            texCoords.push_back(vt);
        } else if (op == "vn") {
            glm::vec3 vn;
            iss >> vn.x >> vn.y >> vn.z;
            normals.push_back(vn);
        } else if (op == "f") {
            std::string vertexData;
            std::vector<int> vidx, tidx, nidx;
            while (iss >> vertexData) {
                std::istringstream viss(vertexData);
                std::string token;
                int indices[3] = {0, 0, 0};
                int i = 0;
                while (std::getline(viss, token, '/') && i < 3) {
                    if (!token.empty()) {
                        indices[i] = std::stoi(token);
                    }
                    i++;
                }
                vidx.push_back(indices[0] - 1);
                tidx.push_back(indices[1] - 1);
                nidx.push_back(indices[2] - 1);
            }
            // faces.push_back(face);
            for(int i = 0; i < vidx.size(); ++i) {
                if(nidx[i] != -1) {
                    verts[vidx[i]].normal = normals[nidx[i]];
                }
            }
            for(int i = 0; i < vidx.size(); ++i) {
                HalfEdge* currhe = new HalfEdge(vidx[i]);
                hedges.emplace_back(currhe);
                if(verts[vidx[i]].edge == nullptr) {
                    verts[vidx[i]].edge = currhe;
                }
            }
            for(int i = 0; i < vidx.size(); ++i) {
                int idx = hedges.size() - vidx.size() + i;
                int nextidx = hedges.size() - vidx.size() + ((i + 1) % vidx.size());
                hedges[idx]->next = hedges[nextidx];
                settings[{hedges[idx]->vertexIndex, hedges[idx]->next->vertexIndex}] = hedges[idx];
                if(settings.find({hedges[idx]->next->vertexIndex, hedges[idx]->vertexIndex}) != settings.end()) {
                    HalfEdge *tw = settings[{hedges[idx]->next->vertexIndex, hedges[idx]->vertexIndex}];
                    hedges[idx]->twin = tw;
                    tw->twin = hedges[idx];
                }
            }
            faces.emplace_back(MeshFace(hedges.back(), vec3(0.0, 0.0, 0.0)));
            for(int i = 0; i < vidx.size(); ++i) {
                int idx = hedges.size() - vidx.size() + i;
                hedges[idx]->face = &faces.back();
            }
        }
    }
    file.close();
    Mesh newMesh = Mesh(hedges, verts, faces);
    for(int i = 0; i < newMesh.faces.size(); ++i) {
        newMesh.faces[i].normal = newMesh.getFaceNormal(i);
    }
    return newMesh;
}