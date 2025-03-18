#include "viewer.hpp"
#include "halfedge.hpp"

namespace V = COL781::Viewer;
using namespace glm;

int main() {
    //Create a unit square mesh
    // Mesh check=unitCube(3,3,3);
    // Mesh check = parseObjFile("./meshes/spot_control_mesh.obj");
    Mesh check = parseObjFile("./meshes/cube.obj");

    // check.triangulate();

    V::Viewer v;
    if (!v.initialize("Mesh viewer", 640, 480)) {
        return EXIT_FAILURE;
    }
    // check.extrudeFace(49,0.4);
    // check.extrudeFace(13,0.4);

    // check.extrudeFace(vec3(-1.5,0.0,0.0),0.4);

    // check.catmullClarkSubdivision();
    // check.catmullClarkSubdivision();

    check.viewMesh2(v);

    //Memory cleanup

    for(int i=0;i<(int)check.halfEdges.size();i++)
    {
        delete(check.halfEdges[i]);
    }
    
    return 0;

    // Mesh curr = parseObjFile("../meshes/cube.obj");
}
