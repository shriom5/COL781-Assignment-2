#include "viewer.hpp"
#include "halfedge.hpp"
#include "shapes.hpp"

namespace V = COL781::Viewer;
using namespace glm;

int main() {
    //Create a unit square mesh
    Mesh check=unitSphere(4,2);

    // check.triangulate();

    V::Viewer v;
    if (!v.initialize("Mesh viewer", 640, 480)) {
        return EXIT_FAILURE;
    }

    // check.viewMesh2(v);

    //Memory cleanup

    for(int i=0;i<(int)check.halfEdges.size();i++)
    {
        delete(check.halfEdges[i]);
    }
}
