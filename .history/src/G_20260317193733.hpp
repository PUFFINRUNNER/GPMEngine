#ifndef G_HPP
#define G_HPP

namespace G {
    struct Vertex {
Point3D position;
Vector3D normal;   // optional at first
// Vec2 uv;        // add later if needed
};

struct Face {
int a, b, c;       // indices into vertices
};

class Mesh {
public:
std::vector<Vertex> vertices;
std::vector<Face> faces;
};

}

#endif