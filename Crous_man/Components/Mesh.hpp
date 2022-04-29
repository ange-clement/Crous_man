#ifndef MESH_COMPONENT_HPP
#define MESH_COMPONENT_HPP

#include "../ECS/ComponentSystem.hpp"

struct Mesh {
    std::vector<glm::vec3> indexed_vertices;
    std::vector<unsigned short> indices;
    std::vector<std::vector<unsigned short>> triangles;
    std::vector<glm::vec3> triangle_normals;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec2> UV;

    void loadFromFile(std::string filename, bool fileHasNormals);
    void loadFromFilePLY(std::string filename, bool invertTriangles);

    void computeTrianglesNormals();

    void computeSmoothVertexNormal();

    void computeNormals();
};

class MeshSystem : public virtual ComponentSystem {
public:
    MeshSystem();

    ~MeshSystem();

    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    Mesh* getMesh(unsigned short i);
};

#endif