#ifndef DEPTH_MESH_E_SHADER_HPP
#define DEPTH_MESH_E_SHADER_HPP

#include "../MeshEShader.hpp"

class DepthMeshEShader : public MeshEShader {
public:
    static DepthMeshEShader* instance;

    GLuint uFromPosLocation;
    GLuint uMaxDistanceLocation;

public:
    DepthMeshEShader();
    ~DepthMeshEShader();

    void setFromPos(glm::vec3 fromPos);
    void setMaxDistance(float maxDistance);
};

#endif