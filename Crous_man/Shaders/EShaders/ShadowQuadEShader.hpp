#ifndef SHADOW_QUAD_E_SHADER_HPP
#define SHADOW_QUAD_E_SHADER_HPP

#include "../QuadEShader.hpp"

class DepthMeshEShader;
class RendererSystem;

class Entity;

class ShadowQuadEShader : public QuadEShader {
public:
    static ShadowQuadEShader* instance;

    RendererSystem* rendererInstance;

    DepthMeshEShader* depthInstance;
    float near_plane = 1.0f, far_plane = 600.0f;
    float width = 100.0f;

    Entity* targetEntity;
    glm::vec3 shadowDirection = glm::normalize(glm::vec3(-.2f, -0.9f, 0.1f));
    float shadowSourceDistance = 500.0f;

    GLuint uShadowMapLocation;
    GLuint gPositionLocation;
    GLuint uLightSpaceMatrixLocation;

public:
    ShadowQuadEShader();
    ~ShadowQuadEShader();

    void setShadowMap(GLuint shadowMap);
    void setPosition(GLuint gPosition);
    void setLightSpaceMatrix(glm::mat4 lightSpaceMatrix);

    virtual void useBuffers(const GLuint* buffers);
    virtual void setOutputShaders(GLuint* buffers);

    virtual void use();
};

#endif