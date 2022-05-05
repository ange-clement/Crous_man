#ifndef SHADOW_QUAD_E_SHADER_HPP
#define SHADOW_QUAD_E_SHADER_HPP

#include "../QuadEShader.hpp"

class DepthMeshEShader;
class RendererSystem;

class ShadowQuadEShader : public QuadEShader {
public:
    static ShadowQuadEShader* instance;

    RendererSystem* rendererInstance;

    DepthMeshEShader* depthInstance;
    float near_plane = 1.0f, far_plane = 20.0f;
    float width = 50.0f;
    glm::vec3 shadowLightSourcePos    = glm::vec3(-5.0f, 10.0f, -2.0f);
    glm::vec3 shadowLightSourceTarget = glm::vec3( 0.0f, 0.0f,  0.0f);

    GLuint uShadowMapLocation;
    GLuint gPositionLocation;
    GLuint uLightSpaceMatrixLocation;

public:
    ShadowQuadEShader();
    ~ShadowQuadEShader();

    void setShadowMap(GLuint shadowMap);
    void setPosition(GLuint gPosition);
    void setLightSpaceMatrix(glm::mat4 lightSpaceMatrix);

    virtual void use();
    virtual void useBuffers(std::vector<GLuint> buffers);
};

#endif