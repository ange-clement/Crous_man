#ifndef SSAO_QUAD_E_SHADER_HPP
#define SSAO_QUAD_E_SHADER_HPP

#include "../QuadEShader.hpp"

class SSAOQuadEShader : public QuadEShader {
public:
    unsigned int noiseTexture;
    std::vector<glm::vec3> ssaoKernel;

    GLuint gPositionLocation;
    GLuint gNormalLocation;
    GLuint texNoiseLocation;

    GLuint viewLocation;
    GLuint projectionLocation;

public:
    SSAOQuadEShader();
    ~SSAOQuadEShader();

    void setShadowMap(GLuint shadowMap);
    void setPosition(GLuint gPosition);
    void setLightSpaceMatrix(glm::mat4 lightSpaceMatrix);

    virtual void useBuffers(const GLuint* buffers);
    virtual void useVP(const glm::mat4& view, const glm::mat4& projection);
    virtual void setOutputShaders(GLuint* buffers);
};

#endif