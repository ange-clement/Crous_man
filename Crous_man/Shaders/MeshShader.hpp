#ifndef MESH_SHADER_HPP
#define MESH_SHADER_HPP

#include "Shader.hpp"

class MeshShader : public Shader {
public:
    GLuint model;
    GLuint view;
    GLuint projection;
    GLuint normal;

public:
    MeshShader(std::string fs);
    ~MeshShader();

    void setMVPN(const glm::mat4& model, const glm::mat4& view, const glm::mat4& projection, const glm::mat4& normal);
};

#endif