#ifndef COLLIDER_SHADER_HPP
#define COLLIDER_SHADER_HPP

#include "Shader.hpp"

class Collider;

class ColliderShader : public Shader {
public:
    GLuint colliderColor;

    GLuint model;
    GLuint view;
    GLuint projection;
    GLuint size;

    glm::vec3 onColidColor      = glm::vec3(1, 0, 0);
    glm::vec3 notOnColidColor   = glm::vec3(0, 1, 0);


    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> axis;
    std::vector<unsigned short> indices;

    GLuint vertexbuffer;
    GLuint elementbuffer;
    GLuint VertexArrayID;
public:
    ColliderShader();
    virtual ~ColliderShader();
    void setMVPC(const glm::mat4& model, const glm::mat4& view, const glm::mat4& projection, bool incontact,const  glm::vec3& size);

    void init_uniform();
    virtual void init_geometry() = 0;
    void init();
    void draw();
};


class SphereColliderShader : public ColliderShader {
public :
    SphereColliderShader();
    virtual ~SphereColliderShader();
    static SphereColliderShader* instance;
    virtual void init_geometry();
};

class BoxColliderShader : public ColliderShader {
public:
    BoxColliderShader();
    virtual ~BoxColliderShader();
    static BoxColliderShader* instance;
    virtual void init_geometry();
};
#endif