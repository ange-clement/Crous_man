#ifndef RENDERER_COMPONENT_HPP
#define RENDERER_COMPONENT_HPP

#include "../ECS/ComponentSystem.hpp"

class GShader;

struct Mesh;

struct Renderer {
    unsigned short meshID;
    GShader* gShaderInstance;

    GLuint vertexArray;

    GLuint vertexbuffer;
    GLuint textCoordBuffer;
    GLuint normalsbuffer;
    GLuint elementbuffer;

    GLuint diffuseBuffer;
    GLuint specularBuffer;
};

class RendererSystem : public virtual ComponentSystem {
public:
    RendererSystem();

    ~RendererSystem();

    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    void initBuffers(unsigned short i, unsigned short entityID);

    void renderAll(glm::mat4 view, glm::mat4 projection);

    Renderer* getRenderer(unsigned short i);
    Mesh* getMesh(unsigned short i);
};

#endif