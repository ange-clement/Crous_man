#ifndef RENDERER_COMPONENT_HPP
#define RENDERER_COMPONENT_HPP

#include "../ECS/ComponentSystem.hpp"

class GShader;

struct MeshC;

struct Renderer {
    unsigned short meshID;
    GShader* gShaderInstance;

    GLuint vertexArray;

    GLuint vertexbuffer;
    GLuint textCoordBuffer;
    GLuint normalsbuffer;
    GLuint elementbuffer;
};

class RendererSystem : public virtual ComponentSystem {
public:
    RendererSystem();

    ~RendererSystem();

    virtual void update(unsigned short i, unsigned short entityID);
    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    void initBuffers(unsigned short i, unsigned short entityID);

    void renderAll(glm::mat4 view, glm::mat4 projection);

    Renderer* getRenderer(unsigned short i);
    MeshC* getMesh(unsigned short i);
};

#endif