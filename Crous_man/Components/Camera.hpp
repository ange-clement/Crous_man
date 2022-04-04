#ifndef CAMERA_COMPONENT_HPP
#define CAMERA_COMPONENT_HPP

#include "../ECS/ComponentSystem.hpp"
#include <common/gBuffer.hpp>

class LShader;

class RendererSystem;


struct CameraC {
    unsigned int frameBufferObject;
    GBuffer gBuffer;

    unsigned int SCR_WIDTH = 1024;
    unsigned int SCR_HEIGHT = 768;

    float fov = 45;
    float ratio = 4.0/3.0;
    float minRange = 0.01;
    float maxRange = 1000.0;


    LShader* lShaderInstance;

    CameraC();
};

class CameraSystem : public virtual ComponentSystem {
public:
    RendererSystem* rendererInstance;
    
public:
    CameraSystem();

    ~CameraSystem();

    virtual void update(unsigned short i, unsigned short entityID);
    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    CameraC* getCamera(unsigned short i);
};

#endif