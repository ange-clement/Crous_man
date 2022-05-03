#ifndef CAMERA_COMPONENT_HPP
#define CAMERA_COMPONENT_HPP

#include "../ECS/ComponentSystem.hpp"
#include <common/gBuffer.hpp>
#include <common/textureFramebuffer.hpp>

class MeshEShader;
class QuadEShader;
class LShader;
class PEShader;

class ColliderSystem;
class RendererSystem;
class PointLightSystem;


struct Camera {
    unsigned int SCR_WIDTH = 1024;
    unsigned int SCR_HEIGHT = 768;

    float fov = 45;
    float ratio = 4.0/3.0;
    float minRange = 0.01;
    float maxRange = 1000.0;

    GBuffer gBuffer;                        // Frammebuffer contenant les textures du GShader (fragPos, normal, albedo, specular)
    TextureFramebuffer textureFramebuffer;  // Frammebuffer contenant la texture du LShader

    std::vector<MeshEShader*> meshEShadersinstances;
    std::vector<QuadEShader*> quadEShadersinstances;
    LShader* lShaderInstance;
    PEShader* peShaderInstance;

    Camera();
    void updateWidthHeight(unsigned int width, unsigned int height);
};

class CameraSystem : public virtual ComponentSystem {
public:
    RendererSystem* rendererInstance;
    PointLightSystem* pointLightInstance;
    ColliderSystem* colliderRenderInstance;
    Camera* screenCamera;
    
public:
    CameraSystem();

    ~CameraSystem();

    virtual void render(unsigned short i, unsigned short entityID);
    virtual void initialize(unsigned short i, unsigned short entityID);
    virtual void addEntityComponent();

    void setScreenCamera(unsigned short entityID);

    Camera* getCamera(unsigned short i);
};

#endif