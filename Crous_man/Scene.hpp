#ifndef SCENE_HPP
#define SCENE_HPP

class Transform;
class Texture;
class Mesh;
class PointLight;
class InputManager;

class SceneObject {
public:
    //Components
    Transform* transform;
    Transform* worldTransform;
    Mesh* mesh;

    //Hierarchy
    SceneObject* parent;
    std::vector<SceneObject*> childrens;
public:

    SceneObject();

    ~SceneObject();

    void addChildren(SceneObject* children);

    void applyTransformToPoints();

    void loadMesh(std::string fileName);
    void loadMesh(std::string fileName, bool fileHasNormals);

    void loadShaders();

    glm::vec3 localToWorld(glm::vec3 localposition);

    virtual void processInput(GLFWwindow *window, InputManager* inputManager, float deltaTime);

    void updateTransforms();

    void updateLights(const std::vector<PointLight*> & pointLights);

    virtual void draw(glm::mat4 view, glm::mat4 projection);
};

#endif