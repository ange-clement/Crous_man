#ifndef OBJECT_CONTROL_HPP
#define OBJECT_CONTROL_HPP

class Camera;
class PointLight;

SceneObject* createSceneObject(Camera*& camera, std::vector<PointLight*>& pointLights);

#endif