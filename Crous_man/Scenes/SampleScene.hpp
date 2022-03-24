#ifndef SAMPLE_SCENE_HPP
#define SAMPLE_SCENE_HPP

class Camera;
class PointLight;

SceneObject* createSceneMonkey(Camera** camera, std::vector<PointLight*>& pointLights);

#endif