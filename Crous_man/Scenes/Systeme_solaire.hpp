#ifndef SYSTEME_SOLAIRE_HPP
#define SYSTEME_SOLAIRE_HPP

class Camera;
class PointLight;

SceneObject* createSceneSolaire(Camera*& camera, std::vector<PointLight*>& pointLights);

#endif