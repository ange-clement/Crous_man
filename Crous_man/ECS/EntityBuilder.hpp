#ifndef ENTITY_BUILDER_HPP
#define ENTITY_BUILDER_HPP

#include "SystemIDs.hpp"

class Entity;

class EntityBuilder {
private:
	Entity* buildEntity;
public:
	EntityBuilder(std::initializer_list<SystemIDs> systems);

	EntityBuilder* setMeshAsQuad();
	EntityBuilder* setMeshAsFile(std::string meshFile, bool fileHasNormals);
	EntityBuilder* setMeshAsFilePLY(std::string meshFile);

	EntityBuilder* updateRenderer();
	EntityBuilder* setRendererDiffuseSpecular(std::string diffuseFile, std::string specularFile);
	EntityBuilder* setRendererDiffuseColor(glm::vec3 diffuseColor);
	EntityBuilder* setRendererDraw(bool draw);

	EntityBuilder* setDestructibleMeshes(std::initializer_list<std::string> meshesFiles);

	EntityBuilder* setLightColor(glm::vec3 color);
	EntityBuilder* setLightLinear(float linear);
	EntityBuilder* setLightQuadratic(float quadratic);

	EntityBuilder* setAsScreenCamera();

	EntityBuilder* setTranslation(glm::vec3 translation);
	EntityBuilder* setScale(glm::vec3 scale);
	EntityBuilder* setRotation(float angle, glm::vec3 axis);

	EntityBuilder* setChildOf(Entity* parent);

	Entity* build();
};

#endif