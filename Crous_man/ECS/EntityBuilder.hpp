#ifndef ENTITY_BUILDER_HPP
#define ENTITY_BUILDER_HPP

#include "SystemIDs.hpp"

struct Mesh;

class RendererSystem;
struct Renderer;

struct Destructible;

struct PointLight;

class Entity;

class EntityBuilder {
private:
	Entity* buildEntity;

	Mesh* mesh;
	
	RendererSystem* rendererSystem;
	Renderer* renderer;
	unsigned short rendererID;

	Destructible* destructible;

	PointLight* pointLight;
public:
	EntityBuilder(std::initializer_list<SystemIDs> systems);

	Mesh* getMesh();
	EntityBuilder* setMeshAsQuad();
	EntityBuilder* setMeshAsFile(std::string meshFile, bool fileHasNormals);
	EntityBuilder* setMeshAsFilePLY(std::string meshFile);
	EntityBuilder* setMeshAsFilePLY(std::string meshFile, bool invertTriangles);

	RendererSystem* getRendererSystem();
	Renderer* getRenderer();
	unsigned short getRendererID();
	EntityBuilder* updateRenderer();
	EntityBuilder* setRendererDiffuseSpecular(std::string diffuseFile, std::string specularFile);
	EntityBuilder* setRendererDiffuseColor(glm::vec3 diffuseColor);
	EntityBuilder* setRendererDraw(bool draw);

	Destructible* getDestructible();
	EntityBuilder* addDestructibleMeshes(std::initializer_list<std::string> meshesFiles);
	EntityBuilder* addDestructibleMeshes(std::initializer_list<std::string> meshesFiles, bool invertTriangles);

	PointLight* getPointLight();
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