#ifndef ENTITY_BUILDER_HPP
#define ENTITY_BUILDER_HPP

#include "SystemIDs.hpp"

struct Mesh;

class RendererSystem;
struct Renderer;

struct Destructible;

struct PointLight;

struct RigidBody;

class Entity;
enum colliderType;

class EntityBuilder {
private:
	Entity* buildEntity;

	Mesh* mesh;
	
	RendererSystem* rendererSystem;
	Renderer* renderer;
	unsigned short rendererID;

	Destructible* destructible;

	PointLight* pointLight;

	RigidBody* rigidBody;
public:
	EntityBuilder(std::initializer_list<SystemIDs> systems);

	EntityBuilder* setActive(bool activeStatus);
	EntityBuilder* initializeComponents();

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

	EntityBuilder* setColliderType(colliderType type);
	EntityBuilder* setColliderPosition(glm::vec3 pos);
	EntityBuilder* setColliderRadius(float radius);
	EntityBuilder* setColliderSize(glm::vec3 size);
	EntityBuilder* setColliderOrientation(glm::mat3 orientation);

	EntityBuilder* fitSphereColliderToMesh();
	EntityBuilder* fitAABBColliderToMesh();
	EntityBuilder* fitOBBColliderToMesh();
	EntityBuilder* setRenderingCollider();

	RigidBody* getRigidBody();
	EntityBuilder* setRigidBodyMass(float mass);


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