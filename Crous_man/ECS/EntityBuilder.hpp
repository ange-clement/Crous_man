#ifndef ENTITY_BUILDER_HPP
#define ENTITY_BUILDER_HPP

#include "SystemIDs.hpp"
#include "../Components/Collider.hpp"

#define DEBUG_ENTITY_BUILDER false

struct Mesh;

class RendererSystem;
struct Renderer;

struct Destructible;

struct PointLight;

struct RigidBody;

class Entity;

class CrousManController;

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

	FollowObject* followObject;

	CrousManController* crousManController;
public:
	EntityBuilder(std::initializer_list<SystemIDs> systems);

	EntityBuilder* setActive(bool activeStatus);
	EntityBuilder* initializeComponents();

	Mesh* getMesh();
	EntityBuilder* setMeshAsQuad();
	EntityBuilder* setMeshAsCube();
	EntityBuilder* setMeshAsFile(std::string meshFile, bool fileHasNormals);
	EntityBuilder* setMeshAsFilePLY(std::string meshFile);
	EntityBuilder* setMeshAsFilePLY(std::string meshFile, bool invertTriangles);
	EntityBuilder* setMeshAsFilePLYCenter(std::string meshFile);

	RendererSystem* getRendererSystem();
	Renderer* getRenderer();
	unsigned short getRendererID();
	EntityBuilder* updateRenderer();
	EntityBuilder* setRendererDiffuse(std::string diffuseFile);
	EntityBuilder* setRendererSpecular(std::string specularFile);
	EntityBuilder* setRendererDiffuseSpecular(std::string diffuseFile, std::string specularFile);
	EntityBuilder* setRendererDiffuseColor(glm::vec3 diffuseColor);
	EntityBuilder* setRendererSpecularValue(float spec);
	EntityBuilder* setRendererDraw(bool draw);
	EntityBuilder* setRendererCastShadows(bool castShadows);

	Destructible* getDestructible();
	EntityBuilder* addDestructibleMeshes(std::initializer_list<std::string> meshesFiles);
	EntityBuilder* addDestructibleMeshes(std::initializer_list<std::string> meshesFiles, bool invertTriangles);
	EntityBuilder* setDestructibleFragmentScaling(glm::vec3 fragmentScaling);
	EntityBuilder* setDestructibleFragmentColor(glm::vec3 fragmentColor);
	EntityBuilder* setDestructibleHealth(float health);

	EntityBuilder* setColliderType(colliderType type);
	EntityBuilder* setColliderCenter(glm::vec3 pos);
	EntityBuilder* setColliderRadius(float radius);
	EntityBuilder* setColliderSize(glm::vec3 size);
	EntityBuilder* setColliderOrientation(glm::mat3 orientation);

	EntityBuilder* fitSphereColliderToMesh();
	EntityBuilder* fitAABBColliderToMesh();
	EntityBuilder* fitOBBColliderToMesh();
	EntityBuilder* fitOBBColliderToMeshOf(Entity* meshEntity);
	EntityBuilder* setRenderingCollider();

	RigidBody* getRigidBody();
	EntityBuilder* setRigidBodyMass(float mass);
	EntityBuilder* setRigidBodyStatic(bool staticStatus);


	PointLight* getPointLight();
	EntityBuilder* setLightColor(glm::vec3 color);
	EntityBuilder* setLightLinear(float linear);
	EntityBuilder* setLightQuadratic(float quadratic);

	EntityBuilder* setAsScreenCamera();

	FollowObject* getFollowObject();
	EntityBuilder* setFollowObjectEntity(Entity* target);
	
	CrousManController* getCrousManController();
	EntityBuilder* setCrousManControllerRotatingCenterForCamera(Entity* target);
	EntityBuilder* setCrousManControllerCameraTarget(Entity* target);
	EntityBuilder* setCrousManControllerCameraEntity(Entity* target);
	EntityBuilder* setCrousManControllerSaucisseEntity(Entity* saucisse);
	EntityBuilder* setCrousManControllerLaserEntity(Entity* laser);

	EntityBuilder* setTranslation(glm::vec3 translation);
	EntityBuilder* setScale(glm::vec3 scale);
	EntityBuilder* setRotation(float angle, glm::vec3 axis);
	EntityBuilder* setLookAt(glm::vec3 target);

	EntityBuilder* setChildOf(Entity* parent);

	EntityBuilder* setAsAudioListener();

	Entity* build();
};

void computeBox(Transform* t, const std::vector<glm::vec3> in_vertices, glm::vec3& position, glm::vec3& size);

#endif