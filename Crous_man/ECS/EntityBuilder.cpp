#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <common/meshGenerator.hpp>
#include <common/texture.hpp>

#include "../Util.hpp"

#include "../Transform.hpp"
#include "../Components/Mesh.hpp"
#include "../Components/Renderer.hpp"
#include "../Components/Destructible.hpp"
#include "../Components/PointLight.hpp"
#include "../Components/Camera.hpp"
#include "../Components/Collider.hpp"
#include "../Components/RigidBody.hpp"
#include "../Components/FollowObject.hpp"
#include "../Components/Controllers/CrousManController.hpp"
#include "../Transform.hpp"
#include "../SoundManager.hpp"

#include "Bitmap.hpp"
#include "EntityManager.hpp"

#include "EntityBuilder.hpp"
#include "Entity.hpp"

EntityBuilder::EntityBuilder(std::initializer_list<SystemIDs> systems) {
	this->buildEntity = new Entity(systems);
	EntityManager::instance->addEntity(this->buildEntity);

	this->mesh = NULL;

	this->rendererSystem = NULL;
	this->renderer = NULL;
	this->rendererID = (unsigned short)-1;

	this->destructible = NULL;

	this->pointLight = NULL;

	this->rigidBody = NULL;

	this->followObject = NULL;
	
	this->crousManController = NULL;

}


EntityBuilder* EntityBuilder::setActive(bool activeStatus) {
	this->buildEntity->isActive = false;
	return this;
}


Mesh* EntityBuilder::getMesh() {
	if (this->mesh == NULL) {
		unsigned short meshID = EntityManager::instance->getComponentId(SystemIDs::MeshID, this->buildEntity->id);
		this->mesh = &EntityManager::instance->meshComponents[meshID];
	}
	return this->mesh;
}

EntityBuilder* EntityBuilder::setMeshAsQuad() {
	Mesh* mesh = this->getMesh();
	quad(mesh->indexed_vertices, mesh->normals, mesh->UV, mesh->indices, mesh->triangles);
	return this;
}

EntityBuilder* EntityBuilder::setMeshAsCube() {
	Mesh* mesh = this->getMesh();
	cube(mesh->indexed_vertices, mesh->normals, mesh->UV, mesh->indices, mesh->triangles);
	return this;
}

EntityBuilder* EntityBuilder::setMeshAsFile(std::string meshFile, bool fileHasNormals) {
	this->getMesh()->loadFromFile(meshFile, false);
	return this;
}

EntityBuilder* EntityBuilder::setMeshAsFilePLYCenter(std::string meshFile) {
	setMeshAsFilePLY(meshFile, false);
	glm::vec3 centerPos = glm::vec3(0.0f);
	unsigned int size = this->getMesh()->indexed_vertices.size();
	for (unsigned int i = 0; i < size; i++) {
		centerPos += this->getMesh()->indexed_vertices[i];
	}
	centerPos /= size;
	for (unsigned int i = 0; i < size; i++) {
		this->getMesh()->indexed_vertices[i] = this->getMesh()->indexed_vertices[i] - centerPos;
	}
	return this;
}
EntityBuilder* EntityBuilder::setMeshAsFilePLY(std::string meshFile) {
	return setMeshAsFilePLY(meshFile, false);
}
EntityBuilder* EntityBuilder::setMeshAsFilePLY(std::string meshFile, bool invertTriangles) {
	this->getMesh()->loadFromFilePLY(meshFile, invertTriangles);
	return this;
}



RendererSystem* EntityBuilder::getRendererSystem() {
	if (this->rendererSystem == NULL) {
		this->rendererSystem = dynamic_cast<RendererSystem*>(EntityManager::instance->systems[SystemIDs::RendererID]);
	}
	return this->rendererSystem;
}

Renderer* EntityBuilder::getRenderer() {
	if (this->renderer == NULL) {
		unsigned short rendererID = this->getRendererID();
		this->renderer = &EntityManager::instance->rendererComponents[rendererID];
	}
	return this->renderer;
}

unsigned short EntityBuilder::getRendererID() {
	if (this->rendererID == (unsigned short)-1) {
		this->rendererID = EntityManager::instance->getComponentId(SystemIDs::RendererID, this->buildEntity->id);
	}
	return this->rendererID;
}

EntityBuilder* EntityBuilder::updateRenderer() {
	unsigned short rendererID = this->getRendererID();
	RendererSystem* rendererSystem = this->getRendererSystem();
	rendererSystem->initBuffers(rendererID, this->buildEntity->id);
	return this;
}

EntityBuilder* EntityBuilder::setRendererDiffuse(std::string diffuseFile) {
	Renderer* renderer = this->getRenderer();
	renderer->diffuseBuffer = loadTextureFromPPM(diffuseFile.c_str());
	return this;
}

EntityBuilder* EntityBuilder::setRendererSpecular(std::string specularFile) {
	Renderer* renderer = this->getRenderer();
	renderer->setSpecularBuffer(loadTextureFromPGM(specularFile.c_str()));
	return this;
}

EntityBuilder* EntityBuilder::setRendererDiffuseSpecular(std::string diffuseFile, std::string specularFile) {
	Renderer* renderer = this->getRenderer();
	renderer->diffuseBuffer = loadTextureFromPPM(diffuseFile.c_str());
	renderer->setSpecularBuffer(loadTextureFromPGM(specularFile.c_str()));
	return this;
}

EntityBuilder* EntityBuilder::setRendererDiffuseColor(glm::vec3 diffuseColor) {
	Renderer* renderer = this->getRenderer();
	renderer->setDiffuseBuffer(loadTextureFromColor(diffuseColor));
	return this;
}

EntityBuilder* EntityBuilder::setRendererSpecularValue(float spec) {
	Renderer* renderer = this->getRenderer();
	renderer->setSpecularBuffer(loadTextureFromFloat(spec));
	return this;
}

EntityBuilder* EntityBuilder::setRendererDraw(bool draw) {
	Renderer* renderer = this->getRenderer();
	renderer->draw = draw;
	return this;
}

EntityBuilder* EntityBuilder::setRendererCastShadows(bool castShadows) {
	Renderer* renderer = this->getRenderer();
	renderer->castShadows = castShadows;
	return this;
}



Destructible* EntityBuilder::getDestructible() {
	if (this->destructible == NULL) {
		unsigned short destructibleID = EntityManager::instance->getComponentId(SystemIDs::DestructibleID, this->buildEntity->id);
		this->destructible = &EntityManager::instance->destructibleComponents[destructibleID];
	}
	return this->destructible;
}

EntityBuilder* EntityBuilder::addDestructibleMeshes(std::initializer_list<std::string> meshesFiles) {
	return addDestructibleMeshes(meshesFiles, false);
}

EntityBuilder* EntityBuilder::addDestructibleMeshes(std::initializer_list<std::string> meshesFiles, bool invertTriangles) {
	Destructible* destructible = this->getDestructible();
	for (std::string meshFile : meshesFiles) {
		destructible->fragmentMeshFiles.push_back(meshFile);
		destructible->fragmentMeshInvertTriangle.push_back(invertTriangles);
	}
	return this;
}

EntityBuilder* EntityBuilder::setDestructibleFragmentScaling(glm::vec3 fragmentScaling) {
	Destructible* destructible = this->getDestructible();
	destructible->fragmentScaling = fragmentScaling;
	return this;
}

EntityBuilder* EntityBuilder::setDestructibleFragmentColor(glm::vec3 fragmentColor) {
	Destructible* destructible = this->getDestructible();
	destructible->fragmentColor = fragmentColor;
	return this;
}

EntityBuilder* EntityBuilder::setDestructibleHealth(float health) {
	Destructible* destructible = this->getDestructible();
	destructible->destructionAmount = health;
	return this;
}


EntityBuilder* EntityBuilder::setColliderType(colliderType type) {
	unsigned short colliderID = EntityManager::instance->getComponentId(SystemIDs::ColliderID, this->buildEntity->id);
	Collider* collider = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID])->getCollider(colliderID);
	collider->type = type;
	return this;
}
EntityBuilder* EntityBuilder::setColliderCenter(glm::vec3 pos) {
	unsigned short colliderID = EntityManager::instance->getComponentId(SystemIDs::ColliderID, this->buildEntity->id);
	Collider* collider = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID])->getCollider(colliderID);
	collider->center = pos;
	collider->position = pos;
	return this;
}
EntityBuilder* EntityBuilder::setColliderRadius(float radius) {
	unsigned short colliderID = EntityManager::instance->getComponentId(SystemIDs::ColliderID, this->buildEntity->id);
	Collider* collider = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID])->getCollider(colliderID);
	collider->radius = radius;
	return this;
}
EntityBuilder* EntityBuilder::setColliderSize(glm::vec3 size) {
	unsigned short colliderID = EntityManager::instance->getComponentId(SystemIDs::ColliderID, this->buildEntity->id);
	Collider* collider = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID])->getCollider(colliderID);
	collider->size = size;
	return this;
}
EntityBuilder* EntityBuilder::setColliderOrientation(glm::mat3 orientation) {
	unsigned short colliderID = EntityManager::instance->getComponentId(SystemIDs::ColliderID, this->buildEntity->id);
	Collider* collider = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID])->getCollider(colliderID);
	collider->orientation = orientation;
	return this;
}

void computeSphere(Transform* t, const std::vector<glm::vec3> in_vertices, glm::vec3& position, float& radius) {
	glm::vec3 tmpPosition = glm::vec3(0.0);
	float tmpRadius = 0.f;

	glm::vec3 tmpVertexPos;
	glm::vec3 vertexPos;
	for (size_t i = 0, size = in_vertices.size(); i < size; i++) {
		tmpVertexPos = in_vertices[i];
		vertexPos = t->applyToVector(tmpVertexPos);
		tmpPosition += vertexPos;
	}
	tmpPosition /= (float)in_vertices.size();

	for (size_t i = 0, size = in_vertices.size(); i < size; i++) {
		tmpVertexPos = in_vertices[i];
		vertexPos = t->applyToVector(tmpVertexPos);
		tmpRadius = std::max(tmpRadius, glm::distance(tmpPosition, vertexPos));
	}

	position = tmpPosition;
	radius = tmpRadius;
}
void computeBox(Transform* t, const std::vector<glm::vec3> in_vertices, glm::vec3& position, glm::vec3& size) {
	glm::vec3 cur_pt;
	cur_pt = in_vertices[0];
	float min_dim = 1;

	float min_x = cur_pt.x;
	float max_x = cur_pt.x;
	float min_y = cur_pt.y;
	float max_y = cur_pt.y;
	float min_z = cur_pt.z;
	float max_z = cur_pt.z;

	for (size_t i = 0; i < in_vertices.size(); i++) {
		glm::vec3 cur_pt = in_vertices[i];

		//cur_pt = t->applyToVector(cur_pt);

		if (cur_pt.x < min_x) {
			min_x = cur_pt.x;
		}
		if (cur_pt.x > max_x) {
			max_x = cur_pt.x;
		}
		if (cur_pt.y < min_y) {
			min_y = cur_pt.y;
		}
		if (cur_pt.y > max_y) {
			max_y = cur_pt.y;
		}
		if (cur_pt.z < min_z) {
			min_z = cur_pt.z;
		}
		if (cur_pt.z > max_z) {
			max_z = cur_pt.z;
		}
	}

	float dim_x = max_x - min_x;
	float dim_y = max_y - min_y;
	float dim_z = max_z - min_z;
	
	dim_x *= t->scaling.x;
	dim_y *= t->scaling.y;
	dim_z *= t->scaling.z;


	dim_x = (dim_x < min_dim) ? min_dim : dim_x;
	dim_y = (dim_y < min_dim) ? min_dim : dim_y;
	dim_z = (dim_z < min_dim) ? min_dim : dim_z;


	position = glm::vec3((max_x + min_x) / 2.0f, (max_y + min_y) / 2.0f, (max_z + min_z) / 2.0f);
	size = glm::vec3(dim_x / 2.0f, dim_y / 2.0f, dim_z / 2.0f);
}
void initShaderCollider(Collider* c) {
	if (!c->shader) {
		if (c->type == colliderType::Sphere) {
			c->shader = SphereColliderShader::instance;
		}else {
			c->shader = BoxColliderShader::instance;
		}
	}
	c->shader->use();
	c->shader->init();
}


EntityBuilder* EntityBuilder::fitSphereColliderToMesh() {
	unsigned short colliderID = EntityManager::instance->getComponentId(SystemIDs::ColliderID, this->buildEntity->id);
	Collider* collider = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID])->getCollider(colliderID);
	collider->type = colliderType::Sphere;
	computeSphere(this->buildEntity->worldTransform, mesh->indexed_vertices, collider->center, collider->radius);
	
	if (DEBUG_ENTITY_BUILDER) {
		print(collider->center);
		std::cout << "radius : " << collider->radius << std::endl;
	}
	return this;
}
EntityBuilder* EntityBuilder::fitAABBColliderToMesh() {
	Mesh* mesh = this->getMesh();
	if (DEBUG_ENTITY_BUILDER)
		std::cout << "AABB" << std::endl;

	unsigned short colliderID = EntityManager::instance->getComponentId(SystemIDs::ColliderID, this->buildEntity->id);
	Collider* collider = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID])->getCollider(colliderID);
	collider->type = colliderType::AABB;
	computeBox(this->buildEntity->worldTransform, mesh->indexed_vertices, collider->center, collider->size);

	if (DEBUG_ENTITY_BUILDER) {
		print(collider->center);
		print(collider->size);
	}

	return this;
}
EntityBuilder* EntityBuilder::fitOBBColliderToMesh() {
	Mesh* mesh = this->getMesh();
	unsigned short colliderID = EntityManager::instance->getComponentId(SystemIDs::ColliderID, this->buildEntity->id);
	Collider* collider = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID])->getCollider(colliderID);
	collider->type = colliderType::OBB;
	computeBox(this->buildEntity->worldTransform, mesh->indexed_vertices, collider->center, collider->size);
	
	if (DEBUG_ENTITY_BUILDER) {
		print(collider->center);
		print(collider->size);
	}
	return this;
}
EntityBuilder* EntityBuilder::fitOBBColliderToMeshOf(Entity* meshEntity) {
	unsigned short meshEntityMeshID = EntityManager::instance->getComponentId(SystemIDs::MeshID, meshEntity->id);
	Mesh* meshEntityMesh = &EntityManager::instance->meshComponents[meshEntityMeshID];

	unsigned short colliderID = EntityManager::instance->getComponentId(SystemIDs::ColliderID, this->buildEntity->id);
	Collider* collider = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID])->getCollider(colliderID);
	collider->type = colliderType::OBB;
	computeBox(meshEntity->worldTransform, meshEntityMesh->indexed_vertices, collider->center, collider->size);

	return this;
}
EntityBuilder* EntityBuilder::setRenderingCollider() {
	unsigned short colliderID = EntityManager::instance->getComponentId(SystemIDs::ColliderID, this->buildEntity->id);
	Collider* collider = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID])->getCollider(colliderID);
	initShaderCollider(collider);
	collider->drawable = true;
	return this;
}



RigidBody* EntityBuilder::getRigidBody() {
	if (this->rigidBody == NULL) {
		unsigned short rigidBodyID = EntityManager::instance->getComponentId(SystemIDs::RigidBodyID, this->buildEntity->id);
		this->rigidBody = &EntityManager::instance->rigidBodyComponents[rigidBodyID];
	}
	return this->rigidBody;
}
EntityBuilder* EntityBuilder::setRigidBodyMass(float mass) {
	getRigidBody()->setMass(mass);
	return this;
}
EntityBuilder* EntityBuilder::setRigidBodyStatic(bool staticStatus) {
	getRigidBody()->static_RB = staticStatus;
	getRigidBody()->setMass(FLT_MAX);
	return this;
}



PointLight* EntityBuilder::getPointLight() {
	if (this->pointLight == NULL) {
		unsigned short pointLightID = EntityManager::instance->getComponentId(SystemIDs::PointLightID, this->buildEntity->id);
		this->pointLight = &EntityManager::instance->pointLightComponents[pointLightID];
	}
	return this->pointLight;
}
EntityBuilder* EntityBuilder::setLightColor(glm::vec3 color) {
	PointLight* PL = this->getPointLight();
	PL->color = color;
	return this;
}
EntityBuilder* EntityBuilder::setLightLinear(float linear) {
	PointLight* PL = this->getPointLight();
	PL->linear = linear;
	return this;
}
EntityBuilder* EntityBuilder::setLightQuadratic(float quadratic) {
	PointLight* PL = this->getPointLight();
	PL->quadratic = quadratic;
	return this;
}



EntityBuilder* EntityBuilder::setAsScreenCamera() {
	dynamic_cast<CameraSystem*>(EntityManager::instance->systems[SystemIDs::CameraID])->setScreenCamera(this->buildEntity->id);
	return this;
}



FollowObject* EntityBuilder::getFollowObject() {
	if (this->followObject == NULL) {
		unsigned short followObjectID = EntityManager::instance->getComponentId(SystemIDs::FollowObjectID, this->buildEntity->id);
		this->followObject = &EntityManager::instance->followObjectComponents[followObjectID];
	}
	return this->followObject;
}

EntityBuilder* EntityBuilder::setFollowObjectEntity(Entity* target) {
	FollowObject* f = this->getFollowObject();
	f->entityToFollow = target;
	return this;
}



CrousManController* EntityBuilder::getCrousManController() {
	if (this->crousManController == NULL) {
		unsigned short crousManControllerID = EntityManager::instance->getComponentId(SystemIDs::CrousManControllerID, this->buildEntity->id);
		this->crousManController = &EntityManager::instance->crousManControllerComponents[crousManControllerID];
	}
	return this->crousManController;
}

EntityBuilder* EntityBuilder::setCrousManControllerRotatingCenterForCamera(Entity* target) {
	CrousManController* crous = this->getCrousManController();
	crous->rotatingCenterForCamera = target;
	return this;
}
EntityBuilder* EntityBuilder::setCrousManControllerCameraTarget(Entity* target) {
	CrousManController* crous = this->getCrousManController();
	crous->cameraTarget = target;
	return this;
}
EntityBuilder* EntityBuilder::setCrousManControllerCameraEntity(Entity* camera) {
	CrousManController* crous = this->getCrousManController();
	crous->camera = camera;
	return this;
}
EntityBuilder* EntityBuilder::setCrousManControllerSaucisseEntity(Entity* saucisse) {
	CrousManController* crous = this->getCrousManController();
	crous->saucisseEntity = saucisse;
	return this;
}
EntityBuilder* EntityBuilder::setCrousManControllerLaserEntity(Entity* laser) {
	CrousManController* crous = this->getCrousManController();
	crous->laserEntity = laser;
	return this;
}



EntityBuilder* EntityBuilder::setTranslation(glm::vec3 translation) {
	this->buildEntity->transform->translation = translation;
	this->buildEntity->updateTransforms();
	return this;
}
EntityBuilder* EntityBuilder::setScale(glm::vec3 scale) {
	this->buildEntity->transform->scaling = scale;
	this->buildEntity->updateTransforms();
	return this;
}
EntityBuilder* EntityBuilder::setRotation(float angle, glm::vec3 axis) {
	this->buildEntity->transform->rotation.setRotation(angle, axis);
	this->buildEntity->updateTransforms();
	return this;
}
EntityBuilder* EntityBuilder::setLookAt(glm::vec3 target) {
	this->buildEntity->transform->lookAt(target);
	this->buildEntity->updateTransforms();
	return this;
}

EntityBuilder* EntityBuilder::setChildOf(Entity* parent) {
	parent->addChildren(this->buildEntity);
	this->buildEntity->updateTransforms();
	return this;
}

EntityBuilder* EntityBuilder::setAsAudioListener() {
	SoundManager::instance->setAudioListener(this->buildEntity);
	return this;
}

EntityBuilder* EntityBuilder::initializeComponents() {
	Bitmap* entityBitmap;
    Bitmap* systemBitmap;

    for (ComponentSystem* system : EntityManager::instance->systems) {
        entityBitmap = this->buildEntity->componentsBitmap;
        systemBitmap = system->requiredComponentsBitmap;
        if (entityBitmap->combine(systemBitmap)->equals(systemBitmap)) {
            system->initialize(system->getComponentId(this->buildEntity->id), this->buildEntity->id);
        }
    }
	return this;
}

Entity* EntityBuilder::build() {
	return this->buildEntity;
}