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
#include "../Transform.hpp"

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

EntityBuilder* EntityBuilder::setMeshAsFile(std::string meshFile, bool fileHasNormals) {
	this->getMesh()->loadFromFile(meshFile, false);
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

EntityBuilder* EntityBuilder::setRendererDiffuseSpecular(std::string diffuseFile, std::string specularFile) {
	Renderer* renderer = this->getRenderer();
	renderer->diffuseBuffer = loadTextureFromPPM(diffuseFile.c_str());
	renderer->specularBuffer = loadTextureFromPGM(specularFile.c_str());
	return this;
}

EntityBuilder* EntityBuilder::setRendererDiffuseColor(glm::vec3 diffuseColor) {
	Renderer* renderer = this->getRenderer();
	renderer->diffuseBuffer = loadTextureFromColor(diffuseColor);
	return this;
}

EntityBuilder* EntityBuilder::setRendererDraw(bool draw) {
	Renderer* renderer = this->getRenderer();
	renderer->draw = draw;
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


EntityBuilder* EntityBuilder::setColliderType(colliderType type) {
	unsigned short colliderID = EntityManager::instance->getComponentId(SystemIDs::ColliderID, this->buildEntity->id);
	Collider* collider = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID])->getCollider(colliderID);
	collider->type = type;
	return this;
}
EntityBuilder* EntityBuilder::setColliderPosition(glm::vec3 pos) {
	unsigned short colliderID = EntityManager::instance->getComponentId(SystemIDs::ColliderID, this->buildEntity->id);
	Collider* collider = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID])->getCollider(colliderID);
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
	position = glm::vec3(0.0);
	radius = 1.f;

	std::vector<glm::vec3> list_bis;
	for (size_t i = 0; i < in_vertices.size(); i++) {
		list_bis.push_back(t->applyToVector(in_vertices[i]));
	}

	for (const auto& p : in_vertices) {
		position += p;
	}
	position /= in_vertices.size();

	for (const auto& p : list_bis) {
		radius = std::max(radius, glm::distance(position, p));
	}
}
void computeBox(Transform* t, const std::vector<glm::vec3> in_vertices, glm::vec3& position, glm::vec3& size) {
	glm::vec3 cur_pt;
	cur_pt = t->applyToVector(in_vertices[0]);
	//cur_pt = in_vertices[0];
	float min_dim = 1;

	float min_x = cur_pt.x;
	float max_x = cur_pt.x;
	float min_y = cur_pt.y;
	float max_y = cur_pt.y;
	float min_z = cur_pt.z;
	float max_z = cur_pt.z;

	for (size_t i = 0; i < in_vertices.size(); i++) {
		glm::vec3 cur_pt = in_vertices[i];

		cur_pt = t->applyToVector(cur_pt);

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
	
	dim_x = (dim_x < min_dim) ? min_dim : dim_x;
	dim_y = (dim_y < min_dim) ? min_dim : dim_y;
	dim_z = (dim_z < min_dim) ? min_dim : dim_z;

	position = glm::vec3((max_x + min_x) / 2.0f, (max_y + min_y) / 2.0f, (max_z + min_z) / 2.0f);
	size = glm::vec3(dim_x/2.0f, dim_y / 2.0f, dim_z / 2.0f);
}

void computeSphereDrawElem(std::vector<glm::vec3>& vertices, std::vector<unsigned short>& indices ,const glm::vec3 position, const float radius) {
	vertices.clear();
	int num_segments = 100;

	for (int ii = 0; ii < num_segments; ii++) {
		float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);
		float x = radius * cosf(theta);
		float y = radius * sinf(theta);
		vertices.push_back(glm::vec3(x, y, 0));
	}
	for (int ii = 0; ii < num_segments; ii++) {
		float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);
		float z = radius * cosf(theta);
		float x = radius * sinf(theta);
		vertices.push_back(glm::vec3(x, 0, z));
	}

	for (int ii = 0; ii < num_segments; ii++) {
		float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);
		float z = radius * cosf(theta);
		float y = radius * sinf(theta);
		vertices.push_back(glm::vec3(0, y, z));
	}

	indices.clear();
	indices.push_back(0);
	for (size_t i = 0; i < num_segments; i++) {
		indices.push_back(i);
		indices.push_back(i);
	}
	indices.push_back(0);

	indices.push_back(num_segments);
	for (size_t i = num_segments; i < num_segments * 2; i++) {
		indices.push_back(i);
		indices.push_back(i);
	}
	indices.push_back(num_segments);

	indices.push_back(2*num_segments);
	for (size_t i = 2*num_segments; i < num_segments * 3; i++) {
		indices.push_back(i);
		indices.push_back(i);
	}
	indices.push_back(2*num_segments);
}
void computeBoxDrawElem(std::vector<glm::vec3>& vertices, std::vector<unsigned short>& indices, const glm::vec3& position, const glm::vec3& size) {
	vertices.resize(8);
	glm::vec3 sized = size;
	vertices[0] = glm::vec3(position.x - sized.x, position.y - sized.y, position.z + sized.z);
	vertices[1] = glm::vec3(position.x + sized.x, position.y - sized.y, position.z + sized.z);
	vertices[2] = glm::vec3(position.x + sized.x, position.y + sized.y, position.z + sized.z);
	vertices[3] = glm::vec3(position.x - sized.x, position.y + sized.y, position.z + sized.z);

	vertices[5] = glm::vec3(position.x + sized.x, position.y - sized.y, position.z - sized.z);
	vertices[6] = glm::vec3(position.x + sized.x, position.y + sized.y, position.z - sized.z);
	vertices[7] = glm::vec3(position.x - sized.x, position.y + sized.y, position.z - sized.z);
	vertices[4] = glm::vec3(position.x - sized.x, position.y - sized.y, position.z - sized.z);

	indices.resize(24);
	indices = {
		0, 1, 1, 2, 2, 3, 3, 0,
		4, 5, 5, 6, 6, 7, 7, 4,
		0, 4, 1, 5, 2, 6, 3, 7
	};
}

void initBuffersCollider(Collider* c) {
	if (!c->shader) c->shader = ColliderShader::instance;
	c->shader->use();
	c->shader->init(c);
}

EntityBuilder* EntityBuilder::fitSphereColliderToMesh() {
	unsigned short colliderID = EntityManager::instance->getComponentId(SystemIDs::ColliderID, this->buildEntity->id);
	Collider* collider = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID])->getCollider(colliderID);
	collider->type = colliderType::Sphere;
	computeSphere(this->buildEntity->transform,mesh->indexed_vertices, collider->center, collider->radius);
	return this;
}
EntityBuilder* EntityBuilder::fitAABBColliderToMesh() {
	std::cout << "AABB" << std::endl;

	unsigned short colliderID = EntityManager::instance->getComponentId(SystemIDs::ColliderID, this->buildEntity->id);
	Collider* collider = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID])->getCollider(colliderID);
	collider->type = colliderType::AABB;
	computeBox(this->buildEntity->transform, mesh->indexed_vertices, collider->center, collider->size);

	print(collider->center);
	print(collider->size);

	return this;
}
EntityBuilder* EntityBuilder::fitOBBColliderToMesh() {
	unsigned short colliderID = EntityManager::instance->getComponentId(SystemIDs::ColliderID, this->buildEntity->id);
	Collider* collider = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID])->getCollider(colliderID);
	collider->type = colliderType::OBB;
	computeBox(this->buildEntity->transform, mesh->indexed_vertices, collider->center, collider->size);
	return this;
}
EntityBuilder* EntityBuilder::setRenderingCollider() {
	unsigned short colliderID = EntityManager::instance->getComponentId(SystemIDs::ColliderID, this->buildEntity->id);
	Collider* collider = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID])->getCollider(colliderID);
	if (collider->type == colliderType::Sphere) {
		computeSphereDrawElem(collider->vertices, collider->indices, collider->center, collider->radius);
	}
	else {
		computeBoxDrawElem(collider->vertices, collider->indices, collider->position, collider->size);
	}
	initBuffersCollider(collider);
	collider->drawable = true;

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


EntityBuilder* EntityBuilder::setTranslation(glm::vec3 translation) {
	this->buildEntity->transform->translation = translation;
	return this;
}
EntityBuilder* EntityBuilder::setScale(glm::vec3 scale) {
	this->buildEntity->transform->scaling = scale;
	return this;
}
EntityBuilder* EntityBuilder::setRotation(float angle, glm::vec3 axis) {
	this->buildEntity->transform->rotation.setRotation(angle, axis);
	return this;
}

EntityBuilder* EntityBuilder::setChildOf(Entity* parent) {
	parent->addChildren(this->buildEntity);
	return this;
}

Entity* EntityBuilder::build() {
	return this->buildEntity;
}