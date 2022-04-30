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
	renderer->diffuseBuffer = loadTextureFromPPM("../ressources/earth.ppm");
	renderer->specularBuffer = loadTextureFromPGM("../ressources/heightmap.pgm");
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