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
}


EntityBuilder* EntityBuilder::setMeshAsQuad() {
	unsigned short meshID = EntityManager::instance->getComponentId(SystemIDs::MeshID, this->buildEntity->id);
	Mesh* mesh = &EntityManager::instance->meshComponents[meshID];
	quad(mesh->indexed_vertices, mesh->normals, mesh->UV, mesh->indices, mesh->triangles);
	return this;
}

EntityBuilder* EntityBuilder::setMeshAsFile(std::string meshFile, bool fileHasNormals) {
	unsigned short meshID = EntityManager::instance->getComponentId(SystemIDs::MeshID, this->buildEntity->id);
	EntityManager::instance->meshComponents[meshID].loadFromFile(meshFile, false);
	return this;
}

EntityBuilder* EntityBuilder::setMeshAsFilePLY(std::string meshFile) {
	return setMeshAsFilePLY(meshFile, false);
}
EntityBuilder* EntityBuilder::setMeshAsFilePLY(std::string meshFile, bool invertTriangles) {
	unsigned short meshID = EntityManager::instance->getComponentId(SystemIDs::MeshID, this->buildEntity->id);
	EntityManager::instance->meshComponents[meshID].loadFromFilePLY(meshFile, invertTriangles);
	return this;
}


EntityBuilder* EntityBuilder::updateRenderer() {
	unsigned short rendererID = EntityManager::instance->getComponentId(SystemIDs::RendererID, this->buildEntity->id);
	RendererSystem* rendererSystem = dynamic_cast<RendererSystem*>(EntityManager::instance->systems[SystemIDs::RendererID]);
	rendererSystem->initBuffers(rendererID, this->buildEntity->id);
	return this;
}

EntityBuilder* EntityBuilder::setRendererDiffuseSpecular(std::string diffuseFile, std::string specularFile) {
	unsigned short rendererID = EntityManager::instance->getComponentId(SystemIDs::RendererID, this->buildEntity->id);
	RendererSystem* rendererSystem = dynamic_cast<RendererSystem*>(EntityManager::instance->systems[SystemIDs::RendererID]);
	Renderer* renderer = rendererSystem->getRenderer(rendererID);
	renderer->diffuseBuffer = loadTextureFromPPM("../ressources/earth.ppm");
	renderer->specularBuffer = loadTextureFromPGM("../ressources/heightmap.pgm");
	return this;
}

EntityBuilder* EntityBuilder::setRendererDiffuseColor(glm::vec3 diffuseColor) {
	unsigned short rendererID = EntityManager::instance->getComponentId(SystemIDs::RendererID, this->buildEntity->id);
	RendererSystem* rendererSystem = dynamic_cast<RendererSystem*>(EntityManager::instance->systems[SystemIDs::RendererID]);
	Renderer* renderer = rendererSystem->getRenderer(rendererID);
	renderer->diffuseBuffer = loadTextureFromColor(diffuseColor);
	return this;
}

EntityBuilder* EntityBuilder::setRendererDraw(bool draw) {
	unsigned short rendererID = EntityManager::instance->getComponentId(SystemIDs::RendererID, this->buildEntity->id);
	RendererSystem* rendererSystem = dynamic_cast<RendererSystem*>(EntityManager::instance->systems[SystemIDs::RendererID]);
	Renderer* renderer = rendererSystem->getRenderer(rendererID);
	renderer->draw = draw;
	return this;
}

EntityBuilder* EntityBuilder::addDestructibleMeshes(std::initializer_list<std::string> meshesFiles) {
	return addDestructibleMeshes(meshesFiles, false);
}
EntityBuilder* EntityBuilder::addDestructibleMeshes(std::initializer_list<std::string> meshesFiles, bool invertTriangles) {
	unsigned short destructibleID = EntityManager::instance->getComponentId(SystemIDs::DestructibleID, this->buildEntity->id);
	Destructible* destructible = dynamic_cast<DestructibleSystem*>(EntityManager::instance->systems[SystemIDs::DestructibleID])->getDestructible(destructibleID);
	for (std::string meshFile : meshesFiles) {
		destructible->fragmentMeshFiles.push_back(meshFile);
		destructible->fragmentMeshInvertTriangle.push_back(invertTriangles);
	}
	return this;
}


EntityBuilder* EntityBuilder::setLightColor(glm::vec3 color) {
	unsigned short pointLightID = EntityManager::instance->getComponentId(SystemIDs::PointLightID, this->buildEntity->id);
	PointLight* PL = dynamic_cast<PointLightSystem*>(EntityManager::instance->systems[SystemIDs::PointLightID])->getPointLight(pointLightID);
	PL->color = color;
	return this;
}

EntityBuilder* EntityBuilder::setLightLinear(float linear) {
	unsigned short pointLightID = EntityManager::instance->getComponentId(SystemIDs::PointLightID, this->buildEntity->id);
	PointLight* PL = dynamic_cast<PointLightSystem*>(EntityManager::instance->systems[SystemIDs::PointLightID])->getPointLight(pointLightID);
	PL->linear = linear;
	return this;
}

EntityBuilder* EntityBuilder::setLightQuadratic(float quadratic) {
	unsigned short pointLightID = EntityManager::instance->getComponentId(SystemIDs::PointLightID, this->buildEntity->id);
	PointLight* PL = dynamic_cast<PointLightSystem*>(EntityManager::instance->systems[SystemIDs::PointLightID])->getPointLight(pointLightID);
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