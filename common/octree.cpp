#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <Crous_man/Util.hpp>
#include <Crous_man/Components/Collider.hpp>
#include <Crous_man/ECS/Bitmap.hpp>

#include <Crous_man/ECS/EntityManager.hpp>
#include <common/ray.hpp>
#include "octree.hpp"


void splitTree(OctreeNode& node, int depth){
	// FIRST STEP :  Decrements depth
	if (depth-- <= 0) {
		//Stop spliting on max depth reach
		return;
	}

	//If this node dont have any childrens, we can perform the 8-spliting
	if (node.children == 0) {
		node.children = new OctreeNode[8];

		glm::vec3 c = node.aabb.position;
		//Get the mid size of the current AABB
		glm::vec3 e = node.aabb.size * 0.5f;

		node.children[0].aabb.position = c + glm::vec3(-e.x, +e.y, -e.z);
		node.children[0].aabb.size = e;
		node.children[0].aabb.type = colliderType::AABB;

		node.children[1].aabb.position = c + glm::vec3(+e.x, +e.y, -e.z);
		node.children[1].aabb.size = e;
		node.children[1].aabb.type = colliderType::AABB;

		node.children[2].aabb.position = c + glm::vec3(-e.x, +e.y, +e.z);
		node.children[2].aabb.size = e;
		node.children[2].aabb.type = colliderType::AABB;

		node.children[3].aabb.position = c + glm::vec3(+e.x, +e.y, +e.z);
		node.children[3].aabb.size = e;
		node.children[3].aabb.type = colliderType::AABB;

		node.children[4].aabb.position = c + glm::vec3(-e.x, -e.y, -e.z);
		node.children[4].aabb.size = e;
		node.children[4].aabb.type = colliderType::AABB;

		node.children[5].aabb.position = c + glm::vec3(+e.x, -e.y, -e.z);
		node.children[5].aabb.size = e;
		node.children[5].aabb.type = colliderType::AABB;

		node.children[6].aabb.position = c + glm::vec3(-e.x, -e.y, +e.z);
		node.children[6].aabb.size = e;
		node.children[6].aabb.type = colliderType::AABB;

		node.children[7].aabb.position = c + glm::vec3(+e.x, -e.y, +e.z);
		node.children[7].aabb.size = e;
		node.children[7].aabb.type = colliderType::AABB;
	}

	//We need to split all models on nodes
	if (node.children != 0 && node.entitiesID.size() > 0) {
		ColliderSystem* CS = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID]);
		
		if (CS) {
			for (int i = 0; i < 8; ++i) { // For each child
				for (int j = 0, size = node.entitiesID.size(); j < size; ++j) {
					Collider* col = CS->getColliderEntityID(node.entitiesID[j]);
					if (col) {
						if (intersect(node.children[i].aabb, *col)) {
							node.children[i].entitiesID.push_back(node.entitiesID[j]);
						}
					}
					else {
						std::cout << "IMPOSSIBLE TO WORK WITH ENTITY : " << node.entitiesID[j] << " NO COLLIDER FOUND" << std::endl;
					}
				}
			}
			//We splited all childrens, we can clear current node splited entities list
			node.entitiesID.clear();

			// Recurse
			for (int i = 0; i < 8; ++i) {
				splitTree(node.children[i], depth);
			}
		}
		else {
			std::cout << "NEED A COLLIDER SYSTEM TO PERFORM OCTREE SCENE SPLIT" << std::endl;
		}
	}
}

void insertInTheTree(OctreeNode& node, unsigned short id) {
	ColliderSystem* CS = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID]);

	if (CS) {
		Collider* col = CS->getColliderEntityID(id);	
		if (col) {
			if (intersect(node.aabb, *col)) {
				if (node.children == 0) {
					node.entitiesID.push_back(id);
				}
				else {
					for (int i = 0; i < 8; ++i) {
						insertInTheTree(node.children[i], id);
					}
				}
			}
		}
		else {
			std::cout << "IMPOSSIBLE TO INSERT THIS ENTITY, NO COLLIDER FOUND" << std::endl;
		}
	}
	else {
		std::cout << "NEED A COLLIDER SYSTEM TO PERFORM OCTREE SCENE INSERT" << std::endl;
	}
}

void removeToTheTree(OctreeNode& node, unsigned short id) {
	if (node.children == 0) {
		std::vector<unsigned short>::iterator it = std::find(node.entitiesID.begin(), node.entitiesID.end(), id);
		if (it != node.entitiesID.end()) {
			node.entitiesID.erase(it);
		}
	}
	else {
		for (int i = 0; i < 8; ++i) {
			removeToTheTree(node.children[i], id);
		}
	}
}

void updateTree(OctreeNode& node, unsigned short id) {
	removeToTheTree(node, id);
	insertInTheTree(node, id);
}

// returns the closest model to the origin of the given ray
// works by comparing the t values of a raycast between the ray and each object
unsigned short findClosest(const std::vector<unsigned short>& set, const Ray& ray){
	if (set.size() == 0) {
		return 0;
	}

	unsigned short closest = -1;
	float closest_t = -1;

	ColliderSystem* CS = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID]);
	RaycastResult res;
	if (CS) {
		for (int i = 0, size = set.size(); i < size; ++i) {
			Collider* col = CS->getColliderEntityID(set[i]);
			if (col) {
				if (RayCastCollider(*col, ray, &res)) {
					float this_t = res.t;
					if (this_t < 0) {
						continue;
					}
					if (closest_t < 0 || this_t < closest_t) {
						closest_t = this_t;
						closest = set[i];
					}
				}
			}
			else {
				std::cout << "IMPOSSIBLE TO WORK WITH ENTITY : " << set[i] << " NO COLLIDER FOUND" << std::endl;
			}
		}
	}
	else {
		std::cout << "NEED A COLLIDER SYSTEM TO PERFORM OCTREE SCENE FIND CLOSEST" << std::endl;
	}

	return closest;
}

// The closest object from the recursive result on node bound is returned
unsigned short raycastOnANode(OctreeNode* node, const Ray& ray) {
	RaycastResult raycast;
	RayCastCollider(node->aabb, ray, &raycast);
	float t = raycast.t;

	if (t >= 0) {
		if (node->children == 0) {
			return findClosest(node->entitiesID, ray);
		}
		else {
			std::vector<unsigned short> results;
			for (int i = 0; i < 8; ++i) {

				//Recursive call to all childrens
				unsigned short result = raycastOnANode(&(node->children[i]), ray);
				if (result != 0) {
					results.push_back(result);
				}
			}
			return findClosest(results, ray);
		}
	}

	return 0;
}


std::vector<unsigned short> queryOnSphere(OctreeNode* node, const Collider& sphere) {
	assert(sphere.type == colliderType::Sphere);

	std::vector<unsigned short> result;
	RaycastResult resRay;

	ColliderResult* res = SphereAABBCollision(node->aabb, sphere);

	ColliderSystem* CS = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID]);
	if (CS) {
		if (res->isInCollision) {

			if (node->children == 0) {
				for (int i = 0, size = node->entitiesID.size(); i < size; ++i) {

					Collider* col = CS->getColliderEntityID(node->entitiesID[i]);
					if (col) {
						if (intersect(sphere, *col)) {
							result.push_back(node->entitiesID[i]);
						}
					}
					else {
						std::cout << "IMPOSSIBLE TO WORK WITH ENTITY : " << node->entitiesID[i] << " NO COLLIDER FOUND" << std::endl;
					}
				}
			}
			else {
				//Recursive call to all childrens 
				for (int i = 0; i < 8; ++i) {
					// recursively collect all objects which intersect the query sphere
					std::vector<unsigned short> child = queryOnSphere(&node->children[i], sphere);
					if (child.size() > 0) {
						result.insert(result.end(), child.begin(), child.end());
					}
				}
			}
		}
	}
	else {
		std::cout << "NEED A COLLIDER SYSTEM TO PERFORM OCTREE SCENE QUERY ON SPHERE" << std::endl;
	}

	return result;
}

std::vector<unsigned short> queryOnAABB(OctreeNode* node, const Collider& aabb) {
	assert(aabb.type == colliderType::AABB);

	std::vector<unsigned short> result;
	RaycastResult resRay;

	ColliderResult* res = AABBAABBCollision(node->aabb, aabb);

	ColliderSystem* CS = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID]);
	if (CS) {
		if (res->isInCollision) {

			if (node->children == 0) {
				for (int i = 0, size = node->entitiesID.size(); i < size; ++i) {

					Collider* col = CS->getColliderEntityID(node->entitiesID[i]);
					if (col) {
						if (intersect(aabb, *col)) {
							result.push_back(node->entitiesID[i]);
						}
					}
					else {
						std::cout << "IMPOSSIBLE TO WORK WITH ENTITY : " << node->entitiesID[i] << " NO COLLIDER FOUND" << std::endl;
					}
				}
			}
			else {
				//Recursive call to all childrens 
				for (int i = 0; i < 8; ++i) {
					// recursively collect all objects which intersect the query sphere
					std::vector<unsigned short> child = queryOnAABB(&node->children[i], aabb);
					if (child.size() > 0) {
						result.insert(result.end(), child.begin(), child.end());
					}
				}
			}
		}
	}
	else {
		std::cout << "NEED A COLLIDER SYSTEM TO PERFORM OCTREE SCENE QUERY ON AABB" << std::endl;
	}

	return result;
}

/*
bool accelerate(const glm::vec3& position, float size) {
	if (octree != 0) {
		return false;
	}

	vec3 min(position.x - size, position.y - size, position.z - size);
	vec3 max(position.x + size, position.y + size, position.z + size);

	// Construct tree root
	octree = new OctreeNode();
	octree->bounds = FromMinMax(min, max);
	octree->children = 0;
	for (int i = 0, size = objects.size(); i < size; ++i) {
		octree->models.push_back(objects[i]);
	}

	// Create tree
	SplitTree(octree, 5);
	return true;
}*/
