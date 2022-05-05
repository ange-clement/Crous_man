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
#include <common/octree.hpp>
#include <queue>
#include <Crous_man/ECS/EntityManager.hpp>

#include "frustrum.hpp"


glm::vec3 Intersection(glm::vec3 normal_p1, glm::vec3 normal_p2, glm::vec3 normal_p3, float distance_p1, float distance_p2, float distance_p3) {
	
	glm::mat3 D;
	D[0] = normal_p1;
	D[1] = normal_p2;
	D[2] = normal_p3;

	glm::vec3 A = glm::vec3(-distance_p1, -distance_p2, -distance_p3);


	glm::mat3 Dx = D, Dy = D, Dz = D;
	Dx[0].x = A.x; Dx[1].x = A.y; Dx[2].x = A.z;
	
	Dy[0].y = A.x; Dy[1].y = A.y; Dy[2].y = A.z;

	Dz[0].z = A.x; Dz[1].z = A.y; Dz[2].z = A.z;

	float detD = glm::determinant(D);

	if (compareWithEpsilon(detD, 0)) {
		return glm::vec3(0);
	}

	float detDx = glm::determinant(Dx);
	float detDy = glm::determinant(Dy);
	float detDz = glm::determinant(Dz);

	return glm::vec3(detDx / detD, detDy / detD, detDz / detD);
	
	/*
	vec3 m1(p1.normal.x, p2.normal.x, p3.normal.x);
	vec3 m2(p1.normal.y, p2.normal.y, p3.normal.y);
	vec3 m3(p1.normal.z, p2.normal.z, p3.normal.z);
	vec3 d(-p1.distance, -p2.distance, -p3.distance);

	vec3 u = Cross(m2, m3);
	vec3 v = Cross(m1, d);
	float denom = Dot(m1, u);
	if (CMP(denom, 0.0f)) {
		return Point();
	}
	Point result;
	result.x = Dot(d, u) / denom;
	result.y = Dot(m3, v) / denom;
	result.z = -Dot(m2, v) / denom;
	return result;*/	
}

void getCorners(const Frustum& f, glm::vec3* outCorners) {
	outCorners[0] = Intersection(
		f.near_normal, 
		f.top_normal, 
		f.left_normal,
		f.distance_to_origin_near_plane,
		f.distance_to_origin_top_plane,
		f.distance_to_origin_left_plane
	);

	outCorners[1] = Intersection(
		f.near_normal, 
		f.top_normal, 
		f.right_normal,
		f.distance_to_origin_near_plane,
		f.distance_to_origin_top_plane,
		f.distance_to_origin_right_plane
	);
	outCorners[2] = Intersection(
		f.near_normal, 
		f.bottom_normal, 
		f.left_normal,
		f.distance_to_origin_near_plane,
		f.distance_to_origin_bottom_plane,
		f.distance_to_origin_left_plane
	);

	outCorners[3] = Intersection(
		f.near_normal, 
		f.bottom_normal, 
		f.right_normal,
		f.distance_to_origin_near_plane,
		f.distance_to_origin_bottom_plane,
		f.distance_to_origin_right_plane
	);
	outCorners[4] = Intersection(
		f.far_normal, 
		f.top_normal, 
		f.left_normal,
		f.distance_to_origin_far_plane,
		f.distance_to_origin_top_plane,
		f.distance_to_origin_left_plane
	);
	outCorners[5] = Intersection(
		f.far_normal, 
		f.top_normal, 
		f.right_normal,
		f.distance_to_origin_far_plane,
		f.distance_to_origin_top_plane,
		f.distance_to_origin_right_plane
	);
	outCorners[6] = Intersection(
		f.far_normal, 
		f.bottom_normal, 
		f.left_normal,
		f.distance_to_origin_far_plane,
		f.distance_to_origin_bottom_plane,
		f.distance_to_origin_left_plane
	);
	outCorners[7] = Intersection(
		f.far_normal, 
		f.bottom_normal, 
		f.right_normal,
		f.distance_to_origin_far_plane,
		f.distance_to_origin_bottom_plane,
		f.distance_to_origin_right_plane
	);
}

Frustum ConstructFrustrumFromViewProjection(glm::mat4 viewProjection) {
	Frustum result;

	glm::vec3 col1 = glm::vec3(viewProjection[0].x, viewProjection[0].y, viewProjection[0].z);
	glm::vec3 col2 = glm::vec3(viewProjection[1].x, viewProjection[1].y, viewProjection[1].z);
	glm::vec3 col3 = glm::vec3(viewProjection[2].x, viewProjection[2].y, viewProjection[2].z);
	glm::vec3 col4 = glm::vec3(viewProjection[3].x, viewProjection[3].y, viewProjection[3].z);

	// Find plane magnitudes
	result.left_normal		= col4 + col1;
	result.right_normal		= col4 - col1;
	result.bottom_normal	= col4 + col2;
	result.top_normal		= col4 - col2;
	result.near_normal		= col4 + col3;
	result.far_normal		= col4 - col3;

	// Find plane distances
	result.distance_to_origin_left_plane	= viewProjection[3].w + viewProjection[0].w;
	result.distance_to_origin_right_plane	= viewProjection[3].w - viewProjection[0].w;
	result.distance_to_origin_bottom_plane	= viewProjection[3].w + viewProjection[1].w;
	result.distance_to_origin_top_plane		= viewProjection[3].w - viewProjection[1].w;
	result.distance_to_origin_near_plane	= viewProjection[3].w + viewProjection[2].w;
	result.distance_to_origin_far_plane		= viewProjection[3].w - viewProjection[2].w;

	// Normalize all 6 planes
	float mag = 1.0f / glm::length(result.left_normal);
	result.left_normal/=mag;
	result.left_normal *= mag;

	mag = 1.0f / glm::length(result.right_normal);
	result.right_normal /= mag;
	result.right_normal *= mag;

	mag = 1.0f / glm::length(result.bottom_normal);
	result.bottom_normal /= mag;
	result.bottom_normal *= mag;

	mag = 1.0f / glm::length(result.top_normal);
	result.top_normal /= mag;
	result.top_normal *= mag;

	mag = 1.0f / glm::length(result.near_normal);
	result.near_normal /= mag;
	result.near_normal *= mag;

	mag = 1.0f / glm::length(result.far_normal);
	result.far_normal /= mag;
	result.far_normal *= mag;
	return result;
}

bool IntersectsWithPoint(const Frustum& f, const glm::vec3& p) {
	glm::vec3 normal = f.top_normal;
	float dist =  f.distance_to_origin_top_plane;
	float side = glm::dot(p, normal) + dist;
	if (side < 0.0f) {
		return false;
	}

	normal = f.bottom_normal;
	dist = f.distance_to_origin_bottom_plane;
	side = glm::dot(p, normal) + dist;
	if (side < 0.0f) {
		return false;
	}
	
	normal = f.left_normal;
	dist = f.distance_to_origin_left_plane;
	side = glm::dot(p, normal) + dist;
	if (side < 0.0f) {
		return false;
	}

	normal =  f.right_normal;
	dist = f.distance_to_origin_right_plane;
	side = glm::dot(p, normal) + dist;
	if (side < 0.0f) {
		return false;
	}

	normal = f.near_normal;
	dist = f.distance_to_origin_near_plane;
	side = glm::dot(p, normal) + dist;
	if (side < 0.0f) {
		return false;
	}

	normal = f.far_normal;
	dist = f.distance_to_origin_far_plane;
	side = glm::dot(p, normal) + dist;
	if (side < 0.0f) {
		return false;
	}

	return true;
}

bool IntersectsWithSphere(const Frustum& f, const Collider& s) {
	assert(s.type == colliderType::Sphere);
	glm::vec3 normal = f.top_normal;
	float dist = f.distance_to_origin_top_plane;
	float side = glm::dot(s.position, normal) + dist;
	if (side < -s.radius) {
		return false;
	}

	normal = f.bottom_normal;
	dist = f.distance_to_origin_bottom_plane;
	side = glm::dot(s.position, normal) + dist;
	if (side < -s.radius) {
		return false;
	}

	normal = f.left_normal;
	dist = f.distance_to_origin_left_plane;
	side = glm::dot(s.position, normal) + dist;
	if (side < -s.radius) {
		return false;
	}

	normal = f.right_normal;
	dist = f.distance_to_origin_right_plane;
	side = glm::dot(s.position, normal) + dist;
	if (side < -s.radius) {
		return false;
	}

	normal = f.near_normal;
	dist = f.distance_to_origin_near_plane;
	side = glm::dot(s.position, normal) + dist;
	if (side < -s.radius) {
		return false;
	}

	normal = f.far_normal;
	dist = f.distance_to_origin_far_plane;
	side = glm::dot(s.position, normal) + dist;
	if (side < -s.radius) {
		return false;
	}

	return true;
}

float ClassifyAABB(const Collider& aabb, glm::vec3 normal_plane, float distance) {
	assert(aabb.type == colliderType::AABB);
	// maximum extent in direction of plane normal 
	float r = 
		std::abs(aabb.size.x * normal_plane.x)+
		std::abs(aabb.size.y * normal_plane.y)+
		std::abs(aabb.size.z * normal_plane.z);

	// signed distance between box center and plane
	//float d = plane.Test(mCenter);
	float d = glm::dot(normal_plane, aabb.position) + distance;

	// return signed distance
	if (std::abs(d) < r) {
		return 0.0f;
	}
	else if (d < 0.0f) {
		return d + r;
	}
	return d - r;
}

float ClassifyOBB(const Collider& obb, glm::vec3 normal_plane, float distance) {
	assert(obb.type == colliderType::OBB);

	// we first transform the normal of the plane into the local space of the OBB
	glm::vec3 normal = projectV3OnM3(normal_plane , obb.orientation);

	// maximum extent in direction of plane normal 
	float r = 
		std::abs(obb.size.x * normal.x) +
		std::abs(obb.size.y * normal.y) +
		std::abs(obb.size.z * normal.z);

	// signed distance between box center and plane
	//float d = plane.Test(mCenter);
	float d = glm::dot(normal_plane, obb.position) + distance;

	// return signed distance
	if (std::abs(d) < r) {
		return 0.0f;
	}
	else if (d < 0.0f) {
		return d + r;
	}
	return d - r;
}


bool IntersectsWithOBB(const Frustum& f, const Collider& obb) {
	assert(obb.type == colliderType::OBB);
	glm::vec3 normal = f.top_normal;
	float dist = f.distance_to_origin_top_plane;
	float side = ClassifyOBB(obb,normal,dist);
	if (side < 0.0f) {
		return false;
	}

	normal = f.bottom_normal;
	dist = f.distance_to_origin_bottom_plane;
	side = ClassifyOBB(obb, normal, dist);
	if (side < 0.0f) {
		return false;
	}

	normal = f.left_normal;
	dist = f.distance_to_origin_left_plane;
	side = ClassifyOBB(obb, normal, dist);
	if (side < 0.0f) {
		return false;
	}

	normal = f.right_normal;
	dist = f.distance_to_origin_right_plane;
	side = ClassifyOBB(obb, normal, dist);
	if (side < 0.0f) {
		return false;
	}

	normal = f.near_normal;
	dist = f.distance_to_origin_near_plane;
	side = ClassifyOBB(obb, normal, dist);
	if (side < 0.0f) {
		return false;
	}

	normal = f.far_normal;
	dist = f.distance_to_origin_far_plane;
	side = ClassifyOBB(obb, normal, dist);
	if (side < 0.0f) {
		return false;
	}

	return true;
}

bool IntersectsWithAABB(const Frustum& f, const Collider& aabb) {
	assert(aabb.type == colliderType::AABB);

	glm::vec3 normal = f.top_normal;
	float dist = f.distance_to_origin_top_plane;
	float side = ClassifyAABB(aabb, normal, dist);
	if (side < 0.0f) {
		return false;
	}

	normal = f.bottom_normal;
	dist = f.distance_to_origin_bottom_plane;
	side = ClassifyAABB(aabb, normal, dist);
	if (side < 0.0f) {
		return false;
	}

	normal = f.left_normal;
	dist = f.distance_to_origin_left_plane;
	side = ClassifyAABB(aabb, normal, dist);
	if (side < 0.0f) {
		return false;
	}

	normal = f.right_normal;
	dist = f.distance_to_origin_right_plane;
	side = ClassifyAABB(aabb, normal, dist);
	if (side < 0.0f) {
		return false;
	}

	normal = f.near_normal;
	dist = f.distance_to_origin_near_plane;
	side = ClassifyAABB(aabb, normal, dist);
	if (side < 0.0f) {
		return false;
	}

	normal = f.far_normal;
	dist = f.distance_to_origin_far_plane;
	side = ClassifyAABB(aabb, normal, dist);
	if (side < 0.0f) {
		return false;
	}

	return true;
}

bool IntersectsWithCollider(const Frustum& f, const Collider& collider) {
	if (collider.type == colliderType::AABB) IntersectsWithAABB(f, collider);
	if (collider.type == colliderType::OBB) IntersectsWithOBB(f, collider);
	if (collider.type == colliderType::Sphere) IntersectsWithSphere(f, collider);
	return false;
}

//FOR OCTREE CULLING
std::vector<unsigned short> cullFromOctree(const Frustum& f, std::vector<unsigned short> entitiesID, OctreeNode* octree) {
	std::vector<unsigned short> result;

	bool* flag = new bool[entitiesID.size()]{};

	std::queue<OctreeNode*> nodes;
	nodes.push(octree);


	ColliderSystem* CS = dynamic_cast<ColliderSystem*>(EntityManager::instance->systems[SystemIDs::ColliderID]);


	if (CS) {
		while (nodes.size() > 0) {
			OctreeNode* active = nodes.front();
			nodes.pop();

			// Has child nodes
			if (active->children != 0) {
				for (int i = 0; i < 8; ++i) {
					Collider aabb = active->children[i].aabb;
					if (IntersectsWithCollider(f, aabb)) {
						nodes.push(&active->children[i]);
					}
				}
			}
			else {
				// Is leaf node
				for (int i = 0, size = active->entitiesID.size(); i < size; ++i) {
					if (!flag[i]) {

						Collider* col = CS->getColliderEntityID(active->entitiesID[i]);
						if (col) {
							if (IntersectsWithCollider(f,*col)){
								flag[i] = true;
								result.push_back(active->entitiesID[i]);
							}
						}
						else {
							std::cout << "IMPOSSIBLE TO WORK WITH ENTITY : " << active->entitiesID[i] << " NO COLLIDER FOUND" << std::endl;
						}
					}
				}
			}
		}
	}
	else {
		std::cout << "NEED A COLLIDER SYSTEM TO PERFORM OCTREE SCENE CAMERA CULLING" << std::endl;
	}
	

	return result;
}