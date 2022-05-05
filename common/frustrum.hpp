#ifndef FRUSTRUM_CAMERA_HPP
#define FRUSTRUM_CAMERA_HPP

typedef struct Frustum {

	glm::vec3 top_normal;
	float distance_to_origin_top_plane;
	glm::vec3 bottom_normal;
	float distance_to_origin_bottom_plane;
	glm::vec3 left_normal;
	float distance_to_origin_left_plane;
	glm::vec3 right_normal;
	float distance_to_origin_right_plane;
	glm::vec3 near_normal;
	float distance_to_origin_near_plane;
	glm::vec3 far_normal;
	float distance_to_origin_far_plane;
	
	inline Frustum() { }
} Frustum;

glm::vec3 Intersection(glm::vec3 normal_p1, glm::vec3 normal_p2, glm::vec3 normal_p3, float distance_p1, float distance_p2, float distance_p3);
void getCorners(const Frustum& f, glm::vec3* outCorners);
Frustum ConstructFrustrumFromViewProjection(glm::mat4 viewProjection);

bool IntersectsWithPoint(const Frustum& f, const glm::vec3& p);
bool IntersectsWithSphere(const Frustum& f, const Collider& s);
bool IntersectsWithOBB(const Frustum& f, const Collider& obb);
bool IntersectsWithAABB(const Frustum& f, const Collider& aabb);
bool IntersectsWithCollider(const Frustum& f, const Collider& collider);

std::vector<unsigned short> cullFromOctree(const Frustum& f, std::vector<unsigned short> entitiesID, OctreeNode* octree);
#endif