#ifndef OCTREE_SCENE_HPP
#define OCTREE_SCENE_HPP

struct Collider;
class Ray;

/*Basic Octree structure for scene spliting*/

typedef struct OctreeNode {
	Collider aabb;
	OctreeNode* children;
	std::vector<unsigned short> entitiesID;

	inline OctreeNode() : children(0) { }
	inline ~OctreeNode() {
		if (children != 0) {
			delete[] children;
		}
	}

} OctreeNode;


void splitTree(OctreeNode& node, int depth);
void insertInTheTree(OctreeNode& node, unsigned short id);
void removeToTheTree(OctreeNode& node, unsigned short id);
void updateTree(OctreeNode& node, unsigned short id);


unsigned short findClosest(const std::vector<unsigned short>& set, const Ray& ray);
unsigned short raycastOnANode(OctreeNode* node, const Ray& ray);

std::vector<unsigned short> queryOnSphere(OctreeNode* node, const Collider& collider);
std::vector<unsigned short> queryOnAABB(OctreeNode* node, const Collider& collider);

OctreeNode* constructOctree(std::vector<unsigned short> objects, const glm::vec3& position, float size, int depth = 5);
#endif