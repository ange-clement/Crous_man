#ifndef OCTREE_SCENE_HPP
#define OCTREE_SCENE_HPP

struct Collider;
class Ray;

/*Basic Octree structure for scene spliting*/

typedef struct OctreeNode {
	Collider aabb;
	OctreeNode* children;
	std::vector<unsigned short> entitiesID;

	void destroy() {
		if (children != 0) {
			children->destroy();
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

#endif