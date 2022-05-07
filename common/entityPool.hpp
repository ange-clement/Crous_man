#ifndef ENITY_POOL_HPP
#define ENITY_POOL_HPP

class Entity;

class EntityPool {
public:
	Entity** pool;
	unsigned int maxNumberOfObjects;
	unsigned int currentInstancedObject;
public:
	EntityPool(std::vector<Entity*> entities);
	~EntityPool();

	Entity* addEntity();
	void addEntity(unsigned int number);
	void deleteEntity();
	void deleteEntity(unsigned int number);
};

#endif