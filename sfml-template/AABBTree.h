#pragma once
#include "Utils.h"
#include "RigidBody.h"
#include <unordered_map>

class AABBTree
{
public:
	void insert(RigidBody* rb);
	void remove(RigidBody* rb);
	void update(RigidBody* rb);

	std::vector<RigidBody*> getPossibleColliders(RigidBody* rb) const;
	std::vector<RigidBody*> getPossibleContainers(const vec2& p) const;

	int count();

private:
	struct Node;

	real insertionCost(Node* toAdd, Node* sibling);
	real subTreeLowerBound(Node* toAdd, Node* top);
	void refitAABBs(Node* start);
	void rotate(Node* top);

	std::unique_ptr<Node> root;
	std::map<idType, Node*> rbNodeMap;
};

struct AABBTree::Node
{
	RigidBody* rb = nullptr;
	AABB aabb;

	bool isLeaf = false;

	Node* parent = nullptr;
	std::unique_ptr<Node> child1 = nullptr;
	std::unique_ptr<Node> child2 = nullptr;
};