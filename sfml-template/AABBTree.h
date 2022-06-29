#pragma once
#include "Utils.h"
#include "RigidBody.h"
#include <unordered_map>

struct Node
{
	RigidBody* rb = nullptr;
	AABB aabb;

	bool isLeaf = false;

	Node* parent = nullptr;
	std::unique_ptr<Node> child1 = nullptr;
	std::unique_ptr<Node> child2 = nullptr;
};


class AABBTree
{
public:
	void insert(RigidBody* rb);
	void remove(RigidBody* rb);
	void update(RigidBody* rb);

	std::forward_list<RigidBody*> getPossibleColliders(RigidBody* rb);

	int count();

private:
	real insertionCost(Node* toAdd, Node* sibling);
	real subTreeLowerBound(Node* toAdd, Node* top);
	void refitAABBs(Node* start);
	void rotate(Node* top);

	std::unique_ptr<Node> root;

	// TODO: unordered_map?
	std::map<idType, Node*> rbNodeMap;
};

