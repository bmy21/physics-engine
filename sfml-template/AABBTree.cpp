#include "AABBTree.h"
#include <stack>
#include <queue>

void AABBTree::insert(RigidBody* rb)
{
	std::unique_ptr<Node> newNode = std::make_unique<Node>();
	Node* newNodeObserver = newNode.get();
	rbNodeMap.insert({ rb->id, newNodeObserver });

	newNode->rb = rb;
	newNode->aabb = rb->getFatAABB();
	newNode->isLeaf = true;

	if (!root)
	{
		root = std::move(newNode);
		return;
	}

	// find best sibling
	Node* bestSibling = root.get();
	real bestCost = insertionCost(newNode.get(), bestSibling);

	using NodeCostPair = std::pair<Node*, real>;
	auto compare = [](const NodeCostPair& p1, const NodeCostPair& p2)
	{
		// greater than to ensure that the lowest cost pair is at the top
		return p1.second > p2.second;
	};

	std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, decltype(compare)> pq(compare);
	pq.push({ bestSibling, bestCost });

	while (!pq.empty())
	{
		Node* n = pq.top().first;
		pq.pop();

		real cost = insertionCost(newNode.get(), n);
		if (cost < bestCost)
		{
			bestCost = cost;
			bestSibling = n;
		}

		if (!n->isLeaf)
		{
			real lb = subTreeLowerBound(newNode.get(), n);
			if (lb < bestCost)
			{
				pq.push({ n->child1.get(), lb });
				pq.push({ n->child2.get(), lb });
			}
		}
	}
	

	// create new parent
	Node* oldParent = bestSibling->parent;
	std::unique_ptr<Node> newParent = std::make_unique<Node>();
	newParent->parent = oldParent;
	newParent->aabb = newNode->aabb.unionWith(bestSibling->aabb); 

	bestSibling->parent = newParent.get();
	newNode->parent = newParent.get();

	if (oldParent)
	{
		// Put the new parent in place of the sibling
		if (oldParent->child1.get() == bestSibling)
		{
			newParent->child1 = std::move(oldParent->child1);
			newParent->child2 = std::move(newNode);

			//oldParent->child1.release();
			oldParent->child1 = std::move(newParent);
		}
		else
		{
			newParent->child1 = std::move(oldParent->child2);
			newParent->child2 = std::move(newNode);

			//oldParent->child2.release();
			oldParent->child2 = std::move(newParent);
		}
	}
	else
	{
		// bestSibling's parent was nullptr, i.e. bestSibling was the root
		newParent->child1 = std::move(root);
		newParent->child2 = std::move(newNode);
		root = std::move(newParent);
	}

	
	// refit aabbs
	refitAABBs(newNodeObserver->parent);
}

void AABBTree::remove(RigidBody* rb)
{
	Node* rbNode = rbNodeMap[rb->id];
	rbNodeMap.erase(rb->id);

	//std::cout << root.get() << "\n";
	//std::cout << rbNode << "\n";

	if (rbNode == root.get())
	{
		// If rb is the root, it must be the only node in the tree
		// as each node has either 0 or 2 children
		root.reset();

		//std::cout << "resetting root\n";
		return;
	}

	Node* parent = rbNode->parent;
	Node* grandparent = parent->parent;

	if (grandparent)
	{
		// Make the sibling's current grandparent its new parent
		if (grandparent->child1.get() == parent)
		{
			if (parent->child1.get() == rbNode)
			{
				grandparent->child1 = std::move(parent->child2);
			}
			else
			{
				grandparent->child1 = std::move(parent->child1);
			}

			grandparent->child1->parent = grandparent;
		}
		else
		{
			if (parent->child1.get() == rbNode)
			{
				grandparent->child2 = std::move(parent->child2);
			}
			else
			{
				grandparent->child2 = std::move(parent->child1);
			}

			grandparent->child2->parent = grandparent;
		}

		refitAABBs(grandparent);
	} 
	else
	{
		// No grandparent, so sibling becomes the root
		if (parent->child1.get() == rbNode)
		{
			//root->child2.release();
			parent->child2->parent = nullptr;
			root = std::move(parent->child2);
		}
		else
		{
			//root->child1.release();
			parent->child1->parent = nullptr;
			root = std::move(parent->child1);
		}
	}
}

void AABBTree::update(RigidBody* rb)
{
	Node* rbNode = rbNodeMap[rb->id];
	if (rbNode->aabb.contains(rb->getAABB()))
	{
		return;
	}

	remove(rb);

	rb->updateFatAABB(0.05);

	insert(rb);
}

std::vector<RigidBody*> AABBTree::getPossibleColliders(RigidBody* rb)
{
	std::vector<RigidBody*> result;

	std::stack<Node*> s;
	if (root)
	{
		s.push(root.get());
	}

	while (!s.empty())
	{
		Node* n = s.top();
		s.pop();

		if (rb != n->rb && n->aabb.overlaps(rb->getAABB()))
		{
			if (n->isLeaf)
			{
				result.push_back(n->rb);
			}
			else
			{
				s.push(n->child1.get());
				s.push(n->child2.get());
			}
		}
	}

	return result;
}

int AABBTree::count()
{
	int result = 0;

	std::stack<Node*> s;
	if (root)
	{
		s.push(root.get());
	}
	

	while (!s.empty())
	{
		Node* n = s.top();
		s.pop();

		if (n->isLeaf)
		{
			++result;
		}
		else
		{
			s.push(n->child1.get());
			s.push(n->child2.get());
		}
	}

	return result;
}

real AABBTree::insertionCost(Node* toAdd, Node* sibling)
{
	// How much extra perimeter would be added by inserting toAdd next to a given sibling?
	
	// direct cost
	real cost = toAdd->aabb.unionWith(sibling->aabb).peri();
	
	Node* n = sibling->parent;
	while (n)
	{
		// inherited cost
		cost += toAdd->aabb.unionWith(n->aabb).peri() - n->aabb.peri();
		n = n->parent;
	}

	return cost;
}

real AABBTree::subTreeLowerBound(Node* toAdd, Node* top)
{
	// lower bound on direct cost
	real cost = toAdd->aabb.peri();

	Node* n = top;
	while (n)
	{
		// inherited cost
		cost += toAdd->aabb.unionWith(n->aabb).peri() - n->aabb.peri();
		n = n->parent;
	}

	return cost;
}

void AABBTree::refitAABBs(Node* start)
{
	Node* n = start;
	while (n)
	{
		n->aabb = n->child1->aabb.unionWith(n->child2->aabb);
		n = n->parent;
	}
}

void AABBTree::rotate(Node* top)
{
}
