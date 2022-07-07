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

	// Find the best sibling
	Node* bestSibling = root.get();
	real bestCost = insertionCost(newNode.get(), bestSibling);

	using NodeCostPair = std::pair<Node*, real>;
	auto compare = [](const NodeCostPair& p1, const NodeCostPair& p2)
	{
		// Greater than to ensure that the lowest cost pair is at the top
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
	
	// Create a new parent
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

			oldParent->child1 = std::move(newParent);
		}
		else
		{
			newParent->child1 = std::move(oldParent->child2);
			newParent->child2 = std::move(newNode);

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

	// Refit the AABBs
	refitAABBs(newNodeObserver->parent);
}

void AABBTree::remove(RigidBody* rb)
{
	Node* rbNode = rbNodeMap[rb->id];
	rbNodeMap.erase(rb->id);

	if (rbNode == root.get())
	{
		// If rb is the root, it must be the only node in the tree
		// as each node has either 0 or 2 children
		root.reset();
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
			parent->child2->parent = nullptr;
			root = std::move(parent->child2);
		}
		else
		{
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
	 
	rb->updateFatAABB();

	insert(rb);
}

std::vector<RigidBody*> AABBTree::getPossibleColliders(RigidBody* rb) const
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
				// Avoid checking a pair twice
				if (n->rb->id < rb->id)
				{
					result.push_back(n->rb);
				}
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
	
	// Direct cost
	real cost = toAdd->aabb.unionWith(sibling->aabb).peri();
	
	Node* n = sibling->parent;
	while (n)
	{
		// Inherited cost
		cost += toAdd->aabb.unionWith(n->aabb).peri() - n->aabb.peri();
		n = n->parent;
	}

	return cost;
}

real AABBTree::subTreeLowerBound(Node* toAdd, Node* top)
{
	// Lower bound on direct cost
	real cost = toAdd->aabb.peri();

	Node* n = top;
	while (n)
	{
		// Inherited cost
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

		rotate(n);

		n = n->parent;
	}
}

void AABBTree::rotate(Node* top)
{
	if (top->isLeaf)
	{
		return;
	}

	Node* A = top;
	Node* B = A->child1.get();
	Node* C = A->child2.get();

	if (B->isLeaf && C->isLeaf)
	{
		return;
	}

	real periB = B->aabb.peri();
	real periC = C->aabb.peri();

	std::vector<int> rotations;

	Node* D = nullptr;
	Node* E = nullptr;
	Node* F = nullptr;
	Node* G = nullptr;

	if (!C->isLeaf)
	{
		rotations.push_back(1);
		rotations.push_back(2);

		F = C->child1.get();
		G = C->child2.get();
	}
	if (!B->isLeaf)
	{
		rotations.push_back(3);
		rotations.push_back(4);

		D = B->child1.get();
		E = B->child2.get();
	}

	int bestType = 0;
	real bestDeltaCost = 0;

	for (int type : rotations)
	{
		real deltaCost = 0;

		switch (type)
		{
		case 1:
			deltaCost = B->aabb.unionWith(G->aabb).peri() - periC;
			break;

		case 2:
			deltaCost = B->aabb.unionWith(F->aabb).peri() - periC;
			break;

		case 3:
			deltaCost = C->aabb.unionWith(D->aabb).peri() - periB;
			break;

		case 4:
			deltaCost = C->aabb.unionWith(E->aabb).peri() - periB;
			break;
		}

		if (deltaCost < bestDeltaCost)
		{
			bestDeltaCost = deltaCost;
			bestType = type;
		}
	}

	switch (bestType)
	{
	case 1:
		A->child1.swap(C->child1);
		F->parent = A;
		B->parent = C;
		C->aabb = B->aabb.unionWith(G->aabb);
		break;

	case 2:
		A->child1.swap(C->child2);
		G->parent = A;
		B->parent = C;
		C->aabb = B->aabb.unionWith(F->aabb);
		break;

	case 3:
		A->child2.swap(B->child2);
		E->parent = A;
		C->parent = B;
		B->aabb = C->aabb.unionWith(D->aabb);
		break;

	case 4:
		A->child2.swap(B->child1);
		D->parent = A;
		C->parent = B;
		B->aabb = C->aabb.unionWith(E->aabb);
		break;
	}
}
