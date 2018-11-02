#include "AABBTreeNode.h"

AABBTreeNode::AABBTreeNode()
{
	parent = nullptr;
	children[0] = children[1] = nullptr;
	polyAABB = nullptr;
	fatAABB = new AABB;
}

AABBTreeNode::~AABBTreeNode()
{
	parent = nullptr;

	if (children[0])
		delete children[0];
	if (children[1])
		delete children[1];

	children[0] = children[1] = nullptr;
	delete polyAABB;
	delete fatAABB;
}

bool AABBTreeNode::IsLeaf()
{
	return !children[0] && !children[1];
}

void AABBTreeNode::SetAsBranch(AABBTreeNode* child1, AABBTreeNode* child2)
{
	child1->parent = this;
	child2->parent = this;

	children[0] = child1;
	children[1] = child2;
}

void AABBTreeNode::SetAsLeaf(AABB* userBox)
{
	polyAABB = userBox;
	children[0] = children[1] = nullptr;
}


AABBTreeNode* AABBTreeNode::GetSibling() const
{
	return this == parent->children[0] ? parent->children[1] : parent->children[0];
}
