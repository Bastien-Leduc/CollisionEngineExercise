#pragma once
#include "AABB.h"

typedef struct  AABBTreeNode
{
	AABBTreeNode();
	~AABBTreeNode();

	AABBTreeNode* parent;
	AABBTreeNode* children[2];
	AABB* fatAABB;
	AABB* polyAABB;
	bool crossed = false;;

	bool IsLeaf();
	void SetAsBranch(AABBTreeNode* child1, AABBTreeNode* child2);
	void SetAsLeaf(AABB* userBox);
	AABBTreeNode* GetSibling() const;
} AABBTreeNode;
