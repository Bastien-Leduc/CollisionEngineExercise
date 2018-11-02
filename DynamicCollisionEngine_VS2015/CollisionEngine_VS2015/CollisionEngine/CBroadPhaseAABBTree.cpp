#include "CBroadPhaseAABBTree.h"
#include "GlobalVariables.h"
#include "World.h"
#include "Renderer.h"
#include <string>

CBroadPhaseAABBTree::CBroadPhaseAABBTree()
{
}


CBroadPhaseAABBTree::~CBroadPhaseAABBTree()
{
	delete m_root;
	m_root = nullptr;
}

void CBroadPhaseAABBTree::Init()
{
	const size_t polyCount = gVars->pWorld->GetPolygonCount();
	for (size_t index = 0; index < polyCount; ++index)
	{
		const CPolygonPtr poly = gVars->pWorld->GetPolygon(index);
		Add(BuildPolyAABB(poly));
	}
}

void CBroadPhaseAABBTree::GetCollidingPairsToCheck(std::vector<SPolygonPair>& pairsToCheck)
{
	m_nodePairs.clear();

	if (!m_root) Init();

	if (m_root->IsLeaf()) return;

	ClearCrossFlag(m_root);

	Update();

	ComputePairs(m_root->children[0], m_root->children[1]);

	const std::string str = "Potential Pairs : " + std::to_string(m_nodePairs.size());
	gVars->pRenderer->DisplayText(str, 50, 150);

	pairsToCheck = m_nodePairs;
}

void CBroadPhaseAABBTree::InsertNode(AABBTreeNode* node, AABBTreeNode** parent) const
{
	AABBTreeNode* refParent = *parent;
	if (refParent->IsLeaf())
	{
		AABBTreeNode* newParent = new AABBTreeNode;
		newParent->parent = refParent->parent;
		newParent->SetAsBranch(node, refParent);
		*parent = newParent;
	}
	else
	{
		AABB* aabb0 = refParent->children[0]->fatAABB;
		AABB* aabb1 = refParent->children[1]->fatAABB;

		const float volumeMin0 = aabb0->Merge(node->fatAABB)->Volume() - aabb0->Volume();
		const float volumeMin1 = aabb1->Merge(node->fatAABB)->Volume() - aabb1->Volume();

		if (volumeMin0 < volumeMin1)
			InsertNode(node, &refParent->children[0]);
		else
			InsertNode(node, &refParent->children[1]);
	}
	UpdateFatAABB(*parent);
}

void CBroadPhaseAABBTree::Add(AABB* trueAABB)
{
	if (m_root)
	{
		AABBTreeNode* newNode = new AABBTreeNode;
		newNode->SetAsLeaf(trueAABB);
		UpdateFatAABB(newNode);
		InsertNode(newNode, &m_root);
	}
	else
	{
		m_root = new AABBTreeNode;
		m_root->SetAsLeaf(trueAABB);
		UpdateFatAABB(m_root);
	}
}

void CBroadPhaseAABBTree::Remove(AABBTreeNode* deleteMe)
{
	AABBTreeNode* parent = deleteMe->parent;

	if (parent)
	{
		AABBTreeNode* sibling = deleteMe->GetSibling();
		if (parent->parent)
		{
			sibling->parent = parent->parent;
			if (parent == parent->parent->children[0])
				parent->parent->children[0] = sibling;
			else
				parent->parent->children[1] = sibling;
		}
		else
		{
			m_root = sibling;
			m_root->parent = nullptr;
		}
		delete deleteMe;
		delete parent;
	}
	else
	{
		delete deleteMe;
		m_root = nullptr;
	}
}

void CBroadPhaseAABBTree::Update()
{
	if (!m_root) return;

	UpdatePolyAABB(m_root);

	if (!m_root->IsLeaf())
	{
		m_invalidNodes.clear();

		GetInvalidNodes(m_root);

		const std::string str = "Invalid Nodes : " + std::to_string(m_invalidNodes.size());
		gVars->pRenderer->DisplayText(str, 50, 100);

		for (AABBTreeNode* node : m_invalidNodes)
		{
			AABBTreeNode* parent = node->parent;
			AABBTreeNode* sibling = node->GetSibling();

			AABBTreeNode** parentLink;
			if (parent->parent)
			{
				if (parent == parent->parent->children[0])
				{
					parentLink = &parent->parent->children[0];
				}
				else
				{
					parentLink = &parent->parent->children[1];
				}
				sibling->parent = parent->parent;
			}
			else
			{
				parentLink = &m_root;
				sibling->parent = nullptr;

			}

			*parentLink = sibling;

			parent->children[0] = parent->children[1] = nullptr;

			delete parent;

			UpdateFatAABB(node);
			InsertNode(node, &m_root);
		}
		m_invalidNodes.clear();
	}
}

void CBroadPhaseAABBTree::DrawGizmos()
{
	if (!m_root) return;
	DrawPolyAABB(m_root);
	DrawFatAABB(m_root);
}

AABB* CBroadPhaseAABBTree::BuildPolyAABB(const CPolygonPtr& poly) const
{
	const size_t size = poly->points.size();
	AABB* polyAABB = new AABB;
	polyAABB->polyRef = poly;
	std::vector<Vec2> transformatedPoints;

	for (const Vec2 point : poly->points)
		transformatedPoints.push_back(poly->TransformPoint(point));

	for (int pointIndex = 0; pointIndex < size; ++pointIndex)
	{
		if (polyAABB->maxX < transformatedPoints[pointIndex].x) polyAABB->maxX = transformatedPoints[pointIndex].x;
		if (polyAABB->minX > transformatedPoints[pointIndex].x) polyAABB->minX = transformatedPoints[pointIndex].x;
		if (polyAABB->maxY < transformatedPoints[pointIndex].y) polyAABB->maxY = transformatedPoints[pointIndex].y;
		if (polyAABB->minY > transformatedPoints[pointIndex].y) polyAABB->minY = transformatedPoints[pointIndex].y;
	}
	return polyAABB;
}

void CBroadPhaseAABBTree::UpdatePolyAABB(AABBTreeNode* node) const
{
	if (node->IsLeaf())
	{
		const size_t size = node->polyAABB->polyRef->points.size();
		CPolygonPtr poly = node->polyAABB->polyRef;

		std::vector<Vec2> transformatedPoints;

		for (Vec2 point : poly->points)
		{
			transformatedPoints.push_back(poly->TransformPoint(point));
		}

		float _maxX = -FLT_MAX;
		float _minX = FLT_MAX;
		float _maxY = -FLT_MAX;
		float _minY = FLT_MAX;

		for (size_t pointIndex = 0; pointIndex < size; ++pointIndex)
		{
			if (_maxX < transformatedPoints[pointIndex].x) _maxX = transformatedPoints[pointIndex].x;
			if (_minX > transformatedPoints[pointIndex].x) _minX = transformatedPoints[pointIndex].x;
			if (_maxY < transformatedPoints[pointIndex].y) _maxY = transformatedPoints[pointIndex].y;
			if (_minY > transformatedPoints[pointIndex].y) _minY = transformatedPoints[pointIndex].y;
		}

		node->polyAABB->maxX = _maxX;
		node->polyAABB->maxY = _maxY;
		node->polyAABB->minX = _minX;
		node->polyAABB->minY = _minY;


	}
	else
	{
		UpdatePolyAABB(node->children[0]);
		UpdatePolyAABB(node->children[1]);
	}
}

void CBroadPhaseAABBTree::UpdateFatAABB(AABBTreeNode* node) const
{
	if (node->IsLeaf())
	{
		node->fatAABB->maxX = node->polyAABB->maxX + m_margin;
		node->fatAABB->minX = node->polyAABB->minX - m_margin;
		node->fatAABB->maxY = node->polyAABB->maxY + m_margin;
		node->fatAABB->minY = node->polyAABB->minY - m_margin;
	}
	else
	{
		delete node->fatAABB;
		node->fatAABB = node->children[0]->fatAABB->Merge(node->children[1]->fatAABB);
	}
}

/*bool CBroadPhaseAABBTree::CheckPolySizes(AABB* poly, AABB* fat) const
{
	return poly->maxX + m_margin == fat->maxX
		|| poly->maxY + m_margin == fat->maxY
		|| poly->minX + m_margin == fat->minX
		|| poly->minY + m_margin == fat->minY;
}*/

void CBroadPhaseAABBTree::GetInvalidNodes(AABBTreeNode* node)
{
	if (node->IsLeaf())
	{
		if (!node->fatAABB->Contain(node->polyAABB))
		{
			m_invalidNodes.push_back(node);
		}
	}
	else
	{
		GetInvalidNodes(node->children[0]);
		GetInvalidNodes(node->children[1]);
	}
}


void CBroadPhaseAABBTree::ComputePairs(AABBTreeNode* brother, AABBTreeNode* sister)
{
	if (brother->IsLeaf())
	{
		if (sister->IsLeaf())
		{
			if (brother->polyAABB->Collide(sister->polyAABB))
			{
				m_nodePairs.emplace_back(brother->polyAABB->polyRef, sister->polyAABB->polyRef);
			}
		}
		else
		{
			CrossChild(sister);
			ComputePairs(brother, sister->children[0]);
			ComputePairs(brother, sister->children[1]);

		}
	}
	else
	{
		if (sister->IsLeaf())
		{
			CrossChild(brother);
			ComputePairs(brother->children[0], sister);
			ComputePairs(brother->children[1], sister);
		}
		else
		{
			CrossChild(brother);
			CrossChild(sister);

			ComputePairs(brother->children[0], sister->children[0]);
			ComputePairs(brother->children[0], sister->children[1]);
			ComputePairs(brother->children[1], sister->children[0]);
			ComputePairs(brother->children[1], sister->children[1]);

		}
	}
	
}

void CBroadPhaseAABBTree::CrossChild(AABBTreeNode* node)
{
	if (node->crossed) return;
	node->crossed = true;
	ComputePairs(node->children[0], node->children[1]);
}

void CBroadPhaseAABBTree::ClearCrossFlag(AABBTreeNode* node)
{
	node->crossed = false;
	if (node->IsLeaf()) return;
	ClearCrossFlag(node->children[0]);
	ClearCrossFlag(node->children[1]);
}

void CBroadPhaseAABBTree::DrawFatAABB(AABBTreeNode* node)
{
	Vec2 gizmosPoints[4];

	gizmosPoints[0] = Vec2(node->fatAABB->minX, node->fatAABB->maxY);
	gizmosPoints[1] = Vec2(node->fatAABB->maxX, node->fatAABB->maxY);
	gizmosPoints[2] = Vec2(node->fatAABB->maxX, node->fatAABB->minY);
	gizmosPoints[3] = Vec2(node->fatAABB->minX, node->fatAABB->minY);

	const int gizmosMaxPoint = 4;

	float r, g, b;
	if (node->IsLeaf())
	{
		r = 1.f;
		g = 0.0f;
		b = 0.0f;
	}
	else
	{
		r = 0.f;
		g = 1.f;
		b = 0.0f;
	}

	for (int index = 0; index < gizmosMaxPoint; ++index)
		gVars->pRenderer->DrawLine(gizmosPoints[index], gizmosPoints[(index + 1) % gizmosMaxPoint], r, g, b);

	if (node->IsLeaf()) return;
	DrawFatAABB(node->children[0]);
	DrawFatAABB(node->children[1]);
}

void CBroadPhaseAABBTree::DrawPolyAABB(AABBTreeNode* node)
{
	if (node->IsLeaf())
	{
		Vec2 gizmosPoints[4];

		gizmosPoints[0] = (Vec2(node->polyAABB->minX, node->polyAABB->maxY));
		gizmosPoints[1] = (Vec2(node->polyAABB->maxX, node->polyAABB->maxY));
		gizmosPoints[2] = (Vec2(node->polyAABB->maxX, node->polyAABB->minY));
		gizmosPoints[3] = (Vec2(node->polyAABB->minX, node->polyAABB->minY));

		const int gizmosMaxPoint = 4;

		for (int index = 0; index < gizmosMaxPoint; ++index)
			gVars->pRenderer->DrawLine(gizmosPoints[index], gizmosPoints[(index + 1) % gizmosMaxPoint], 0.f, 0.f, 1.f);
	}
	else
	{
		DrawPolyAABB(node->children[0]);
		DrawPolyAABB(node->children[1]);
	}
}

