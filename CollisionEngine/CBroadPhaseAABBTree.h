#pragma once
#include "BroadPhase.h"
#include "AABBTreeNode.h"

class CBroadPhaseAABBTree : public IBroadPhase
{
public:
	CBroadPhaseAABBTree();
	~CBroadPhaseAABBTree() override;

	void Init() override;
	void GetCollidingPairsToCheck(std::vector<SPolygonPair>& pairsToCheck) override;
	void InsertNode(AABBTreeNode* node, AABBTreeNode** parent) const;
	void Add(AABB* trueAABB);
	void Remove(AABBTreeNode* deleteMe);
	void Update();
	void DrawGizmos() override;

private:
	AABB* BuildPolyAABB(const CPolygonPtr& poly) const;
	void UpdatePolyAABB(AABBTreeNode* node) const;
	void UpdateFatAABB(AABBTreeNode* node) const;
	void GetInvalidNodes(AABBTreeNode* node);
	void ComputePairs(AABBTreeNode* brother, AABBTreeNode* sister);
	void CrossChild(AABBTreeNode* node);
	void ClearCrossFlag(AABBTreeNode* node);

	void DrawFatAABB(AABBTreeNode* node);
	void DrawPolyAABB(AABBTreeNode* node);


	AABBTreeNode*				m_root;
	std::vector<AABBTreeNode*>	m_invalidNodes;
	std::vector<SPolygonPair>	m_nodePairs;
	const float					m_margin = 0.2f;
};

