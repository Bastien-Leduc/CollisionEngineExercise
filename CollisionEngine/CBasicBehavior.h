#pragma once
#include "Behavior.h"
#include "Collision.h"
class CBasicBehavior : public CBehavior
{
public:
	CBasicBehavior();
	virtual ~CBasicBehavior();

	virtual void Start() override;
	virtual void Update(float frameTime);

private:
	void GenerateManifold(SCollision& collision, CPolygonPtr polyA, CPolygonPtr polyB);
	std::vector<Vec2> GetManifoldPoints(SCollision& collision, CPolygonPtr polyA, CPolygonPtr polyB);
	SCollisionSegmentData GetBestEdge(const Vec2& collisionNormal, CPolygonPtr poly) const;
	void GetClippedPoints(const Vec2& referentNormal, float threshold, const SCollisionSegmentData& incidentData, std::vector<Vec2>& pointArray);
	void GetClippedPoints(const Vec2& referentNormal, float threshold, std::vector<Vec2>& pointArray);

	float ApplyCollisionResponse(const SCollision& collision);
	void ApplyFriction(const SCollision& collision, float impulse);

	void DrawGizmos(const SCollision& collision);

	std::vector<CPolygonPtr> m_shapes;
	SCollision lastCol;
};

