#pragma once
#include "Behavior.h"
class CBasicBehavior : public CBehavior
{
public:
	CBasicBehavior();
	virtual ~CBasicBehavior();

	virtual void Start() override;
	virtual void Update(float frameTime);

private:

	float ApplyCollisionResponse(const SCollision& collision);
	void ApplyFriction(const SCollision& collision, float impulse);

	std::vector<CPolygonPtr> m_shapes;
};

