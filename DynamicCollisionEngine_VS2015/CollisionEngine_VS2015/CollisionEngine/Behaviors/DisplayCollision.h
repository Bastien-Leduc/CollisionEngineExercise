#ifndef _DISPLAY_COLLISION_H_
#define _DISPLAY_COLLISION_H_

#include "Behavior.h"
#include "PhysicEngine.h"
#include "GlobalVariables.h"
#include "Renderer.h"
#include "RenderWindow.h"
#include "World.h"

#include <string>
#include <iostream>

class CDisplayCollision : public CBehavior
{
public:
	CPolygonPtr polyA;
	CPolygonPtr polyB;

	SCollision lastCollision;
	bool collided = false;

private:
	virtual void Update(float frameTime) override
	{
		gVars->pPhysicEngine->Activate(true);
		collided = polyA->CheckCollision(*polyB, lastCollision);
	}

	virtual void DrawGizmos()
	{
		if (!collided) return;
		gVars->pRenderer->DisplayTextWorld("collision point", lastCollision.point);
		gVars->pRenderer->DisplayText("Collision distance : " + std::to_string(lastCollision.distance), 50, 50);

		gVars->pRenderer->DrawLine(lastCollision.point, lastCollision.point + lastCollision.normal * lastCollision.distance, 0.0f, 1.0f, 0.0f);
	}
};


#endif