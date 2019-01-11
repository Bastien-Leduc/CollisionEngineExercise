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
		/*if (!collided) return;
		gVars->pRenderer->DisplayTextWorld("collision point", lastCollision.point);
		gVars->pRenderer->DisplayText("Collision distance : " + std::to_string(lastCollision.distance), 50, 50);

		Vec2 arrowBase = lastCollision.point;
		Vec2 arrowEnd = lastCollision.point + lastCollision.normal * lastCollision.distance;

		Vec2 branch = -lastCollision.normal;
		branch.Rotate(45.f);

		gVars->pRenderer->DrawLine(arrowBase, arrowEnd, 0.0f, 0.0f, 1.0f);
		gVars->pRenderer->DrawLine(arrowEnd, arrowEnd + branch * 0.7f, 0.0f, 0.0f, 1.0f);
		
		branch.Rotate(-90.f);
		gVars->pRenderer->DrawLine(arrowEnd, arrowEnd + branch * 0.7f, 0.0f, 0.0f, 1.0f);*/

	}
};


#endif