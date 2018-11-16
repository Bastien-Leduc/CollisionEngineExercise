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

private:
	virtual void Update(float frameTime) override
	{
		gVars->pPhysicEngine->Activate(true);

		SCollision collision;

		if (polyA->CheckCollision(*polyB, collision))
		{
			gVars->pRenderer->DisplayTextWorld("collision point", collision.point);
			gVars->pRenderer->DisplayText("Collision distance : " + std::to_string(collision.distance), 50, 50);

			gVars->pRenderer->DrawLine(collision.point, collision.point + collision.normal * collision.distance, 0.0f, 1.0f, 0.0f);
		}
	}
};


#endif