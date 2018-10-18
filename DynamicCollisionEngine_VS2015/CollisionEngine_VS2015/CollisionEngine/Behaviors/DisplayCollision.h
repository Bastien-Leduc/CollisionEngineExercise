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
		gVars->pPhysicEngine->Activate(false);


	}
};


#endif