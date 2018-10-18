// CollisionEngine.cpp : définit le point d'entrée pour l'application console.
//

#include "stdafx.h"

#include "Application.h"

#include "SceneManager.h"


#include <iostream>
#include <string>




//#include "InertiaTensor.h"

#include "Scenes/SceneDebugCollisions.h"
#include "Scenes/SceneBouncingPolys.h"
#include "Scenes/SceneSimplePhysic.h"
#include "Scenes/SceneSpheres.h"
#include "Scenes/SceneComplexPhysic.h"
#include "Scenes/SceneSmallPhysic.h"


/*
* Entry point
*/
int _tmain(int argc, char** argv)
{


	InitApplication(1260, 768, 50.0f);

	gVars->pSceneManager->AddScene(new CSceneSmallPhysic());
	gVars->pSceneManager->AddScene(new CSceneSimplePhysic());
	gVars->pSceneManager->AddScene(new CSceneComplexPhysic(25));



	RunApplication();
	return 0;
}

