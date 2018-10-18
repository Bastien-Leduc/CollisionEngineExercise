#ifndef _PHYSIC_ENGINE_H_
#define _PHYSIC_ENGINE_H_

#include <vector>
#include <unordered_map>
#include "Maths.h"
#include "Polygon.h"
#include "Collision.h"

class IBroadPhase;

class CPhysicEngine
{



public:
	void	Reset();
	void	Activate(bool active);

	void	DetectCollisions();

	void	Step(float deltaTime);

	template<typename TFunctor>
	void	ForEachCollision(TFunctor functor)
	{
		for (const SCollision& collision : m_collidingPairs)
		{
			functor(collision);
		}
	}

private:
	void							CollisionBroadPhase();
	void							CollisionNarrowPhase();

	bool							m_active = true;

	// Collision detection
	IBroadPhase*					m_broadPhase;
	std::vector<SPolygonPair>		m_pairsToCheck;
	std::vector<SCollision>			m_collidingPairs;

};

#endif