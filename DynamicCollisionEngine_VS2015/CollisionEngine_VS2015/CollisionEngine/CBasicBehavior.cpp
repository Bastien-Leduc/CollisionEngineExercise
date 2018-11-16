#include "CBasicBehavior.h"
#include "PhysicEngine.h"
#include "Renderer.h"
#include "GlobalVariables.h"
#include "World.h"

#define GRAVITY 9.f
#define DECELERATION 0.3f

#define BOUNCINESS 0.0f
#define FRICTION 0.f

CBasicBehavior::CBasicBehavior()
{
	m_shapes = std::vector<CPolygonPtr>();
}


CBasicBehavior::~CBasicBehavior()
{
}

void CBasicBehavior::Start()
{
	gVars->pPhysicEngine->Activate(true);
}

void CBasicBehavior::Update(float frameTime)
{
	gVars->pWorld->ForEachPolygon([&](CPolygonPtr poly) 
	{
		ApplyGravity(poly, frameTime);
	});

	gVars->pPhysicEngine->ForEachCollision([&](const SCollision& collision)
	{
		float lastImpulse = ApplyCollisionResponse(collision);
		ApplyFriction(collision, lastImpulse);
	});
}

void CBasicBehavior::ApplyGravity(CPolygonPtr poly, float frameTime)
{
	if (poly->GetMass() == 0) return;
	/*poly->speed.y -= GRAVITY * frameTime;
	poly->speed -= poly->speed * DECELERATION * frameTime;*/
}

float CBasicBehavior::ApplyCollisionResponse(const SCollision & collision)
{
	CPolygonPtr polyA = collision.polyA;
	CPolygonPtr polyB = collision.polyB;

	Vec2 diffspeed = polyB->speed - polyA->speed;

	float polyAMass = polyA->GetMass();
	float polyBMass = polyB->GetMass();

	float impulse;
	
	impulse = (-(BOUNCINESS + 1.f) * (diffspeed | collision.normal)) / ((1.f / polyAMass) + (1.f / polyBMass));

	collision.polyA->speed -= collision.normal * (impulse / polyAMass);
	collision.polyA->position -= collision.normal * (collision.distance * 0.5f);

	collision.polyB->speed += collision.normal * (impulse / polyBMass);
	collision.polyB->position += collision.normal * (collision.distance * 0.5f);


	return impulse;
}

void CBasicBehavior::ApplyFriction(const SCollision& collision, float impulse)
{
	CPolygonPtr polyA = collision.polyA;
	CPolygonPtr polyB = collision.polyB;

	Vec2 diffspeed = polyB->speed - polyA->speed;

	float polyAMass = polyA->GetMass();
	float polyBMass = polyB->GetMass();

	Vec2 tan = diffspeed - (collision.normal * (diffspeed | collision.normal));
	tan.Normalize();
	float coeffFric = diffspeed | tan;

	float impulseFric;
	impulseFric = -coeffFric / ((1 / polyAMass) + (1 / polyBMass));

	impulseFric = Clamp(impulseFric, -abs(impulse) * FRICTION, abs(impulse) * FRICTION);

	collision.polyA->speed -= tan * (1 / polyAMass) * impulseFric;
	collision.polyB->speed += tan * (1 / polyBMass) * impulseFric;
}
