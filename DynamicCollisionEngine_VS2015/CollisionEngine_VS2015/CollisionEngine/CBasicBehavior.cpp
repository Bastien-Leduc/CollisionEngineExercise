#include "CBasicBehavior.h"
#include "PhysicEngine.h"
#include "Renderer.h"
#include "GlobalVariables.h"
#include "World.h"

#define BOUNCINESS 1.f
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
	gVars->pPhysicEngine->ForEachCollision([&](const SCollision& collision)
	{
		float lastImpulse = ApplyCollisionResponse(collision);
		ApplyFriction(collision, lastImpulse);
	});
}

float CBasicBehavior::ApplyCollisionResponse(const SCollision & collision)
{
	CPolygonPtr polyA = collision.polyA;
	CPolygonPtr polyB = collision.polyB;

	Vec2 diffspeed = polyB->speed - polyA->speed;

	float relativeSpeed = (diffspeed | collision.normal);
	if (relativeSpeed > 0) return 0;

	float polyAMass = polyA->GetMass();
	float polyBMass = polyB->GetMass();

	float polyAInvMass = polyAMass == 0 ? 0.f : 1.f / polyAMass;
	float polyBInvMass = polyBMass == 0 ? 0.f : 1.f / polyBMass;

	float polyInvMass = (polyAInvMass + polyBInvMass) == 0 ? 1.f : (polyAInvMass + polyBInvMass);

	float impulse;
	
	impulse = (-(BOUNCINESS + 1.f) * relativeSpeed) / polyInvMass;

	float damping = 1.f;
	float correction = (collision.distance * damping) / polyInvMass;

	collision.polyA->speed -= collision.normal * (impulse * polyAInvMass);
	collision.polyA->position -= collision.normal * correction * polyAInvMass;

	collision.polyB->speed += collision.normal * (impulse * polyBInvMass);
	collision.polyB->position += collision.normal * correction * polyBInvMass;

	return impulse;
}

void CBasicBehavior::ApplyFriction(const SCollision& collision, float impulse)
{
	CPolygonPtr polyA = collision.polyA;
	CPolygonPtr polyB = collision.polyB;

	Vec2 diffspeed = polyB->speed - polyA->speed;

	float relativeSpeed = (diffspeed | collision.normal);

	float polyAMass = polyA->GetMass();
	float polyBMass = polyB->GetMass();

	float polyAInvMass = polyAMass == 0 ? 0.f : 1.f / polyAMass;
	float polyBInvMass = polyBMass == 0 ? 0.f : 1.f / polyBMass;

	float polyInvMass = (polyAInvMass + polyBInvMass) == 0 ? 1.f : (polyAInvMass + polyBInvMass);


	Vec2 tan = diffspeed - (collision.normal * relativeSpeed);
	tan.Normalize();
	float coeffFric = diffspeed | tan;

	float impulseFric;
	impulseFric = -coeffFric / polyInvMass;

	impulseFric = Clamp(impulseFric, -abs(impulse) * FRICTION, abs(impulse) * FRICTION);

	collision.polyA->speed -= tan * polyAInvMass * impulseFric;
	collision.polyB->speed += tan * polyBInvMass * impulseFric;
}
