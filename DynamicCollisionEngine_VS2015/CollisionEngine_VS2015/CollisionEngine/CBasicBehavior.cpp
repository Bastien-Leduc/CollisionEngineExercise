#include "CBasicBehavior.h"
#include "PhysicEngine.h"
#include "Renderer.h"
#include "GlobalVariables.h"
#include "World.h"

#define BOUNCINESS 0.f
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
		GenerateManifold(collision);
		float lastImpulse = ApplyCollisionResponse(collision);
		ApplyFriction(collision, lastImpulse);
	});
}

void CBasicBehavior::GenerateManifold(const SCollision& collision, CPolygonPtr polyA, CPolygonPtr polyB)
{
	SCollisionPointData bestPointA = GetBestEdge(collision.normal, polyA);
	SCollisionPointData bestPointB = GetBestEdge(collision.normal, polyB);


}

SCollisionPointData CBasicBehavior::GetBestEdge(const Vec2& collisionNormal, CPolygonPtr poly) const
{
	float maxProjectionValue = -FLT_MAX;
	size_t bestPointIndex;

	size_t verticesCount = poly->points.size();
	for (size_t index = 0; index < verticesCount; ++index)
	{
		Vec2 currentPoint = poly->TransformPoint(poly->points[verticesCount]);
		float projection = collisionNormal | currentPoint;
		if (projection < maxProjectionValue) continue;
		maxProjectionValue = projection;
		bestPointIndex = index;
	}

	Vec2 bestPoint = poly->points[bestPointIndex];
	Vec2 leftPoint = poly->points[(bestPointIndex + 1) % verticesCount];
	Vec2 rightPoint = poly->points[(bestPointIndex - 1) % verticesCount];

	Vec2 leftSegment = bestPoint - leftPoint;
	Vec2 rightSegment = bestPoint - rightPoint;	

	leftSegment.Normalize();
	rightSegment.Normalize();

	if ((rightSegment | collisionNormal) <= (leftSegment | collisionNormal))
	{
		return SCollisionPointData(bestPoint, rightPoint, leftPoint, rightSegment);
	}
	else
	{
		return SCollisionPointData(bestPoint, rightPoint, leftPoint, leftSegment);
	}
}

float CBasicBehavior::ApplyCollisionResponse(const SCollision& collision)
{
	CPolygonPtr polyA = collision.polyA;
	CPolygonPtr polyB = collision.polyB;


	Vec2 diffspeed = polyB->speed - polyA->speed;
	float relativeSpeed = (diffspeed | collision.normal);
	if (relativeSpeed > 0) return 0;

	/************** SPEED AND POSITION **************/
	float polyAMass = polyA->GetMass();
	float polyBMass = polyB->GetMass();

	float polyAInvMass = polyAMass == 0 ? 0.f : 1.f / polyAMass;
	float polyBInvMass = polyBMass == 0 ? 0.f : 1.f / polyBMass;

	float polyInvMass = (polyAInvMass + polyBInvMass) == 0 ? 1.f : (polyAInvMass + polyBInvMass);
	
	/************** ROTATION **************/
	Vec2 centerToCollisionA = collision.point - polyA->position;
	Vec2 centerToCollisionB = collision.point - polyB->position;

	float tensorA = polyA->GetInertiaTensor();
	float tensorB = polyB->GetInertiaTensor();

	float tensorInverseA = tensorA == 0.f ? 0.f : 1.f / tensorA;
	float tensorInverseB = tensorB == 0.f ? 0.f : 1.f / tensorB;

	float torqueA = centerToCollisionA ^ collision.normal;
	float torqueB = centerToCollisionB ^ collision.normal;

	float momentumA = tensorInverseA * torqueA;
	float momentumB = tensorInverseB * torqueB;

	float rotWeightA = (centerToCollisionA.GetNormal() * momentumA) | collision.normal;
	float rotWeightB = (centerToCollisionB.GetNormal() * momentumB) | collision.normal;

	Vec2 angSpeedA = polyA->speed + (centerToCollisionA.GetNormal() * polyA->angularVelocity);
	Vec2 angSpeedB = polyB->speed + (centerToCollisionB.GetNormal() * polyB->angularVelocity);

	Vec2 angSpeedDiff = angSpeedB - angSpeedA;
	relativeSpeed = angSpeedDiff | collision.normal;

	float finalRotWeight = rotWeightA + rotWeightB;

	/************** IMPULSE **************/
	float impulse;
	
	impulse = (-(BOUNCINESS + 1.f) * relativeSpeed) / (polyInvMass + finalRotWeight);

	//if (impulse < 0) return 0;

	float damping = 0.5f;
	float correction = collision.distance / polyInvMass * damping;

	collision.polyA->speed -= collision.normal * (impulse * polyAInvMass);
	collision.polyA->position -= collision.normal * correction * polyAInvMass;
	collision.polyA->angularVelocity -= impulse * momentumA;

	collision.polyB->speed += collision.normal * (impulse * polyBInvMass);
	collision.polyB->position += collision.normal * correction * polyBInvMass;
	collision.polyB->angularVelocity += impulse * momentumB;

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
