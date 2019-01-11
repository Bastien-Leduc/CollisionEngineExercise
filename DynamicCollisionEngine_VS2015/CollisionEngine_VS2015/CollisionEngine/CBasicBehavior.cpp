#include "CBasicBehavior.h"
#include "PhysicEngine.h"
#include "Renderer.h"
#include "GlobalVariables.h"
#include "World.h"
#include <string>

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
	gVars->pPhysicEngine->ForEachCollision([&](SCollision& collision)
	{
		GenerateManifold(collision, collision.polyA, collision.polyB);
		lastCol = collision;
		float lastImpulse = ApplyCollisionResponse(collision);
		ApplyFriction(collision, lastImpulse);
	});

	DrawGizmos(lastCol);
}

void CBasicBehavior::GenerateManifold(SCollision& collision, CPolygonPtr polyA, CPolygonPtr polyB)
{
	std::vector<Vec2> clippedPoints = GetManifoldPoints(collision, polyA, polyB);
	if (clippedPoints.size() == 0) return;
	collision.point = clippedPoints[0];
}

/** Returns clipped points **/
std::vector<Vec2> CBasicBehavior::GetManifoldPoints(SCollision& collision, CPolygonPtr polyA, CPolygonPtr polyB)
{
	/** Get the nearest edge of shapes from the collision normal **/
	SCollisionSegmentData bestPointA = GetBestEdge(collision.normal, polyA);
	SCollisionSegmentData bestPointB = GetBestEdge(-collision.normal, polyB);

	/** Show gathered edges **/
	if (gVars->bDebug)
	{
		gVars->pRenderer->DrawLine(bestPointA.collisionSegmentStart + Vec2(0.5f, 0.5f), bestPointA.collisionSegmentEnd + Vec2(0.5f, 0.5f), 1.f, 0.f, 0.f);
		gVars->pRenderer->DrawLine(bestPointB.collisionSegmentStart + Vec2(0.5f, 0.5f), bestPointB.collisionSegmentEnd + Vec2(0.5f, 0.5f), 1.f, 0.f, 0.f);
	}

	SCollisionSegmentData referent, incident;
	bool requireFlip = false;

	float refDot = abs(bestPointA.collisionSegmentDir | collision.normal);
	float incDot = abs(bestPointB.collisionSegmentDir | collision.normal);

	/** If the first shape edges direction dot with the normal is near 0, it means it is almost perpenducular to it **/
	/** It also means the first shape is the one being collided on **/
	if (refDot < incDot)
	{
		referent = bestPointA;
		incident = bestPointB;
	}
	else
	{
		referent = bestPointB;
		incident = bestPointA;
	}

	std::vector<Vec2> clippedPoint;

	/** We calculate a threshold, meaning any dot value minus the threshold lesser than 0 means the point is not in the collision area **/
	float threshold = referent.collisionSegmentDir | referent.collisionSegmentStart;
	GetClippedPoints(referent.collisionSegmentDir, threshold, incident, clippedPoint);
	if (clippedPoint.size() < 2) return clippedPoint;

	threshold = referent.collisionSegmentDir | referent.collisionSegmentEnd;
	GetClippedPoints(-referent.collisionSegmentDir, -threshold, clippedPoint);
	if (clippedPoint.size() < 2) return clippedPoint;

	/** We also clip points past the collided with shape edges **/
	Vec2 refNormal = referent.collisionSegmentDir;
	refNormal.x = -referent.collisionSegmentDir.y;
	refNormal.y = referent.collisionSegmentDir.x;

	
	float maxThreshold = refNormal | referent.collisionSegmentStart;

	size_t eraseIndex = 0;

	if ((refNormal | clippedPoint[eraseIndex]) - maxThreshold < 0.f) clippedPoint.erase(clippedPoint.begin());
	else ++eraseIndex;

	if ((refNormal | clippedPoint[eraseIndex]) - maxThreshold < 0.f) clippedPoint.erase(clippedPoint.begin() + eraseIndex);

	if (gVars->bDebug)
	{
		if (clippedPoint.size() != 0) gVars->pRenderer->DisplayTextWorld("1", clippedPoint[0]);
		if (clippedPoint.size() == 2) gVars->pRenderer->DisplayTextWorld("2", clippedPoint[1]);
	}

	return clippedPoint;
}

SCollisionSegmentData CBasicBehavior::GetBestEdge(const Vec2& collisionNormal, CPolygonPtr poly) const
{
	float maxProjectionValue = -FLT_MAX;
	size_t bestPointIndex;

	size_t verticesCount = poly->points.size();
	for (size_t index = 0; index < verticesCount; ++index)
	{
		Vec2 currentPoint = poly->TransformPoint(poly->points[index]);
		float projection = collisionNormal | currentPoint;
		if (projection <= maxProjectionValue) continue;
		maxProjectionValue = projection;
		bestPointIndex = index;
	}

	Vec2 bestPoint = poly->TransformPoint(poly->points[bestPointIndex]);
	Vec2 leftPoint = poly->TransformPoint((poly->points[(bestPointIndex + (verticesCount - 1)) % verticesCount]));
	Vec2 rightPoint = poly->TransformPoint((poly->points[(bestPointIndex + 1) % verticesCount]));

	Vec2 leftSegment = bestPoint - leftPoint;
	Vec2 rightSegment = bestPoint - rightPoint;	

	if ((rightSegment | collisionNormal) <= (leftSegment | collisionNormal))
	{
		return SCollisionSegmentData(bestPoint, rightPoint, -rightSegment);
	}
	else
	{
		return SCollisionSegmentData(bestPoint, leftPoint, leftSegment);
	}
}

void CBasicBehavior::GetClippedPoints(const Vec2& referentNormal, float threshold, const SCollisionSegmentData& incidentData, std::vector<Vec2>& pointArray)
{
	float firstPointDot = (referentNormal | incidentData.collisionSegmentStart) - threshold;
	float secondPointDot = (referentNormal | incidentData.collisionSegmentEnd) - threshold;

	/** If the point is in the collided area, we register it **/
	if (firstPointDot >= 0.f) pointArray.push_back(incidentData.collisionSegmentStart);
	if (secondPointDot >= 0.f) pointArray.push_back(incidentData.collisionSegmentEnd);


	/** If on of the two point is not in the colliding area, we try to create one, wich is the point on the colliding edge with the limit of the threshold axis **/
	if (firstPointDot * secondPointDot < 0.f)
	{
		Vec2 segment = incidentData.collisionSegmentDir;
		float ratio = firstPointDot / (firstPointDot - secondPointDot);
		segment *= ratio;
		segment += referentNormal;
		pointArray.push_back(segment);
	}
}

void CBasicBehavior::GetClippedPoints(const Vec2& referentNormal, float threshold, std::vector<Vec2>& pointArray)
{
	/** Same with GetClippedPoints, but with already clipped points **/
	float firstPointDot = (referentNormal | pointArray[0]) - threshold;
	float secondPointDot = (referentNormal | pointArray[1]) - threshold;

	Vec2 point1 = pointArray[0];
	Vec2 point2 = pointArray[1];


	if (firstPointDot < 0.f) pointArray.erase(pointArray.begin());
	if (secondPointDot < 0.f) pointArray.erase(pointArray.begin());

	if (pointArray.size() == 2) return;
	
	if (firstPointDot * secondPointDot < 0.f)
	{
		Vec2 segment = point2 - point1;
		float ratio = firstPointDot / (firstPointDot - secondPointDot);
		segment *= ratio;
		segment += referentNormal;
		pointArray.push_back(segment);
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

void CBasicBehavior::DrawGizmos(const SCollision& collision)
{
	Vec2 arrowBase = collision.point;
	Vec2 arrowEnd = collision.point + collision.normal * collision.distance;

	Vec2 branch = collision.normal * -1.f;
	branch.Rotate(45.f);

	gVars->pRenderer->DrawLine(arrowBase, arrowEnd, 0.0f, 0.0f, 1.0f);
	gVars->pRenderer->DrawLine(arrowEnd, arrowEnd + branch * 0.7f, 0.0f, 0.0f, 1.0f);

	branch.Rotate(-90.f);
	gVars->pRenderer->DrawLine(arrowEnd, arrowEnd + branch * 0.7f, 0.0f, 0.0f, 1.0f);
}
