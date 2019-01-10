#include "Polygon.h"
#include <GL/glu.h>

#include "InertiaTensor.h"

#include "PhysicEngine.h"
#include "World.h"
#include "GlobalVariables.h"
#include "Renderer.h"
#include "Collision.h"

CPolygon::CPolygon(size_t index)
	: m_vertexBufferId(0), m_index(index), density(0.1f)
{
}

CPolygon::~CPolygon()
{
	DestroyBuffers();
}

void CPolygon::Build()
{
	m_lines.clear();

	ComputeArea();
	RecenterOnCenterOfMass();
	ComputeLocalInertiaTensor();

	CreateBuffers();
	BuildLines();
}

void CPolygon::Draw()
{
	// Set transforms (qssuming model view mode is set)
	float transfMat[16] = {	rotation.X.x, rotation.X.y, 0.0f, 0.0f,
							rotation.Y.x, rotation.Y.y, 0.0f, 0.0f,
							0.0f, 0.0f, 0.0f, 1.0f,
							position.x, position.y, -1.0f, 1.0f };
	glPushMatrix();
	glMultMatrixf(transfMat);

	// Draw vertices
	BindBuffers();
	glDrawArrays(GL_LINE_LOOP, 0, points.size());
	glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();
}

size_t	CPolygon::GetIndex() const
{
	return m_index;
}

float	CPolygon::GetArea() const
{
	return fabsf(m_signedArea);
}

Vec2	CPolygon::TransformPoint(const Vec2& point) const
{
	return position + rotation * point;
}

Vec2	CPolygon::InverseTransformPoint(const Vec2& point) const
{
	return rotation.GetInverseOrtho() * (point - position);
}

bool	CPolygon::IsPointInside(const Vec2& point) const
{
	float maxDist = -FLT_MAX;

	for (const Line& line : m_lines)
	{
		Line globalLine = line.Transform(rotation, position);
		float pointDist = globalLine.GetPointDist(point);
		maxDist = Max(maxDist, pointDist);
	}

	return maxDist <= 0.0f;
}

bool	CPolygon::IsLineIntersectingPolygon(const Line& line, Vec2& colPoint, float& colDist) const
{
	//float dist = 0.0f;
	float minDist = FLT_MAX;
	Vec2 minPoint;
	float lastDist = 0.0f;
	bool intersecting = false;

	for (const Vec2& point : points)
	{
		Vec2 globalPoint = TransformPoint(point);
		float dist = line.GetPointDist(globalPoint);
		if (dist < minDist)
		{
			minPoint = globalPoint;
			minDist = dist;
		}

		intersecting = intersecting || (dist != 0.0f && lastDist * dist < 0.0f);
		lastDist = dist;
	}

	if (minDist <= 0.0f)
	{
		colDist = -minDist;
		colPoint = minPoint;
	}
	return (minDist <= 0.0f);
}

bool	CPolygon::CheckCollision(const CPolygon& poly, SCollision& collision) const
{
	collision.distance = FLT_MAX;
	if (SatCollisionChecker(poly, collision.point, collision.normal, collision.distance, true))
		return poly.SatCollisionChecker(*this, collision.point, collision.normal, collision.distance, false);


	return false;
}

Vec2* CPolygon::GetSATAxis() const
{
	const size_t size = points.size();
	Vec2* axis = new Vec2[size];
	
	for (size_t index = 0; index < size; ++index)
		axis[index] = (TransformPoint(points[(index + 1) % size]) - TransformPoint(points[index])).GetNormal().Normalized();

	return axis;
}

Projection CPolygon::Project(const Vec2& axis) const
{
	float min = axis | TransformPoint(m_lines[0].point);
	float max = min;

	size_t size = points.size();
	for (size_t index = 0; index < size; ++index)
	{
		float projResult = axis | TransformPoint(m_lines[index].point);
		if (projResult < min) min = projResult;
		if (projResult > max) max = projResult;
	}

	return Projection(min, max);
}


//void CPolygon::UpdateAABB()
//{
//	aabb.Center(position);
//	for (const Vec2& point : points)
//	{
//		aabb.Extend(TransformPoint(point));
//	}
//}

float CPolygon::GetMass() const
{
	return density * GetArea();
}

float CPolygon::GetInertiaTensor() const
{
	return m_localInertiaTensor * GetMass();
}

Vec2 CPolygon::GetPointVelocity(const Vec2& point) const
{
	return speed + (point - position).GetNormal() * angularVelocity;
}

void CPolygon::CreateBuffers()
{
	DestroyBuffers();

	float* vertices = new float[3 * points.size()];
	for (size_t i = 0; i < points.size(); ++i)
	{
		vertices[3 * i] = points[i].x;
		vertices[3 * i + 1] = points[i].y;
		vertices[3 * i + 2] = 0.0f;
	}

	glGenBuffers(1, &m_vertexBufferId);

	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferId);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * points.size(), vertices, GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	delete[] vertices;
}

void CPolygon::BindBuffers()
{
	if (m_vertexBufferId != 0)
	{
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferId);

		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, 0, (void*)0);
	}
}

void CPolygon::DestroyBuffers()
{
	if (m_vertexBufferId != 0)
	{
		glDeleteBuffers(1, &m_vertexBufferId);
		m_vertexBufferId = 0;
	}
}

void CPolygon::BuildLines()
{
	for (size_t index = 0; index < points.size(); ++index)
	{
		const Vec2& pointA = points[index];
		const Vec2& pointB = points[(index + 1) % points.size()];

		Vec2 lineDir = (pointA - pointB).Normalized();

		m_lines.push_back(Line(pointB, lineDir, (pointA - pointB).GetLength()));
	}
}

bool CPolygon::SatCollisionChecker(const CPolygon& poly, Vec2& colPoint, Vec2& colNormal, float& colDist, bool receiver) const
{
	Vec2* shape1Axies = GetSATAxis();
	size_t size = points.size();

	for (size_t index = 0; index < size; ++index)
	{
		//gVars->pRenderer->DrawLine(shape1Axies[index], shape1Axies[index]*2.f, 1.f, 0.f, 0.f);
		Vec2 axis = shape1Axies[index];
		Projection shapeProj1 = Project(axis);
		Projection shapeProj2 = poly.Project(axis);

		if (!shapeProj1.IsOverlaping(shapeProj2)) return false;

		float tempPen = shapeProj1.GetOverlapValue(shapeProj2);

		if (tempPen > colDist) continue;
		
		colDist = tempPen;
		colNormal = axis;
	}
	
	delete[] shape1Axies;

	Vec2* shape2Axies = poly.GetSATAxis();
	size = poly.points.size();

	for (size_t index = 0; index < size; ++index)
	{
		//gVars->pRenderer->DrawLine(shape2Axies[index], shape2Axies[index] * 5.f, 0.f, 0.f, 1.f);
		Vec2 axis = shape2Axies[index];
		Projection shapeProj1 = Project(axis);
		Projection shapeProj2 = Project(axis);

		if (!shapeProj1.IsOverlaping(shapeProj2)) return false;

		float tempPen = shapeProj1.GetOverlapValue(shapeProj2);

		if (tempPen > colDist) continue;

		colDist = tempPen;
		colNormal = axis;
	}

	delete[] shape2Axies;

	Vec2 dist = (poly.position - position).Normalized();
	if ((dist | colNormal) < 0.f)
		colNormal *= -1.f;

	return true;
	/*const size_t ShapeAMaxEdge = points.size();

	for (int ShapeANormalIndex = 0; ShapeANormalIndex < ShapeAMaxEdge; ++ShapeANormalIndex)
	{
		const Vec2 p1 = TransformPoint(points[ShapeANormalIndex]);
		const Vec2 p2 = TransformPoint(points[(ShapeANormalIndex + 1) % ShapeAMaxEdge]);
		Vec2 penPointA, penPointB;
		Vec2 direction = (p2 - p1).Normalized();
		const Vec2 normal = direction.GetNormal();

		if (colNormal == normal)
		{
			continue;
		}

		Line projectionLine(p1, normal);

		Line firstSegment(p1, direction);


		float shapeAMinDistance = 0;
		float shapeAMaxDistance = shapeAMinDistance;


		for (int shapeAPointIndex = 0; shapeAPointIndex < ShapeAMaxEdge; ++shapeAPointIndex)
		{
			const Vec2 worldPoint = TransformPoint(points[shapeAPointIndex]);

			const Vec2 temp = projectionLine.Project(worldPoint);
			//gVars->pRenderer->DrawLine(normal * -20.f, normal * 20.f, 1.f, 0.f, 0.f);

			const float tempDistance = firstSegment.GetPointDist(temp);


			if (tempDistance >= shapeAMaxDistance)
			{
				shapeAMaxDistance = tempDistance;
				penPointA = worldPoint;
			}
			else if (tempDistance < shapeAMinDistance)
			{
				shapeAMinDistance = tempDistance;
			}
		}

		const size_t shapeBMaxEdge = poly.points.size();

		float shapeBMinDistance = 0;
		float shapeBMaxDistance = shapeBMinDistance;


		for (int shapeBPointIndex = 0; shapeBPointIndex < shapeBMaxEdge; ++shapeBPointIndex)
		{
			const Vec2 worldPoint = poly.TransformPoint(poly.points[shapeBPointIndex]);

			const Vec2 temp = projectionLine.Project(worldPoint);

			const float tempDistance = firstSegment.GetPointDist(temp);

			if (shapeBPointIndex == 0)
			{
				shapeBMinDistance = shapeBMaxDistance = tempDistance;
				penPointB = worldPoint;

			}
			else if (tempDistance > shapeBMaxDistance)
			{
				shapeBMaxDistance = tempDistance;
				penPointB = worldPoint;
			}
			else if (tempDistance < shapeBMinDistance)
			{
				shapeBMinDistance = tempDistance;
			}
		}

		float penA = shapeAMaxDistance - shapeBMinDistance;
		float penB = shapeBMaxDistance - shapeAMinDistance;

		float epsilon = 0.1f;
		
		if (penA < penB) epsilon = -0.1f;
		else if (penB < penA) epsilon = 0.1f;

		if (penA <= 0 || penB <= 0) return false;

		const float finalPen = Min(penA, penB);

		if (finalPen < colDist + epsilon)
		{

			colDist = finalPen;

			if (shapeAMaxDistance < shapeBMaxDistance 
				&& shapeAMinDistance > shapeBMinDistance 
				|| shapeAMaxDistance < shapeBMaxDistance 
				&& shapeAMinDistance < shapeBMinDistance)
			{

				if (receiver)
				{
					colNormal = normal;
					colPoint = penPointA - (normal * colDist);
				}
				else
				{
					colNormal = normal * -1;
					colPoint = penPointA;
				}
			}
			else
			{
				if (receiver)
				{
					colPoint = penPointB;
					colNormal = normal * -1.f;
				}
				else
				{
					colNormal = normal;
					colPoint = penPointB - (normal * finalPen);
				}
			}

		}
	}


	return true;*/
}

void CPolygon::ComputeArea()
{
	m_signedArea = 0.0f;
	for (size_t index = 0; index < points.size(); ++index)
	{
		const Vec2& pointA = points[index];
		const Vec2& pointB = points[(index + 1) % points.size()];
		m_signedArea += pointA.x * pointB.y - pointB.x * pointA.y;
	}
	m_signedArea *= 0.5f;
}

void CPolygon::RecenterOnCenterOfMass()
{
	Vec2 centroid;
	for (size_t index = 0; index < points.size(); ++index)
	{
		const Vec2& pointA = points[index];
		const Vec2& pointB = points[(index + 1) % points.size()];
		float factor = pointA.x * pointB.y - pointB.x * pointA.y;
		centroid.x += (pointA.x + pointB.x) * factor;
		centroid.y += (pointA.y + pointB.y) * factor;
	}
	centroid /= 6.0f * m_signedArea;

	for (Vec2& point : points)
	{
		point -= centroid;
	}
	position += centroid;
}

void CPolygon::ComputeLocalInertiaTensor()
{
	m_localInertiaTensor = 0.0f;
	for (size_t i = 0; i + 1 < points.size(); ++i)
	{
		const Vec2& pointA = points[i];
		const Vec2& pointB = points[i + 1];

		m_localInertiaTensor += ComputeInertiaTensor_Triangle(Vec2(), pointA, pointB);
	}
}


