#ifndef _POLYGON_H_
#define _POLYGON_H_

#include <GL/glew.h>
#include <vector>
#include <memory>
#include "Maths.h"




class CPolygon
{
private:
	friend class CWorld;

	CPolygon(size_t index);
public:
	~CPolygon();

	Vec2				position;
	Mat2				rotation;
	std::vector<Vec2>	points;
	//AABB				aabb;

	void				Build();
	void				Draw();
	size_t				GetIndex() const;

	float				GetArea() const;

	Vec2				TransformPoint(const Vec2& point) const;
	Vec2				InverseTransformPoint(const Vec2& point) const;

	// if point is outside then returned distance is negative (and doesn't make sense)
	bool				IsPointInside(const Vec2& point) const;

	// If line intersect polygon, colDist is the penetration distance, and colPoint most penetrating point of poly inside the line
	bool				IsLineIntersectingPolygon(const Line& line, Vec2& colPoint, float& colDist) const;
	bool				CheckCollision(const CPolygon& poly, struct SCollision& collision) const;
	Vec2*				GetSATAxis() const;
	Projection			Project(const Vec2& axis) const;


	float				GetMass() const;
	float				GetInertiaTensor() const;

	Vec2				GetPointVelocity(const Vec2& point) const;

	// Physics
	float				density;
	Vec2				speed;
	float				angularVelocity = 0.0f;
	Vec2				forces;
	float				torques = 0.0f;


private:
	void				CreateBuffers();
	void				BindBuffers();
	void				DestroyBuffers();

	void				BuildLines();

	bool				SatCollisionChecker(const CPolygon& poly, Vec2& colPoint, Vec2& colNormal, float& colDist, bool receiver) const;

	void				ComputeArea();
	void				RecenterOnCenterOfMass(); // Area must be computed
	void				ComputeLocalInertiaTensor(); // Must be centered on center of mass

	GLuint				m_vertexBufferId;
	size_t				m_index;

	std::vector<Line>	m_lines;

	float				m_signedArea;

	// Physics
	float				m_localInertiaTensor; // don't consider mass
};

typedef std::shared_ptr<CPolygon>	CPolygonPtr;

#endif