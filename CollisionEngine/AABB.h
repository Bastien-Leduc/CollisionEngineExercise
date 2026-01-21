#pragma once
#include <memory>
#include "Polygon.h"
#include "Maths.h"

struct AABB
{
	AABB(float _maxX = FLT_MIN, float _minX = FLT_MAX, float _maxY = FLT_MIN, float _minY = FLT_MAX);
	AABB* Merge(AABB* other);
	bool Contain(AABB* other) const;
	bool Collide(AABB* other) const;
	float Volume() const;

	float maxX;
	float minX;
	float maxY;
	float minY;

	CPolygonPtr polyRef;
};

