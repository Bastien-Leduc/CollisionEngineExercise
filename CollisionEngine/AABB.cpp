#include "AABB.h"
#include <algorithm>


AABB::AABB(float _maxX, float _minX, float _maxY, float _minY)
	: maxX(_maxX), minX(_minX), maxY(_maxY), minY(_minY), polyRef(nullptr)
{
}

AABB* AABB::Merge(AABB* other)
{
	const float _maxX = (other->maxX > maxX) ? other->maxX : maxX;
	const float _maxY = (other->maxY > maxY) ? other->maxY : maxY;
	const float _minX = (other->minX < minX) ? other->minX : minX;
	const float _minY = (other->minY < minY) ? other->minY : minY;

	AABB* newAABB = new AABB(_maxX, _minX, _maxY, _minY);
	return newAABB;
}

bool AABB::Contain(AABB* other) const
{
	return (minX <= other->minX && maxX >= other->maxX)
		&& (minY <= other->minY && maxY >= other->maxY);
}

bool AABB::Collide(AABB* other) const
{
	return (maxX >= other->minX && minX <= other->maxX)
		&& (maxY >= other->minY && minY <= other->maxY);
}

float AABB::Volume() const
{
	const float length = maxX - minX;
	const float height = maxY - minY;
	return length * height;
}
