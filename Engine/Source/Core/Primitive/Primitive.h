#pragma once

#include "Common/primitive_type.h"

namespace ph
{

class Ray;
class Intersection;
class AABB;
class PrimitiveMetadata;
class Vector3f;
class PositionSample;

class Primitive
{
public:
	Primitive(const PrimitiveMetadata* const metadata);
	virtual ~Primitive() = 0;

	virtual bool isIntersecting(const Ray& ray, Intersection* const out_intersection) const = 0;
	virtual bool isIntersecting(const Ray& ray) const = 0;
	virtual bool isIntersectingVolume(const AABB& aabb) const = 0;
	virtual void calcAABB(AABB* const out_aabb) const = 0;
	virtual void genPositionSample(PositionSample* const out_sample) const = 0;

	inline const PrimitiveMetadata* getMetadata() const
	{
		return m_metadata;
	}

	inline float32 getReciExtendedArea() const
	{
		return m_reciExtendedArea;
	}

	virtual float32 calcExtendedArea() const = 0;

protected:
	const PrimitiveMetadata* m_metadata;
	float32 m_reciExtendedArea;
};

}// end namespace ph