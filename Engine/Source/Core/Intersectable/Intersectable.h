#pragma once

#include "Core/Bound/TAABB3D.h"

namespace ph
{

class Ray;
class HitProbe;
class HitDetail;

class Intersectable
{
public:
	virtual ~Intersectable();

	// Checks whether the specified ray intersects this intersectable.
	// If there is an intersection, true is returned and a brief intersection
	// report is stored inside the probe. If there is no intersection, false is
	// returned and the state of the probe is undefined.
	//
	virtual bool isIntersecting(const Ray& ray, HitProbe& probe) const = 0;

	// This method calculates a detailed description of the intersection from
	// the ray and probe used for calling isIntersecting() (if an intersection 
	// is found). The process of calculating intersection detail will destroy
	// the input probe.
	//
	virtual void calcIntersectionDetail(const Ray& ray, HitProbe& probe,
	                                    HitDetail* out_detail) const = 0;

	virtual void calcAABB(AABB3D* out_aabb) const = 0;

	// If greater performance is desired, you can override the default 
	// implementation which simply calls isIntersecting(2) to do the job.
	//
	virtual bool isIntersecting(const Ray& ray) const;

	// The default implementation performs conservative intersecting test using the
	// AABB calculated by calcAABB(). Although false-positives are allowed for this
	// method, providing an implementation with higher accuracy is benefitial for
	// many algorithms used by the renderer.
	//
	virtual bool isIntersectingVolumeConservative(const AABB3D& volume) const;
};

}// end namespace ph