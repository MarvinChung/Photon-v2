#pragma once

#include "Common/primitive_type.h"
#include "Math/TVector3.h"
#include "FileIO/SDL/TCommandInterface.h"
#include "Math/Transform/TDecomposedTransform.h"
#include "Core/Filmic/filmic_fwd.h"

#include <iostream>
#include <memory>

namespace ph
{

class Ray;
class Sample;
class SampleGenerator;
class InputPacket;
class Transform;
class RayDifferential;

class Camera : public TCommandInterface<Camera>
{
public:
	Camera();
	Camera(const Vector3R& position, const Vector3R& direction, const Vector3R& upAxis);
	virtual ~Camera() = default;

	// TODO: consider changing <filmNdcPos> to 64-bit float

	// Given a NDC position on the film, generate a corresponding ray
	// that would have hit that position from the light entry of the camera (i.e., 
	// the furthest plane parallel to film the camera's lens system can reach).
	virtual void genSensedRay(const Vector2R& filmNdcPos, Ray* out_ray) const = 0;

	// Given a ray generated by genSensedRay() along with the parameters for it, 
	// calculates differential information on the origin of the ray.
	// The default implementation uses numerical differentiation for 
	// the differentials.
	virtual void calcSensedRayDifferentials(const Vector2R& filmNdcPos, const Ray& sensedRay,
	                                        RayDifferential* out_result) const;

	virtual void evalEmittedImportanceAndPdfW(const Vector3R& targetPos, Vector2R* const out_filmCoord, Vector3R* const out_importance, real* out_filmArea, real* const out_pdfW) const = 0;

	virtual void setAspectRatio(real ratio);

	const Vector3R& getPosition() const;
	const Vector3R& getDirection() const;
	const Vector3R& getUpAxis() const;
	void getPosition(Vector3R* out_position) const;
	void getDirection(Vector3R* out_direction) const;
	real getAspectRatio() const;

protected:
	TDecomposedTransform<hiReal> m_cameraToWorldTransform;

private:
	Vector3R m_position;
	Vector3R m_direction;
	Vector3R m_upAxis;
	real     m_aspectRatio;

	void updateCameraToWorldTransform(const Vector3R& position, const Vector3R& direction, const Vector3R& upAxis);

// command interface
public:
	explicit Camera(const InputPacket& packet);
	static SdlTypeInfo ciTypeInfo();
	static void ciRegister(CommandRegister& cmdRegister);
};

// In-header Implementations:

inline void Camera::setAspectRatio(const real ratio)
{
	m_aspectRatio = ratio;
}

inline const Vector3R& Camera::getPosition() const
{
	return m_position;
}

inline const Vector3R& Camera::getDirection() const
{
	return m_direction;
}

inline const Vector3R& Camera::getUpAxis() const
{
	return m_upAxis;
}

inline void Camera::getPosition(Vector3R* const out_position) const
{
	m_position.set(out_position);
}

inline void Camera::getDirection(Vector3R* const out_direction) const
{
	m_direction.set(out_direction);
}

inline real Camera::getAspectRatio() const
{
	return m_aspectRatio;
}

}// end namespace ph

/*
	<SDL_interface>

	<category>  camera </category>
	<type_name> camera </type_name>

	<name> Camera </name>
	<description>
		A camera for observing the scene.
	</description>

	<command type="creator" intent="blueprint">
		<input name="position" type="vector3">
			<description>Position of the camera.</description>
		</input>
		<input name="direction" type="vector3">
			<description>Direction that this camera is looking at.</description>
		</input>
		<input name="up-axis" type="vector3">
			<description>The direction that this camera consider as upward.</description>
		</input>
	</command>

	</SDL_interface>
*/