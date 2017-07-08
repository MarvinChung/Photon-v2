#pragma once

#include "Core/Camera/Camera.h"
#include "Common/primitive_type.h"
#include "Math/TVector3.h"
#include "Math/Vector2f.h"
#include "FileIO/SDL/ISdlResource.h"
#include "FileIO/SDL/TCommandInterface.h"

#include <iostream>

namespace ph
{

class Ray;
class Sample;
class SampleGenerator;
class Film;
class InputPacket;

class PerspectiveCamera : public Camera, public TCommandInterface<PerspectiveCamera>
{
public:
	virtual ~PerspectiveCamera() = 0;

	virtual void genSensingRay(const Sample& sample, Ray* const out_ray) const = 0;
	virtual void evalEmittedImportanceAndPdfW(
		const Vector3R& targetPos, 
		Vector2f* const out_filmCoord, 
		Vector3R* const out_importance, 
		real* out_filmArea, 
		real* const out_pdfW) const = 0;

	virtual void onFilmSet(Film* newFilm) override;

protected:
	std::shared_ptr<Transform> m_rasterToCamera;
	std::shared_ptr<Transform> m_cameraToWorld;
	std::shared_ptr<Transform> m_rasterToWorld;
	
private:
	real m_filmWidthMM;
	real m_filmOffsetMM;

	void updateTransforms();

// command interface
public:
	PerspectiveCamera(const InputPacket& packet);
	static SdlTypeInfo ciTypeInfo();
	static ExitStatus ciExecute(const std::shared_ptr<PerspectiveCamera>& targetResource, const std::string& functionName, const InputPacket& packet);
};

}// end namespace ph