#pragma once

#include "Common/primitive_type.h"
#include "Core/Camera/Camera.h"
#include "World/VisualWorld.h"
#include "Core/Camera/Film.h"
#include "Core/Integrator/Integrator.h"
#include "Core/SampleGenerator/PixelJitterSampleGenerator.h"
#include "FileIO/RenderOption.h"
#include "FileIO/NamedResourceStorage.h"

#include <vector>
#include <memory>
#include <string>

namespace ph
{

class Description final
{
public:
	RenderOption         renderOption;
	NamedResourceStorage resources;

	VisualWorld visualWorld;

	Description();

	void update(const real deltaS);

	inline std::shared_ptr<Camera>                     getCamera()          const { return m_camera;          };
	inline std::shared_ptr<Film>                       getFilm()            const { return m_film;            };
	inline std::shared_ptr<Integrator>                 getIntegrator()      const { return m_integrator;      };
	inline std::shared_ptr<PixelJitterSampleGenerator> getSampleGenerator() const { return m_sampleGenerator; };

private:
	std::shared_ptr<Camera>                     m_camera;
	std::shared_ptr<Film>                       m_film;
	std::shared_ptr<Integrator>                 m_integrator;
	std::shared_ptr<PixelJitterSampleGenerator> m_sampleGenerator;
};

}// end namespace ph