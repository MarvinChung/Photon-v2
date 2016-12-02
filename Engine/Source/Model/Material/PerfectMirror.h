#pragma once

#include "Model/Material/Material.h"
#include "Model/Material/Integrand/SiPerfectMirror.h"

namespace ph
{

class PerfectMirror : public Material
{
public:
	virtual ~PerfectMirror() override;

	virtual const SurfaceIntegrand* getSurfaceIntegrand() const override;

private:
	SiPerfectMirror m_surfaceIntegrand;
};

}// end namespace ph