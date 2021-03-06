#include "Core/SurfaceBehavior/Property/SchlickApproxDielectricFresnel.h"

#include <cmath>

namespace ph
{

SchlickApproxDielectricFresnel::SchlickApproxDielectricFresnel(
	const real iorOuter, 
	const real iorInner) : 
	DielectricFresnel(iorOuter, iorInner)
{
	const real numi = std::pow(iorOuter - iorInner, 2);
	const real deno = std::pow(iorOuter + iorInner, 2);
	m_f0 = numi / deno;

	m_tirIorRatio2 = std::max(iorOuter, iorInner) / std::min(iorOuter, iorInner);
}

SchlickApproxDielectricFresnel::~SchlickApproxDielectricFresnel() = default;

void SchlickApproxDielectricFresnel::calcReflectance(
	const real cosThetaIncident,
	SpectralStrength* const out_reflectance) const
{
	real cosTheta = std::abs(cosThetaIncident);

	// According to SIGGRAPH 2015 Course: 
	// "Extending the Disney BRDF to a BSDF with Integrated Subsurface Scattering (Brent Burley)",
	// since Schlick's approximation erroneously ignores the critical angle during calculation,
	// to fix this, we can use the refraction angle instead when TIR is possible (i.e., the
	// situation where the IOR of the incident side is greater than the opposite side).
	//
	if((cosThetaIncident > 0 && m_iorOuter > m_iorInner) ||
	   (cosThetaIncident < 0 && m_iorInner > m_iorOuter))
	{
		const real sinT2 = (1.0_r - cosThetaIncident * cosThetaIncident) * m_tirIorRatio2;

		// handles TIR
		if(sinT2 >= 1.0_r)
		{
			out_reflectance->setValues(1.0_r);
			return;
		}

		cosTheta = std::sqrt(1.0_r - sinT2);
	}

	out_reflectance->setValues(m_f0 + (1.0_r - m_f0) * std::pow(1.0_r - cosTheta, 5));
}

}// end namespace ph