#include "Core/Quantity/SpectralData.h"
#include "Common/assertion.h"
#include "Math/Function/TPiecewiseLinear1D.h"
#include "Math/Solver/TAnalyticalIntegrator1D.h"

namespace ph
{

SampledSpectralStrength SpectralData::calcPiecewiseAveraged(
	const real* const wavelengthsNm, const real* const values, const std::size_t numPoints)
{
	PH_ASSERT(wavelengthsNm != nullptr && values != nullptr);

	// construct a curve from specified points
	//
	TPiecewiseLinear1D<real> curve;
	for(std::size_t i = 0; i < numPoints; i++)
	{
		const real wavelengthNm = wavelengthsNm[i];
		const real value        = values[i];

		curve.addPoint({wavelengthNm, value});
	}
	curve.update();

	// sample curve values by averaging each wavelength interval
	//
	SampledSpectralStrength       sampled;
	TAnalyticalIntegrator1D<real> areaCalculator;
	for(std::size_t i = 0; i < SampledSpectralStrength::NUM_INTERVALS; i++)
	{
		const auto& range = SampledSpectralStrength::lambdaRangeNmOf(i);

		areaCalculator.setIntegrationDomain(range.first, range.second);

		const real area         = areaCalculator.integrate(curve);
		const real averageValue = area / (range.second - range.first);
		sampled[i] = averageValue;
	}
	
	return sampled;
}

}// end namespace ph