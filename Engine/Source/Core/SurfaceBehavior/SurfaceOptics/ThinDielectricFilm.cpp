#include "Core/SurfaceBehavior/SurfaceOptics/ThinDielectricFilm.h"
#include "Core/SurfaceBehavior/Property/ExactDielectricFresnel.h"
#include "Common/assertion.h"
#include "Math/Random.h"
#include "Core/LTABuildingBlock/SidednessAgreement.h"
#include "Core/Texture/TConstantTexture.h"
#include "Core/Texture/TSampler.h"
#include "Math/math.h"

namespace ph
{

ThinDielectricFilm::ThinDielectricFilm(
	const std::shared_ptr<DielectricFresnel>& fresnel,
	const std::vector<SampledSpectralStrength>& reflectanceTable,
	const std::vector<SampledSpectralStrength>& transmittanceTable) :

	SurfaceOptics(),

	m_fresnel(fresnel),
	m_reflectanceTable(reflectanceTable),
	m_transmittanceTable(transmittanceTable)
{
	PH_ASSERT(fresnel);
	PH_ASSERT_EQ(reflectanceTable.size(), 91);
	PH_ASSERT_EQ(transmittanceTable.size(), 91);

	m_phenomena.set({ESurfacePhenomenon::DELTA_REFLECTION, ESurfacePhenomenon::DELTA_TRANSMISSION});
	m_numElementals = 2;
}

ESurfacePhenomenon ThinDielectricFilm::getPhenomenonOf(const SurfaceElemental elemental) const
{
	PH_ASSERT_LT(elemental, 2);

	return elemental == REFLECTION ? ESurfacePhenomenon::DELTA_REFLECTION : 
	                                 ESurfacePhenomenon::DELTA_TRANSMISSION;
}

void ThinDielectricFilm::calcBsdf(
	const BsdfEvaluation::Input& in,
	BsdfEvaluation::Output&      out,
	const SidednessAgreement&    sidedness) const
{
	out.bsdf.setValues(0.0_r);
}

void ThinDielectricFilm::calcBsdfSample(
	const BsdfSample::Input&  in,
	BsdfSample::Output&       out,
	const SidednessAgreement& sidedness) const
{
	const bool canReflect  = in.elemental == ALL_ELEMENTALS || in.elemental == REFLECTION;
	const bool canTransmit = in.elemental == ALL_ELEMENTALS || in.elemental == TRANSMISSION;

	if(!canReflect && !canTransmit)
	{
		out.setMeasurability(false);
		return;
	}

	const Vector3R& N = in.X.getShadingNormal();

	SpectralStrength F;
	m_fresnel->calcReflectance(N.dot(in.V), &F);
	const real reflectProb = F.avg();

	bool sampleReflect  = canReflect;
	bool sampleTransmit = canTransmit;

	// we cannot sample both path, choose one randomly
	if(sampleReflect && sampleTransmit)
	{
		const real dart = Random::genUniformReal_i0_e1();
		if(dart < reflectProb)
		{
			sampleTransmit = false;
		}
		else
		{
			sampleReflect = false;
		}
	}

	PH_ASSERT(sampleReflect || sampleTransmit);

	// calculate reflected L
	out.L = in.V.mul(-1.0_r).reflect(N).normalizeLocal();

	real degree = math::to_degrees(N.absDot(out.L));
	std::size_t index = math::clamp(static_cast<std::size_t>(degree + 0.5_r), std::size_t(0), std::size_t(90));

	SampledSpectralStrength scale(0);

	if(sampleReflect)
	{
		if(!sidedness.isSameHemisphere(in.X, in.V, out.L))
		{
			out.setMeasurability(false);
			return;
		}

		scale = m_reflectanceTable[index];

		// account for probability
		if(in.elemental == ALL_ELEMENTALS)
		{
			scale.divLocal(reflectProb);
		}
	}
	else if(sampleTransmit && m_fresnel->calcRefractDir(in.V, N, &(out.L)))
	{
		if(!sidedness.isOppositeHemisphere(in.X, in.V, out.L))
		{
			out.setMeasurability(false);
			return;
		}

		scale = m_transmittanceTable[index];

		/*if(in.transported == ETransport::RADIANCE)
		{
			real etaI = m_fresnel->getIorOuter();
			real etaT = m_fresnel->getIorInner();
			if(N.dot(out.L) < 0.0_r)
			{
				std::swap(etaI, etaT);
			}
			F.mulLocal(etaT * etaT / (etaI * etaI));
		}*/

		// account for probability
		if(in.elemental == ALL_ELEMENTALS)
		{
			scale.divLocal(1.0_r - reflectProb);
		}
	}
	else
	{
		// RARE: may be called due to numerical error
		out.setMeasurability(false);
		return;
	}

	SpectralStrength value;
	value.setSampled(scale / N.absDot(out.L));
	out.pdfAppliedBsdf.setValues(value);
	out.setMeasurability(true);
}

void ThinDielectricFilm::calcBsdfSamplePdfW(
	const BsdfPdfQuery::Input& in,
	BsdfPdfQuery::Output&      out,
	const SidednessAgreement&  sidedness) const
{
	out.sampleDirPdfW = 0.0_r;
}

}// end namespace ph