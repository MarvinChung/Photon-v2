#include "Core/SurfaceBehavior/SurfaceOptics/TranslucentMicrofacet.h"
#include "Core/Texture/TConstantTexture.h"
#include "Core/Ray.h"
#include "Math/TVector3.h"
#include "Math/Random.h"
#include "Math/constant.h"
#include "Core/SurfaceBehavior/Property/IsoTrowbridgeReitz.h"
#include "Math/Math.h"
#include "Core/SurfaceBehavior/Property/SchlickApproxDielectricFresnel.h"
#include "Core/SurfaceBehavior/BsdfHelper.h"
#include "Common/assertion.h"
#include "Core/SidednessAgreement.h"

#include <memory>
#include <iostream>

namespace ph
{

TranslucentMicrofacet::TranslucentMicrofacet() :
	SurfaceOptics(),
	m_fresnel   (std::make_shared<SchlickApproxDielectricFresnel>(1.0_r, 1.5_r)),
	m_microfacet(std::make_shared<IsoTrowbridgeReitz>(0.5_r))
{
	m_phenomena.set({ESP::GLOSSY_REFLECTION, ESP::GLOSSY_TRANSMISSION});
}

TranslucentMicrofacet::~TranslucentMicrofacet() = default;

void TranslucentMicrofacet::evalBsdf(
	const SurfaceHit&         X,
	const Vector3R&           L,
	const Vector3R&           V,
	const SidednessAgreement& sidedness,
	SpectralStrength* const   out_bsdf) const
{
	PH_ASSERT(out_bsdf);

	const Vector3R& N = X.getShadingNormal();

	const real NoL = N.dot(L);
	const real NoV = N.dot(V);

	// reflection
	if(sidedness.isSameHemisphere(X, L, V))
	{
		Vector3R H;
		if(!BsdfHelper::makeHalfVectorSameHemisphere(L, V, N, &H))
		{
			out_bsdf->setValues(0);
			return;
		}

		const real HoV = H.dot(V);
		const real NoH = N.dot(H);
		const real HoL = H.dot(L);

		SpectralStrength F;
		m_fresnel->calcReflectance(HoL, &F);

		const real D = m_microfacet->distribution(X, N, H);
		const real G = m_microfacet->shadowing(X, N, H, L, V);

		*out_bsdf = F.mul(D * G / (4.0_r * std::abs(NoV * NoL)));
	}
	// refraction
	else if(sidedness.isOppositeHemisphere(X, L, V))
	{
		real etaI = m_fresnel->getIorOuter();
		real etaT = m_fresnel->getIorInner();
		if(NoL < 0.0_r)
		{
			std::swap(etaI, etaT);
		}

		// H should be on the same hemisphere as N
		Vector3R H = L.mul(-etaI).add(V.mul(-etaT));
		if(H.isZero())
		{
			out_bsdf->setValues(0);
			return;
		}
		H.normalizeLocal();
		if(N.dot(H) < 0.0_r)
		{
			H.mulLocal(-1.0_r);
		}

		const real HoV = H.dot(V);
		const real NoH = N.dot(H);
		const real HoL = H.dot(L);

		SpectralStrength F;
		m_fresnel->calcTransmittance(HoL, &F);

		const real D = m_microfacet->distribution(X, N, H);
		const real G = m_microfacet->shadowing(X, N, H, L, V);

		const real dotTerm = std::abs(HoL * HoV / (NoV * NoL));
		const real iorTerm = etaI / (etaI * HoL + etaT * HoV);
		*out_bsdf = F.complement().mul(D * G * dotTerm * (iorTerm * iorTerm));
	}
	else
	{
		out_bsdf->setValues(0);
	}
}

void TranslucentMicrofacet::genBsdfSample(
	const SurfaceHit&         X,
	const Vector3R&           V,
	const SidednessAgreement& sidedness,
	Vector3R* const           out_L,
	SpectralStrength* const   out_pdfAppliedBsdf) const
{
	PH_ASSERT(out_L && out_pdfAppliedBsdf);

	out_pdfAppliedBsdf->setValues(0);

	// Cook-Torrance microfacet specular BRDF for translucent surface is:
	// |HoL||HoV|/(|NoL||NoV|)*(iorO^2)*(D(H)*F(V, H)*G(L, V, H)) / (iorI*HoL + iorO*HoV)^2.
	// The importance sampling strategy is to generate a microfacet normal (H) which follows D(H)'s distribution, and
	// generate L by reflecting/refracting -V using H.
	// The PDF for this sampling scheme is (D(H)*|NoH|) * (iorO^2 * |HoV| / ((iorI*HoL + iorO*HoV)^2)).
	// The reason that the latter multiplier in the PDF exists is because there's a jacobian involved 
	// (from H's probability space to L's).

	const Vector3R& N = X.getShadingNormal();

	Vector3R H;
	m_microfacet->genDistributedH(X, 
	                              Random::genUniformReal_i0_e1(),
	                              Random::genUniformReal_i0_e1(), 
	                              N, &H);

	const real NoV = N.dot(V);
	const real HoV = H.dot(V);
	const real NoH = N.dot(H);

	SpectralStrength F;
	m_fresnel->calcReflectance(HoV, &F);

	// use Fresnel term to select which path to take and calculate L

	const real dart = Random::genUniformReal_i0_e1();
	const real reflectProb = F.avg();

	// reflect path
	if(dart < reflectProb)
	{
		// calculate reflected L
		*out_L = V.mul(-1.0_r).reflect(H).normalizeLocal();
		if(!sidedness.isSameHemisphere(X, V, *out_L))
		{
			return;
		}

		// account for probability
		F.divLocal(reflectProb);
	}
	// refract path
	else if(m_fresnel->calcRefractDir(V, H, out_L))
	{
		if(!sidedness.isOppositeHemisphere(X, V, *out_L))
		{
			return;
		}

		m_fresnel->calcTransmittance(out_L->dot(H), &F);

		// account for probability
		F.divLocal(1.0_r - reflectProb);
	}
	else
	{
		// RARE: may be called due to numerical error
		out_pdfAppliedBsdf->setValues(0);
		return;
	}

	const Vector3R& L = *out_L;

	const real NoL = N.dot(L);
	const real HoL = H.dot(L);

	const real G = m_microfacet->shadowing(X, N, H, L, V);

	const real dotTerms = std::abs(HoL / (NoV * NoL * NoH));
	out_pdfAppliedBsdf->setValues(F.mul(G * dotTerms));
}

void TranslucentMicrofacet::calcBsdfSamplePdf(
	const SurfaceHit&         X,
	const Vector3R&           L,
	const Vector3R&           V,
	const SidednessAgreement& sidedness,
	real* const               out_pdfW) const
{
	PH_ASSERT(out_pdfW);

	const Vector3R& N = X.getShadingNormal();

	// reflection
	if(sidedness.isSameHemisphere(X, L, V))
	{
		Vector3R H;
		if(!BsdfHelper::makeHalfVectorSameHemisphere(L, V, N, &H))
		{
			*out_pdfW = 0;
			return;
		}

		const real NoH = N.dot(H);
		const real HoL = H.dot(L);
		const real HoV = H.dot(V);
		const real D = m_microfacet->distribution(X, N, H);

		SpectralStrength F;
		m_fresnel->calcReflectance(HoL, &F);
		const real reflectProb = F.avg();

		*out_pdfW = std::abs(D * NoH / (4.0_r * HoL)) * reflectProb;
	}
	// refraction
	else if(sidedness.isOppositeHemisphere(X, L, V))
	{
		const real NoV = N.dot(V);
		const real NoL = N.dot(L);

		real etaI = m_fresnel->getIorOuter();
		real etaT = m_fresnel->getIorInner();
		if(NoL < 0.0_r)
		{
			std::swap(etaI, etaT);
		}

		// H should be on the same hemisphere as N
		Vector3R H = L.mul(-etaI).add(V.mul(-etaT));
		if(H.isZero())
		{
			*out_pdfW = 0;
			return;
		}
		H.normalizeLocal();
		if(N.dot(H) < 0.0_r)
		{
			H.mulLocal(-1.0_r);
		}

		const real HoV = H.dot(V);
		const real NoH = N.dot(H);
		const real HoL = H.dot(L);

		const real D = m_microfacet->distribution(X, N, H);

		SpectralStrength F;
		m_fresnel->calcReflectance(HoL, &F);
		const real refractProb = 1.0_r - F.avg();

		const real iorTerm = etaI * HoL + etaT * HoV;
		const real multiplier = (etaI * etaI * HoL) / (iorTerm * iorTerm);

		*out_pdfW = std::abs(D * NoH * multiplier) * refractProb;
	}
	else
	{
		*out_pdfW = 0.0_r;
	}
}

}// end namespace ph