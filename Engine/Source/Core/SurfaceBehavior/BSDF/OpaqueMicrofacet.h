#pragma once

#include "Core/SurfaceBehavior/BSDF.h"
#include "Math/TVector3.h"
#include "Actor/Texture/Texture.h"
#include "Actor/Texture/ConstantTexture.h"
#include "Core/SurfaceBehavior/Utility/Microfacet.h"
#include "Core/SurfaceBehavior/Utility/FresnelEffect.h"

#include <memory>

namespace ph
{

class OpaqueMicrofacet final : public BSDF
{
public:
	OpaqueMicrofacet();
	virtual ~OpaqueMicrofacet() override;

	inline void setAlbedo(const std::shared_ptr<Texture>& albedo)
	{
		m_albedo = albedo;
	}

	inline void setMicrofacet(const std::shared_ptr<Microfacet>& microfacet)
	{
		m_microfacet = microfacet;
	}

	inline void setFresnelEffect(const std::shared_ptr<FresnelEffect>& fresnel)
	{
		m_fresnel = fresnel;
	}

private:
	virtual void evaluate(const Intersection& X, const Vector3R& L, const Vector3R& V,
	                      SpectralStrength* out_bsdf, 
	                      ESurfacePhenomenon* out_type) const override;

	virtual void genSample(const Intersection& X, const Vector3R& V,
	                       Vector3R* out_L, 
	                       SpectralStrength* out_pdfAppliedBsdf, 
	                       ESurfacePhenomenon* out_type) const override;

	virtual void calcSampleDirPdfW(const Intersection& X, const Vector3R& L, const Vector3R& V, 
	                               const ESurfacePhenomenon& type,
	                               real* out_pdfW) const override;

private:
	std::shared_ptr<Texture>       m_albedo;
	std::shared_ptr<Microfacet>    m_microfacet;
	std::shared_ptr<FresnelEffect> m_fresnel;
};

}// end namespace ph