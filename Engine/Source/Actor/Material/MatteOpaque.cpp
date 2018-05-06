#include "Actor/Material/MatteOpaque.h"
#include "Core/Texture/TConstantTexture.h"
#include "FileIO/InputPacket.h"
#include "FileIO/InputPrototype.h"
#include "Actor/Image/Image.h"
#include "Actor/Image/ConstantImage.h"
#include "FileIO/PictureLoader.h"
#include "Actor/Image/LdrPictureImage.h"

namespace ph
{

MatteOpaque::MatteOpaque() : 
	MatteOpaque(Vector3R(0.5_r))
{}

MatteOpaque::MatteOpaque(const Vector3R& linearSrgbAlbedo) : 
	Material(),
	m_albedo(nullptr)
{
	setAlbedo(linearSrgbAlbedo);
}

MatteOpaque::~MatteOpaque() = default;

void MatteOpaque::genSurfaceBehavior(CookingContext& context, SurfaceBehavior* const out_surfaceBehavior) const
{
	std::unique_ptr<LambertianDiffuse> surfaceOptics = std::make_unique<LambertianDiffuse>();
	surfaceOptics->setAlbedo(m_albedo->genTextureSpectral(context));
	out_surfaceBehavior->setOptics(std::move(surfaceOptics));
}

void MatteOpaque::setAlbedo(const Vector3R& albedo)
{
	setAlbedo(albedo.x, albedo.y, albedo.z);
}

void MatteOpaque::setAlbedo(const real r, const real g, const real b)
{
	m_albedo = std::make_shared<ConstantImage>(std::vector<real>{r, g, b}, ConstantImage::EType::ECF_LINEAR_SRGB);
}

void MatteOpaque::setAlbedo(const std::shared_ptr<Image>& albedo)
{
	m_albedo = albedo;
}

// command interface

SdlTypeInfo MatteOpaque::ciTypeInfo()
{
	return SdlTypeInfo(ETypeCategory::REF_MATERIAL, "matte-opaque");
}

void MatteOpaque::ciRegister(CommandRegister& cmdRegister)
{
	SdlLoader loader;
	loader.setFunc<MatteOpaque>(ciLoad);
	cmdRegister.setLoader(loader);
}

std::unique_ptr<MatteOpaque> MatteOpaque::ciLoad(const InputPacket& packet)
{
	InputPrototype imageFileAlbedo;
	imageFileAlbedo.addString("albedo");

	InputPrototype constAlbedo;
	constAlbedo.addVector3r("albedo");

	std::unique_ptr<MatteOpaque> material = std::make_unique<MatteOpaque>();
	if(packet.isPrototypeMatched(imageFileAlbedo))
	{
		const Path& imagePath = packet.getStringAsPath("albedo", Path(), DataTreatment::REQUIRED());
		const auto& albedo    = std::make_shared<LdrPictureImage>(PictureLoader::loadLdr(imagePath));
		material->setAlbedo(albedo);
	}
	else if(packet.isPrototypeMatched(constAlbedo))
	{
		const Vector3R albedo = packet.getVector3r("albedo", Vector3R(0.5_r), 
			DataTreatment::OPTIONAL("all components are set to 0.5"));
		material->setAlbedo(albedo);
	}
	else
	{
		const auto& albedo = packet.get<Image>("albedo");
		if(albedo != nullptr)
		{
			material->setAlbedo(albedo);
		}
		else
		{
			std::cerr << "warning: at MatteOpaque::ciLoad(), " 
		              << "ill-formed input detected" << std::endl;
		}
	}
	
	return material;
}

}// end namespace ph