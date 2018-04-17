#pragma once

#include "Common/primitive_type.h"
#include "Actor/Material/Material.h"
#include "Core/SurfaceBehavior/SurfaceOptics.h"
#include "Core/Quantity/SpectralStrength.h"

#include <memory>
#include <functional>

namespace ph
{

class IdealSubstance : public Material, public TCommandInterface<IdealSubstance>
{
public:
	IdealSubstance();
	virtual ~IdealSubstance() override;

	virtual void populateSurfaceBehavior(CookingContext& context, SurfaceBehavior* out_surfaceBehavior) const override;

	void asDielectricReflector(real iorInner, real iorOuter);

	// FIXME: specifying ior-outer is redundent, f0 already includes this
	void asMetallicReflector(const Vector3R& linearSrgbF0, real iorOuter);

	void asTransmitter(real iorInner, real iorOuter);
	void asAbsorber();

private:
	std::function<std::unique_ptr<SurfaceOptics>()> m_opticsGenerator;

// command interface
public:
	static SdlTypeInfo ciTypeInfo();
	static void ciRegister(CommandRegister& cmdRegister);
	static std::unique_ptr<IdealSubstance> ciLoad(const InputPacket& packet);
};

}// end namespace ph