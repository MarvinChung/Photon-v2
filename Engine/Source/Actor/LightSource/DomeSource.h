#pragma once

#include "Actor/LightSource/EmitterBuildingMaterial.h"
#include "FileIO/SDL/TCommandInterface.h"
#include "Actor/LightSource/LightSource.h"
#include "Common/Logger.h"
#include "FileIO/FileSystem/Path.h"

#include <memory>

namespace ph
{

class DomeSource final : public LightSource, public TCommandInterface<DomeSource>
{
public:
	DomeSource();
	DomeSource(const Path& sphericalEnvMap);
	virtual ~DomeSource() override;

	virtual std::unique_ptr<Emitter> genEmitter(
		CookingContext& context, EmitterBuildingMaterial&& data) const override;

	virtual std::shared_ptr<Geometry> genGeometry(CookingContext& context) const override;

	virtual std::shared_ptr<Material> genMaterial(CookingContext& context) const override;

private:
	Path m_sphericalEnvMap;

// command interface
public:
	DomeSource(const InputPacket& packet);
	static SdlTypeInfo ciTypeInfo();
	static void ciRegister(CommandRegister& cmdRegister);

	static const Logger logger;
};

}// end namespace ph