#pragma once

#include "Core/Emitter/Sampler/EmitterSampler.h"
#include "Math/Random/TPwcDistribution1D.h"

#include <vector>
#include <unordered_map>

namespace ph
{

class ESPowerFavoring : public EmitterSampler
{
public:
	void update(const CookedDataStorage& cookedActors) override;
	const Emitter* pickEmitter(real* const out_PDF) const override;
	void genDirectSample(DirectLightSample& sample) const override;
	real calcDirectPdfW(const SurfaceHit& emitPos, const Vector3R& targetPos) const override;

private:
	std::vector<const Emitter*> m_emitters;
	TPwcDistribution1D<real>    m_distribution;

	std::unordered_map<
		const Emitter*, std::size_t
	> m_emitterToIndexMap;
};

}// end namespace ph