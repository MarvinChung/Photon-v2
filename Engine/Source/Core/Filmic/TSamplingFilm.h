#pragma once

#include "Core/Filmic/Film.h"
#include "Core/Bound/TAABB2D.h"
#include "Math/TVector2.h"
#include "Frame/frame_fwd.h"
#include "Core/Filmic/TMergeableFilm.h"
#include "Core/Filmic/SampleFilter.h"

#include <vector>
#include <functional>
#include <memory>

namespace ph
{

template<typename Sample>
class TSamplingFilm : public Film
{
public:
	TSamplingFilm(
		int64               actualWidthPx, 
		int64               actualHeightPx,
		const SampleFilter& filter);

	TSamplingFilm(
		int64                 actualWidthPx, 
		int64                 actualHeightPx,
		const TAABB2D<int64>& effectiveWindowPx,
		const SampleFilter&   filter);

	virtual void addSample(float64 xPx, float64 yPx, const Sample& sample) = 0;

	// Generates a child film with the same actual dimensions and filter as 
	// parent's, but potentially with a different effective window. Note that
	// the returned film depends on its parent, and it is caller's 
	// responsibility to make sure a child does not outlive its parent.
	virtual TMergeableFilm<Sample> genChild(const TAABB2D<int64>& effectiveWindowPx) = 0;

	void clear() override = 0;

	void setEffectiveWindowPx(const TAABB2D<int64>& effectiveWindow) override;

	TVector2<float64> getSampleResPx() const;
	const TAABB2D<float64>& getSampleWindowPx() const;
	const SampleFilter& getFilter() const;

private:
	void developRegion(HdrRgbFrame& out_frame, const TAABB2D<int64>& regionPx) const override = 0;

	SampleFilter     m_filter;
	TAABB2D<float64> m_sampleWindowPx;

	void updateSampleDimensions();
};

// In-header Implementations:

template<typename Sample>
inline const SampleFilter& TSamplingFilm<Sample>::getFilter() const
{
	return m_filter;
}

template<typename Sample>
inline TVector2<float64> TSamplingFilm<Sample>::getSampleResPx() const
{
	return {m_sampleWindowPx.getWidth(), m_sampleWindowPx.getHeight()};
}

template<typename Sample>
inline const TAABB2D<float64>& TSamplingFilm<Sample>::getSampleWindowPx() const
{
	return m_sampleWindowPx;
}

}// end namespace ph

#include "Core/Filmic/TSamplingFilm.ipp"