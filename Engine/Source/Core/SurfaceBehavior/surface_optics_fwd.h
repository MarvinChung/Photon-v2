#pragma once

#include "Utility/TBitFlags.h"
#include "Common/primitive_type.h"

namespace ph
{

/*
	Available surface phenomena.
*/
enum class ESurfacePhenomenon : uint32
{
	DIFFUSE_REFLECTION  = uint32(1) << 0,
	DELTA_REFLECTION    = uint32(1) << 1,
	GLOSSY_REFLECTION   = uint32(1) << 2,
	DELTA_TRANSMISSION  = uint32(1) << 3,
	GLOSSY_TRANSMISSION = uint32(1) << 4
};

enum class ETransport
{
	RADIANCE,
	IMPORTANCE
};

using SurfacePhenomena = TBitFlags<uint32, ESurfacePhenomenon>;
using SurfaceElemental = int;

constexpr SurfaceElemental ALL_ELEMENTALS = -1;

}// end namespace ph