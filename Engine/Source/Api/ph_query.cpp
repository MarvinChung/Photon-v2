#include "ph_query.h"
#include "Api/ApiDatabase.h"
#include "Core/Renderer.h"
#include "Image/Film.h"

#include <iostream>

void phQueryRendererPercentageProgress(const PHuint64 rendererId, PHfloat32* const out_percentage)
{
	using namespace ph;

	Renderer* renderer = ApiDatabase::getRenderer(rendererId);
	if(renderer)
	{
		*out_percentage = renderer->queryPercentageProgress();
	}
}