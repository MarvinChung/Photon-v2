#include "Core/MtImportanceRenderer.h"
#include "Common/primitive_type.h"
#include "Image/Film.h"
#include "World/World.h"
#include "Camera/Camera.h"
#include "Core/Ray.h"
#include "Core/Intersection.h"
#include "Model/Material/Material.h"
#include "Model/Material/SurfaceIntegrand.h"
#include "Math/constant.h"
#include "Core/SampleGenerator.h"
#include "Core/Sample.h"
#include "Math/random_number.h"
#include "Math/Color.h"
#include "Math/Math.h"
#include "Core/BackwardPathIntegrator.h"

#include <cmath>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

namespace ph
{

const uint32 MtImportanceRenderer::nThreads;

MtImportanceRenderer::MtImportanceRenderer()
{
	for(std::size_t threadIndex = 0; threadIndex < nThreads; threadIndex++)
	{
		m_workerProgresses.push_back(std::make_unique<std::atomic<float32>>(0.0f));
		m_workerSampleFrequencies.push_back(std::make_unique<std::atomic<float32>>(0.0f));
	}
}

MtImportanceRenderer::~MtImportanceRenderer() = default;

void MtImportanceRenderer::render(const World& world, const Camera& camera) const
{
	m_subFilms.clear();
	m_subFilms.shrink_to_fit();

	BackwardPathIntegrator integrator;
	integrator.cook(world);

	std::atomic<int32> numSpp = 0;
	std::thread renderWorkers[nThreads];
	std::vector<std::unique_ptr<SampleGenerator>> subSampleGenerators;

	m_subFilms = std::vector<Film>(nThreads, Film(camera.getFilm()->getWidthPx(), camera.getFilm()->getHeightPx()));
	m_sampleGenerator->split(nThreads, &subSampleGenerators);

	for(std::size_t threadIndex = 0; threadIndex < nThreads; threadIndex++)
	{
		SampleGenerator*      subSampleGenerator = subSampleGenerators[threadIndex].get();
		Film*                 subFilm            = &(m_subFilms[threadIndex]);
		std::atomic<float32>* workerProgress     = m_workerProgresses[threadIndex].get();
		std::atomic<float32>* workerSampleFreq   = m_workerSampleFrequencies[threadIndex].get();

		renderWorkers[threadIndex] = std::thread([this, &camera, &integrator, &world, &numSpp, subSampleGenerator, subFilm, workerProgress, workerSampleFreq]()
		{
		// ****************************** thread start ****************************** //

		const uint32 widthPx = camera.getFilm()->getWidthPx();
		const uint32 heightPx = camera.getFilm()->getHeightPx();
		const float32 aspectRatio = static_cast<float32>(widthPx) / static_cast<float32>(heightPx);

		std::vector<Sample> samples;

		Ray primaryRay;
		Vector3f radiance;

		const uint32 totalSpp = subSampleGenerator->getSppBudget();
		uint32 currentSpp = 0;

		std::chrono::time_point<std::chrono::system_clock> t1;
		std::chrono::time_point<std::chrono::system_clock> t2;

		while(subSampleGenerator->hasMoreSamples())
		{
			t1 = std::chrono::system_clock::now();

			samples.clear();
			subSampleGenerator->requestMoreSamples(*subFilm, &samples);

			Sample sample;
			while(!samples.empty())
			{
				sample = samples.back();
				samples.pop_back();
				camera.genSampleRay(sample, &primaryRay, aspectRatio);

				integrator.radianceAlongRay(primaryRay, world, &radiance);

				uint32 x = static_cast<uint32>((sample.m_cameraX + 1.0f) / 2.0f * widthPx);
				uint32 y = static_cast<uint32>((sample.m_cameraY + 1.0f) / 2.0f * heightPx);
				if(x >= widthPx) x = widthPx - 1;
				if(y >= heightPx) y = heightPx - 1;

				subFilm->accumulateRadiance(x, y, radiance);
			}// end while

			currentSpp++;
			*workerProgress = static_cast<float32>(currentSpp) / static_cast<float32>(totalSpp);

			m_rendererMutex.lock();
			std::cout << "SPP: " << ++numSpp << std::endl;
			m_rendererMutex.unlock();

			t2 = std::chrono::system_clock::now();

			auto msPassed = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
			*workerSampleFreq = static_cast<float32>(widthPx * heightPx) / static_cast<float32>(msPassed.count()) * 1000.0f;
		}

		m_rendererMutex.lock();
		camera.getFilm()->accumulateRadiance(*subFilm);
		m_rendererMutex.unlock();

		// ****************************** thread end ****************************** //
		});
	}

	for(auto& renderWorker : renderWorkers)
	{
		renderWorker.join();
	}
}

float32 MtImportanceRenderer::queryPercentageProgress() const
{
	float32 avgWorkerProgress = 0.0f;
	for(uint32 threadId = 0; threadId < m_workerProgresses.size(); threadId++)
	{
		avgWorkerProgress += *(m_workerProgresses[threadId]);
	}
	avgWorkerProgress /= static_cast<float32>(m_workerProgresses.size());

	return avgWorkerProgress * 100.0f;
}

float32 MtImportanceRenderer::querySampleFrequency() const
{
	float32 avgWorkerSampleFreq = 0.0f;
	for(uint32 threadId = 0; threadId < m_workerSampleFrequencies.size(); threadId++)
	{
		avgWorkerSampleFreq += *(m_workerSampleFrequencies[threadId]);
	}
	avgWorkerSampleFreq /= static_cast<float32>(m_workerSampleFrequencies.size());

	return avgWorkerSampleFreq;
}

}// end namespace ph