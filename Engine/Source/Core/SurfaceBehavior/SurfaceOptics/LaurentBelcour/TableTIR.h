#pragma once

#include "FileIO/BinaryFileReader.h"
#include "FileIO/FileSystem/Path.h"
#include "Common/Logger.h"
#include "Common/assertion.h"
#include "Common/primitive_type.h"

#include <vector>

namespace ph
{

class TableTIR final
{
public:
	explicit TableTIR(const Path& tableFilePath);

	real sample(real cosWi, real alpha, real relIor) const;

private:
	std::vector<float> m_table;

	int m_numCosWi;
	int m_numAlpha;
	int m_numRelIor;

	float m_minCosWi,  m_maxCosWi;
	float m_minAlpha,  m_maxAlpha;
	float m_minRelIor, m_maxRelIor;

	static const Logger logger;

	int calcIndex(int iCosWi, int iAlpha, int iRelIor) const;
};

// In-header Implementations:

inline TableTIR::TableTIR(const Path& tableFilePath) :
	m_table(),

	m_numCosWi (0),
	m_numAlpha (0),
	m_numRelIor(0),

	m_minCosWi (0.0f), m_maxCosWi (0.0f),
	m_minAlpha (0.0f), m_maxAlpha (0.0f),
	m_minRelIor(0.0f), m_maxRelIor(0.0f)
{
	logger.log(ELogLevel::NOTE_MED, "loading <" + tableFilePath.toString() + ">");

	BinaryFileReader reader(tableFilePath);
	if(!reader.open())
	{
		return;
	}

	reader.read(&m_numCosWi);
	reader.read(&m_numAlpha);
	reader.read(&m_numRelIor);
	reader.read(&m_minCosWi);  reader.read(&m_maxCosWi);
	reader.read(&m_minAlpha);  reader.read(&m_maxAlpha);
	reader.read(&m_minRelIor); reader.read(&m_maxRelIor);

	logger.log(ELogLevel::DEBUG_MED, "dimension: "
		"(cos-w_i = " +      std::to_string(m_numCosWi) + ", "
		 "alpha = " +        std::to_string(m_numAlpha) + ", "
		 "relative-IOR = " + std::to_string(m_numRelIor) + ")");
	logger.log(ELogLevel::DEBUG_MED, "range: "
		"(cos-w_i = [" +      std::to_string(m_minCosWi) +  ", " + std::to_string(m_maxCosWi) +  "], "
		 "alpha = [" +        std::to_string(m_minAlpha) +  ", " + std::to_string(m_maxAlpha) +  "], "
		 "relative-IOR = [" + std::to_string(m_minRelIor) + ", " + std::to_string(m_maxRelIor) + "])");

	PH_ASSERT(m_numCosWi > 0 && m_numAlpha > 0 && m_numRelIor > 0);

	const std::size_t tableSize = 
		static_cast<std::size_t>(m_numCosWi) *
		static_cast<std::size_t>(m_numAlpha) *
		static_cast<std::size_t>(m_numRelIor);
	m_table.resize(tableSize, 0.0f);
	reader.read(m_table.data(), m_table.size());
}

inline int TableTIR::calcIndex(const int iCosWi, const int iAlpha, const int iRelIor) const
{
	// make sure the indices stay in the limits
	PH_ASSERT(0 <= iCosWi  && iCosWi  < m_numCosWi);
	PH_ASSERT(0 <= iAlpha  && iAlpha  < m_numAlpha);
	PH_ASSERT(0 <= iRelIor && iRelIor < m_numRelIor);

	return iRelIor + m_numRelIor * (iAlpha + m_numAlpha * iCosWi);
}

}// end namespace ph