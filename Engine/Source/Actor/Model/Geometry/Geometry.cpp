#include "Actor/Model/Geometry/Geometry.h"
#include "Actor/Model/TextureMapper/TextureMapper.h"
#include "Actor/Model/TextureMapper/DefaultMapper.h"

namespace ph
{

Geometry::Geometry() : 
	m_textureMapper(std::make_shared<DefaultMapper>())
{

}

Geometry::~Geometry() = default;

void Geometry::setTextureMapper(const std::shared_ptr<TextureMapper>& textureMapper)
{
	m_textureMapper = textureMapper;
}

const TextureMapper* Geometry::getTextureMapper() const
{
	return m_textureMapper.get();
}

}// end namespace ph