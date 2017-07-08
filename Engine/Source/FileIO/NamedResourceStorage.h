#pragma once

#include "Actor/Geometry/Geometry.h"
#include "Actor/Texture/Texture.h"
#include "Actor/Material/Material.h"
#include "Actor/LightSource/LightSource.h"
#include "Actor/AModel.h"
#include "Actor/ALight.h"
#include "FileIO/DataTreatment.h"
#include "FileIO/SDL/SdlTypeInfo.h"
#include "FileIO/SDL/ISdlResource.h"

#include <unordered_map>
#include <string>
#include <memory>
#include <vector>
#include <iostream>

namespace ph
{

class NamedResourceStorage final
{
public:
	NamedResourceStorage();

	void addResource(
		const SdlTypeInfo& typeInfo,
		const std::string& resourceName,
		const std::shared_ptr<ISdlResource>& resource);

	std::shared_ptr<ISdlResource> getResource(
		const SdlTypeInfo& typeInfo,
		const std::string& resourceName,
		const DataTreatment& treatment = DataTreatment()) const;

	template<typename T>
	std::shared_ptr<T> getResource(
		const std::string& resourceName, 
		const DataTreatment& treatment = DataTreatment()) const;

	std::vector<std::shared_ptr<Actor>> getActors() const;

private:
	std::vector<std::unordered_map<std::string, std::shared_ptr<ISdlResource>>> m_resources;
	
private:
	static void reportResourceNotFound(const std::string& categoryName, const std::string& name, const DataTreatment& treatment);
	static bool checkCategoryIndex(const std::size_t index);
};

// template implementations:

template<typename T>
std::shared_ptr<T> NamedResourceStorage::getResource(
	const std::string& resourceName, 
	const DataTreatment& treatment) const
{
	/*std::shared_ptr<T> resource = nullptr;

	const SdlTypeInfo& typeInfo      = T::ciTypeInfo();
	const std::size_t  categoryIndex = static_cast<std::size_t>(typeInfo.typeCategory);
	if(!checkCategoryIndex(categoryIndex))
	{
		return resource;
	}

	const auto& resourcesNameMap = m_resources[categoryIndex];
	const auto& iter = resourcesNameMap.find(resourceName);
	if(iter == resourcesNameMap.end())
	{
		reportResourceNotFound(SdlTypeInfo::categoryToName(typeInfo.typeCategory), resourceName, treatment);
		return resource;
	}*/

	const SdlTypeInfo& typeInfo = T::ciTypeInfo();
	const std::shared_ptr<ISdlResource>& rawResource = getResource(typeInfo, resourceName, treatment);
	if(rawResource == nullptr)
	{
		return nullptr;
	}

	const std::shared_ptr<T> castedResource = std::dynamic_pointer_cast<T>(rawResource);
	if(castedResource == nullptr)
	{
		reportResourceNotFound(SdlTypeInfo::categoryToName(typeInfo.typeCategory), resourceName, treatment);
		std::cerr << "resource not found detail: requested type mismatch" << std::endl;
		return nullptr;
	}

	return castedResource;
}

}// end namespace ph