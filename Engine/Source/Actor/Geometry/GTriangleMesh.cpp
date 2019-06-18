#include "Actor/Geometry/GTriangleMesh.h"
#include "Actor/Geometry/GTriangle.h"
#include "Core/Intersectable/PrimitiveMetadata.h"
#include "Actor/AModel.h"
#include "Actor/Geometry/PrimitiveBuildingMaterial.h"
#include "FileIO/SDL/InputPacket.h"
#include "Actor/Geometry/GeometrySoup.h"
#include "Common/Logger.h"
#include "Core/Intersectable/IntelSimdBvh/QBVH.h"
#include "Utility/Timer.h"
#include "Core/Intersectable/PTriangle.h"

#include <iostream>

#define SIMD 1

namespace ph
{

namespace
{
	const Logger logger(LogSender("Triangle Mesh"));
}

GTriangleMesh::GTriangleMesh() : 
	Geometry(), 
	m_gTriangles()
{}

GTriangleMesh::GTriangleMesh(const std::vector<Vector3R>& positions,
                             const std::vector<Vector3R>& texCoords,
                             const std::vector<Vector3R>& normals) : 
	GTriangleMesh()
{
	if(!(positions.size() == texCoords.size() && texCoords.size() == normals.size()) ||
	    (positions.empty() || texCoords.empty() || normals.empty()) ||
	    (positions.size() % 3 != 0 || texCoords.size() % 3 != 0 || normals.size() % 3 != 0))
	{
		std::cerr << "warning: at GTriangleMesh::GTriangleMesh(), " 
		          << "bad input detected" << std::endl;
		return;
	}

	for(std::size_t i = 0; i < positions.size(); i += 3)
	{
		GTriangle triangle(positions[i + 0], positions[i + 1], positions[i + 2]);
		if(triangle.isDegenerate())
		{
			continue;
		}

		triangle.setUVWa(texCoords[i + 0]);
		triangle.setUVWb(texCoords[i + 1]);
		triangle.setUVWc(texCoords[i + 2]);
		triangle.setNa(normals[i + 0].lengthSquared() > 0 ? normals[i + 0].normalize() : Vector3R(0, 1, 0));
		triangle.setNb(normals[i + 1].lengthSquared() > 0 ? normals[i + 1].normalize() : Vector3R(0, 1, 0));
		triangle.setNc(normals[i + 2].lengthSquared() > 0 ? normals[i + 2].normalize() : Vector3R(0, 1, 0));
		addTriangle(triangle);
	}
}

void GTriangleMesh::genPrimitive(
	const PrimitiveBuildingMaterial& data,
	std::vector<std::unique_ptr<Primitive>>& out_primitives) const
{
	if(SIMD)
	{
		logger.log("start building qbvh...");

		Timer timer;
		timer.start();

		std::vector<PTriangle> items;
		for(const auto& gTriangle : m_gTriangles)
		{
			// FIXME: leak
			// Triangle* triangle = new Triangle();
			// triangle->setTvertices(
			// 	gTriangle.getVa().x, 
			// 	gTriangle.getVa().y,
			// 	gTriangle.getVa().z,
			// 	gTriangle.getVb().x,
			// 	gTriangle.getVb().y,
			// 	gTriangle.getVb().z, 
			// 	gTriangle.getVc().x,
			// 	gTriangle.getVc().y,
			// 	gTriangle.getVc().z);
			PTriangle p(data.metadata, Vector3R(0,0,0), Vector3R(0,0,0), Vector3R(0,0,0));
			p.setNa(gTriangle.getVa());
			p.setNb(gTriangle.getVb());
			p.setNc(gTriangle.getVc());
			p.setUVWa(gTriangle.getUVWa());
			p.setUVWb(gTriangle.getUVWb());
			p.setUVWc(gTriangle.getUVWc());
			// triangles.tris.push_back(triangle);
			items.push_back(p);
		}

		// QBVH(
		// 	int traversalCost, 
		// 	int intersectionCost,
		// 	float emptyBonus,
		// 	std::size_t maxNodeItems);

		auto qbvh = std::make_unique<QBVH<PTriangle, unsigned int>>(10, 10, 10.0f, 64);
		qbvh.get()->buildQBVH(std::move(items));
		//out_primitives.push_back(std::move(qbvh));

		timer.finish();

		logger.log("QBVH build time: " + std::to_string(timer.getDeltaMs()) + " ms");
	}
	else
	{
		
		for(const auto& gTriangle : m_gTriangles)
		{
			gTriangle.genPrimitive(data, out_primitives);
		}
	}
}

void GTriangleMesh::addTriangle(const GTriangle& gTriangle)
{
	m_gTriangles.push_back(gTriangle);
}

std::shared_ptr<Geometry> GTriangleMesh::genTransformed(
	const StaticAffineTransform& transform) const
{
	auto geometrySoup = std::make_shared<GeometrySoup>();
	for(const auto& gTriangle : m_gTriangles)
	{
		geometrySoup->add(std::make_shared<GTriangle>(gTriangle));
	}

	return geometrySoup->genTransformed(transform);
}

// command interface

SdlTypeInfo GTriangleMesh::ciTypeInfo()
{
	return SdlTypeInfo(ETypeCategory::REF_GEOMETRY, "triangle-mesh");
}

void GTriangleMesh::ciRegister(CommandRegister& cmdRegister)
{
	SdlLoader loader;
	loader.setFunc<GTriangleMesh>(ciLoad);
	cmdRegister.setLoader(loader);
}

std::unique_ptr<GTriangleMesh> GTriangleMesh::ciLoad(const InputPacket& packet)
{
	const std::vector<Vector3R> positions = packet.getVector3Array("positions");
	const std::vector<Vector3R> texCoords = packet.getVector3Array("texture-coordinates");
	const std::vector<Vector3R> normals   = packet.getVector3Array("normals");

	return std::make_unique<GTriangleMesh>(positions, texCoords, normals);
}

}// end namespace ph
