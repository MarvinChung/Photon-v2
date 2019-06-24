#include <gtest/gtest.h>
//#include "simdpp/simd.h"
#include "Common/primitive_type.h"
#include <Core/Intersectable/IntelSimdBvh/BVH.h>
//#include "Core/Intersectable/IntelSimdBvh/BVHNode.h"   
#include <Math/math.h>
#include "Core/Intersectable/PTriangle.h"

template<typename Item, typename Index>
void testBVH()
{
	using namespace ph;
	const int traversalCost = 10;
	const int intersectionCost = 10;
	const int emptyBonus = 10;
	const int maxNodeItems = 10;
	//std::unique_ptr<BVH<Item, Index>> bvh = std::make_unique<BVH<Item, Index>>(traversalCost, intersectionCost, emptyBonus, maxNodeItems);
	std::vector<Item> items;
	//items.push_back();
	//bvh.get();
	BVH<Item, Index> bvh = BVH<Item, Index>(traversalCost, intersectionCost, emptyBonus, maxNodeItems);
    bvh.build(items);
}
TEST(SIMD_BVH_Test, SIMD_BVH_is_Working)
{
	
	testBVH<ph::PTriangle, unsigned int>();
}
