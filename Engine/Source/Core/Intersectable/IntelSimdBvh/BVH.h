#pragma once

#include "Core/Bound/TAABB3D.h"
#include "Core/Bound/TAABB2D.h"
#include "Math/TVector2.h"
#include "Utility/utility.h"
#include "Core/Ray.h"
#include "Core/HitProbe.h"
#include "Common/assertion.h"
#include "Math/constant.h"
#include "Core/Intersectable/IntelSimdBvh/BVHNode.h"
#include "Core/Intersectable/IntelSimdBvh/SimdWidth.h"
#include "Math/constant.h"
// using AxisIndexType = int;
// inline constexpr AxisIndexType X_AXIS       = 0;
// inline constexpr AxisIndexType Y_AXIS       = 1;
// inline constexpr AxisIndexType Z_AXIS       = 2;

#include <vector>
#include <utility>
#include <memory>
#include <array>
#include <cmath>
#include <limits>
#include <algorithm>


namespace ph{
    
template<typename Item, typename Index> 
class BVH
{

public:
	using Node = BVHNode<Index>;

	BVH(
		int traversalCost, 
		int intersectionCost,
		float emptyBonus,
		std::size_t maxNodeItems);

	void build(std::vector<Item>& items);
	bool isIntersecting(const Ray& ray, HitProbe& probe) const;
	void getAABB(AABB3D* out_aabb) const;
	std::vector<Node> m_nodeBuffer;//store node in tree-like structue, but make search-first memory contiguous. Therefore depth-first
private:

	std::vector<Item> m_items;//items in the view
	int m_traversalCost;
	int m_intersectionCost;
	float m_emptyBonus;
	std::size_t m_maxNodeItems;
	std::size_t m_maxNodeDepth;
    std::size_t m_NodeDepth;
	AABB3D m_rootAABB;	
	std::size_t m_numNodes;
	std::vector<Index> m_itemIndices;
	void buildNodeRecursive(
		std::size_t nodeIndex,
		const AABB3D& nodeAABB,
		const Index* nodeItemIndices,
		std::size_t numNodeItems,
		std::size_t currentNodeDepth,
		std::size_t currentBadRefines,
		const std::vector<AABB3D>& itemAABBs,
		Index* const negativeItemIndicesCache,
		Index* const positiveItemIndicesCache);


};	

template<class ForwardIterator>
inline size_t argmax(ForwardIterator first, ForwardIterator last)
{
    return std::distance(first, std::max_element(first, last));
}

template<typename Item, typename Index>
inline BVH<Item, Index>::BVH(

	const int traversalCost,
	const int intersectionCost,
	const float emptyBonus,
	const std::size_t maxNodeItems) :
	
	m_items(),
	m_traversalCost(traversalCost),
	m_intersectionCost(intersectionCost),
	m_emptyBonus(emptyBonus),
	m_maxNodeItems(maxNodeItems),
	m_maxNodeDepth(0),
    m_NodeDepth(0),
	m_rootAABB(),
	m_nodeBuffer(),
	m_numNodes(0),
	m_itemIndices()
{    
}

template<typename Item, typename Index>
inline void BVH<Item, Index>::build(std::vector<Item>& items)
{
	m_items = std::move(items);
	if(m_items.empty())
	{
		return;
	}

	m_maxNodeDepth = static_cast<std::size_t>(8 + 1.3 * std::log2(m_items.size()) + 0.5);

	std::vector<AABB3D> itemAABBs;
	regular_access(m_items.front()).calcAABB(&m_rootAABB);
	for(const auto& item : m_items)
	{
		AABB3D aabb;
		regular_access(item).calcAABB(&aabb);

		itemAABBs.push_back(aabb);
		m_rootAABB.unionWith(aabb);
	}

	std::unique_ptr<Index[]> negativeItemIndicesCache(new Index[m_items.size()]);
	std::unique_ptr<Index[]> positiveItemIndicesCache(new Index[m_items.size() * m_maxNodeDepth]);

	PH_ASSERT(m_items.size() - 1 <= std::numeric_limits<Index>::max());
	for(std::size_t i = 0; i < m_items.size(); ++i)
	{
		negativeItemIndicesCache[i] = static_cast<Index>(i);
	}

    
    buildNodeRecursive(
    0, 
    m_rootAABB, 
    negativeItemIndicesCache.get(),
    m_items.size(),
    0,
    0,
    itemAABBs,
    negativeItemIndicesCache.get(),
	positiveItemIndicesCache.get()
    );
}

template<typename Item, typename Index>
inline void BVH<Item, Index>::buildNodeRecursive(
	const std::size_t nodeIndex,
	const AABB3D& nodeAABB,
	const Index* const nodeItemIndices,
	const std::size_t numNodeItems,
	const std::size_t currentNodeDepth,
	const std::size_t currentBadRefines,
	const std::vector<AABB3D>& itemAABBs,
	Index* const negativeItemIndicesCache,
	Index* const positiveItemIndicesCache
)
{
	++m_numNodes;
   
    if(currentNodeDepth > m_NodeDepth)
        m_NodeDepth = currentNodeDepth;
    
	if(m_numNodes > m_nodeBuffer.size())
	{
		m_nodeBuffer.resize(m_numNodes * 2);
	}
	PH_ASSERT(nodeIndex < m_nodeBuffer.size());

	if(currentNodeDepth == m_maxNodeDepth || numNodeItems <= m_maxNodeItems)
	{
		m_nodeBuffer[nodeIndex] = Node::makeLeaf(nodeItemIndices, numNodeItems, m_itemIndices, nodeAABB);
		return;
	}

	const real     noSplitCost         = m_intersectionCost * static_cast<real>(numNodeItems);
	const real     reciNodeSurfaceArea = 1.0_r / nodeAABB.getSurfaceArea();
	const Vector3R nodeExtents         = nodeAABB.getExtents();

	real        bestSplitCost     = std::numeric_limits<real>::max();
	int         bestAxis          = -1;
	std::size_t bestEndpointIndex = std::numeric_limits<std::size_t>::max();
	//int         axis              = nodeExtents.maxDimension();
	//int         numSplitTrials    = 0;
	// while(bestAxis == -1 && numSplitTrials < 3)
	// {
	// 	//do bin

	// 	++numSplitTrials;
	// 	axis = (axis + 1) % 3;

	// }

	// do bin
	if(nodeExtents.x > nodeExtents.y && nodeExtents.x > nodeExtents.z)
		bestAxis = constant::X_AXIS;
	else if(nodeExtents.y > nodeExtents.x && nodeExtents.y > nodeExtents.z)
		bestAxis = constant::Y_AXIS;
	else
		bestAxis = constant::Z_AXIS;

    int minCostSplitBucket = 0;
    float leftBoundAxis = nodeAABB.getMinVertex()[bestAxis];
    float LongestExtent = nodeExtents[bestAxis];
    const int nBuckets = 12;
    
    struct BucketInfo
    {
        int count = 0;
        AABB3D bounds;
    };
    struct BucketInfo buckets[nBuckets];
    float cost[nBuckets];
    //initialize bucket bounds, union bound with primitives
    for (int i = 0; i < numNodeItems; ++i) {
        const Index  itemIndex = nodeItemIndices[i];

        int b = nBuckets * (itemAABBs[itemIndex].getCentroid()[bestAxis] - nodeAABB.getMinVertex()[bestAxis])/LongestExtent; 
        
        buckets[b].count++;
        buckets[b].bounds = buckets[b].bounds.unionWith(itemAABBs[itemIndex]);
    }


    for (int i = 0; i < nBuckets - 1; ++i) {
        AABB3D b0, b1;
        int count0 = 0, count1 = 0;
        //left buckets
        for (int j = 0; j <= i; ++j) {
            b0 = b0.unionWith(buckets[j].bounds);
            count0 += buckets[j].count;
        }
        
        //right buckets
        for (int j = i+1; j < nBuckets; ++j) {
            b1 = b1.unionWith(buckets[j].bounds);
            count1 += buckets[j].count;
        }
        cost[i] = .125f + (count0 * b0.getSurfaceArea() +
                          count1 * b1.getSurfaceArea()) / b0.unionWith(b1).getSurfaceArea();
    }

    for (int i = 0; i < nBuckets - 1; ++i) {
        if (cost[i] < bestSplitCost)
        {
           bestSplitCost = cost[i];
           minCostSplitBucket = i;
        }
    }
	
    
	std::size_t newNumBadRefines = currentBadRefines;
	if(bestSplitCost > noSplitCost)
	{
		++newNumBadRefines;
	}

	if((bestSplitCost > 4 * noSplitCost && numNodeItems < 16) || 
	   bestAxis == -1 ||
	   newNumBadRefines == 3)
	{
		m_nodeBuffer[nodeIndex] = Node::makeLeaf(nodeItemIndices, numNodeItems, m_itemIndices, nodeAABB);
		return;
	}   
	

	int numNegativeItems = 0;
	int numPositiveItems = 0;

	for (int i = 0; i < numNodeItems; ++i) {
        const Index  itemIndex = nodeItemIndices[i];

        int b = nBuckets * (itemAABBs[itemIndex].getCentroid()[bestAxis] - nodeAABB.getMinVertex()[bestAxis])/LongestExtent; 
        if(b <= minCostSplitBucket)
        	negativeItemIndicesCache[numNegativeItems++] = itemIndex;
        else
        	positiveItemIndicesCache[numPositiveItems++] = itemIndex;
    }


    buildNodeRecursive(
    nodeIndex + 1, 
    buckets[minCostSplitBucket].bounds,
    negativeItemIndicesCache,
    numNegativeItems,
    currentNodeDepth + 1,
    newNumBadRefines,
    itemAABBs,
    negativeItemIndicesCache,
    positiveItemIndicesCache + numNodeItems
    );
    
    const std::size_t positiveChildIndex = m_numNodes;
    m_nodeBuffer[nodeIndex] = Node::makeInner(minCostSplitBucket, bestAxis, positiveChildIndex, nodeAABB);
        
    
    buildNodeRecursive(
    positiveChildIndex,
    buckets[minCostSplitBucket + 1].bounds,
    positiveItemIndicesCache,
    numPositiveItems,
    currentNodeDepth + 1,
    newNumBadRefines,
    itemAABBs,
    negativeItemIndicesCache,
    positiveItemIndicesCache + numNodeItems
    );
    }    

}