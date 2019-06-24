#pragma once
//#include "Source/Math/TVector3.h"
#include "Common/primitive_type.h"
#include "Core/Bound/TAABB3D.h"

namespace ph 
{
template<typename Index, bool USE_SINGLE_ITEM_OPT = true>
class BVHNode
{
public:
	static BVHNode makeInner(
		real                    splitPos,
		constant::AxisIndexType splitAxisIndex,
		std::size_t             positiveChildIndex,
		AABB3D 				    aabb);

	static BVHNode makeLeaf(
		Index               index,
		std::size_t         numItems,
		AABB3D				aabb);

	static BVHNode makeLeaf(
		const Index*        itemIndices,
		std::size_t         numItems,
		std::vector<Index>& indexBuffer,
		AABB3D 			aabb);

	BVHNode();
	
	AABB3D m_aabb;

	bool isLeaf() const;
	std::size_t positiveChildIndex() const;
	std::size_t numItems() const;
	real splitPos() const;
	int splitAxisIndex() const;
	std::size_t index() const;
	std::size_t singleItemDirectIndex() const;
	std::size_t indexBufferOffset() const;
	bool isEmpty() const;
private:
	constexpr static std::size_t NUM_U1_NUMBER_BITS = sizeof(Index) * CHAR_BIT - 2;
	constexpr static std::size_t MAX_U1_NUMBER      = (std::size_t(1) << (NUM_U1_NUMBER_BITS - 1)) - 1;

	/*
		For inner nodes: Splitting position <splitPos> along the axis of 
		splitting is stored.
		
		For leaf nodes:  An index value for accessing item is stored in <index>.
	*/
	

	union
	{
		real  u0_splitPos;
		Index u0_index;
	};

	/*
		Assuming Index type has N bits, we divide it into two parts: 
		[N - 2 bits][2 bits]. The [2 bits] part <flags> has the following meaning

		0b00: splitting axis is X // indicates this node is inner
		0b01: splitting axis is Y //
		0b10: splitting axis is Z //
		0b11: this node is leaf

		For inner nodes, <positiveChildIndex> is stored in the upper [N - 2 bits].
		For leaf nodes, <numItems> is stored in the upper [N - 2 bits] instead. 
	*/
	union
	{
		Index u1_flags;
		Index u1_numItems;
		Index u1_positiveChildIndex;
	};
};

// In-header Implementations:
// code copy from TIndexedKdtreeNode.h
template<typename Index, bool USE_SINGLE_ITEM_OPT>
inline BVHNode<Index, USE_SINGLE_ITEM_OPT>::
	BVHNode() = default;

template<typename Index, bool USE_SINGLE_ITEM_OPT>
inline auto BVHNode<Index, USE_SINGLE_ITEM_OPT>::
	makeInner(
		const real                    splitPos,
		const constant::AxisIndexType splitAxisIndex,
		const std::size_t             rightChildIndex,
		const AABB3D 				  aabb) -> BVHNode
{
	PH_ASSERT(
		(!std::isnan(splitPos) && !std::isinf(splitPos)) &&
		(0 <= splitAxisIndex && splitAxisIndex <= 2)     &&
		(rightChildIndex <= MAX_U1_NUMBER));

	BVHNode node;

	node.u0_splitPos = splitPos;

	const Index shiftedIndex = static_cast<Index>(rightChildIndex << 2);
	node.u1_flags = static_cast<Index>(splitAxisIndex);
	node.u1_positiveChildIndex |= shiftedIndex;
	node.m_aabb = aabb;

	return node;
}

template<typename Index, bool USE_SINGLE_ITEM_OPT>
inline auto BVHNode<Index, USE_SINGLE_ITEM_OPT>::
	makeLeaf(
		const Index       index,
		const std::size_t numItems,
		const AABB3D    aabb) -> BVHNode
{
	PH_ASSERT(numItems <= MAX_U1_NUMBER);

	BVHNode node;

	const Index shiftedNumItems = static_cast<Index>(numItems << 2);
	node.u1_flags = 0b11;
	node.u1_numItems |= shiftedNumItems;

	node.u0_index = index;

	node.m_aabb = aabb;

	return node;
}

template<typename Index, bool USE_SINGLE_ITEM_OPT>
inline auto BVHNode<Index, USE_SINGLE_ITEM_OPT>::
	makeLeaf(
		const Index* const  itemIndices,
		const std::size_t   numItems,
		std::vector<Index>& indexBuffer,
		const AABB3D       aabb) -> BVHNode
{
	PH_ASSERT(itemIndices && numItems <= MAX_U1_NUMBER);

	if(!(USE_SINGLE_ITEM_OPT && numItems == 1))
	{
		// For leaf nodes we directly store index offset value in <u0>. If Index is signed type, 
		// value conversion from negative Index back to std::size_t can mess up the stored bits. 
		// So here we make sure that we will not overflow Index.
		// OPT: try to find an efficient way to make use of the sign bit for storing index
		PH_ASSERT(indexBuffer.size() <= static_cast<std::size_t>(std::numeric_limits<Index>::max()));
		const Index indexBufferOffset = static_cast<Index>(indexBuffer.size());

		for(std::size_t i = 0; i < numItems; ++i)
		{
			indexBuffer.push_back(itemIndices[i]);
		}

		return makeLeaf(indexBufferOffset, numItems, aabb);
	}
	else
	{
		return makeLeaf(itemIndices[0], numItems, aabb);
	}
}

template<typename Index, bool USE_SINGLE_ITEM_OPT>
inline auto BVHNode<Index, USE_SINGLE_ITEM_OPT>::
	isLeaf() const -> bool
{
	return (u1_flags & 0b11) == 0b11;
}

template<typename Index, bool USE_SINGLE_ITEM_OPT>
inline auto BVHNode<Index, USE_SINGLE_ITEM_OPT>::
	isEmpty() const -> bool
{
	return (index() == -1);
}

template<typename Index, bool USE_SINGLE_ITEM_OPT>
inline auto BVHNode<Index, USE_SINGLE_ITEM_OPT>::
	positiveChildIndex() const -> std::size_t
{
	PH_ASSERT(!isLeaf());

	return static_cast<std::size_t>(u1_positiveChildIndex >> 2);
}

template<typename Index, bool USE_SINGLE_ITEM_OPT>
inline auto BVHNode<Index, USE_SINGLE_ITEM_OPT>::
	numItems() const -> std::size_t
{
	PH_ASSERT(isLeaf());

	return static_cast<std::size_t>(u1_numItems >> 2);
}

template<typename Index, bool USE_SINGLE_ITEM_OPT>
inline auto BVHNode<Index, USE_SINGLE_ITEM_OPT>::
	splitPos() const -> real
{
	PH_ASSERT(!isLeaf());

	return u0_splitPos;
}

template<typename Index, bool USE_SINGLE_ITEM_OPT>
inline auto BVHNode<Index, USE_SINGLE_ITEM_OPT>::
	splitAxisIndex() const -> int
{
	PH_ASSERT(!isLeaf());

	return static_cast<int>(u1_flags & 0b11);
}

template<typename Index, bool USE_SINGLE_ITEM_OPT>
inline auto BVHNode<Index, USE_SINGLE_ITEM_OPT>::
	index() const -> std::size_t
{
	PH_ASSERT(isLeaf());

	return static_cast<std::size_t>(u0_index);
}

template<typename Index, bool USE_SINGLE_ITEM_OPT>
inline auto BVHNode<Index, USE_SINGLE_ITEM_OPT>::
	singleItemDirectIndex() const -> std::size_t
{
	if constexpr(USE_SINGLE_ITEM_OPT)
	{
		PH_ASSERT(USE_SINGLE_ITEM_OPT && numItems() == 1);

		return index();
	}
	else
	{
		PH_ASSERT_UNREACHABLE_SECTION();
		return std::size_t(-1);
	}
}

template<typename Index, bool USE_SINGLE_ITEM_OPT>
inline auto BVHNode<Index, USE_SINGLE_ITEM_OPT>::
	indexBufferOffset() const -> std::size_t
{
	PH_ASSERT(!(USE_SINGLE_ITEM_OPT && numItems() == 1));

	return index();
}    
    
}
