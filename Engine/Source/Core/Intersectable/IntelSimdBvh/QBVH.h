#pragma once
#include "Core/Intersectable/IntelSimdBvh/BVH.h"
#include "Core/Intersectable/IntelSimdBvh/QBVHNode.h"

#include <iostream>
#include <memory>
#include <queue>
#include <cmath>
//#include "simdpp/simd.h"

namespace ph
{

constexpr int LEFTTREENODE = 1;
constexpr int RIGHTTREENODE = 2;
constexpr int ROOTTREENODE = 0;

template<typename Item, typename Index> 
class TreeNode{ 
private:
    std::size_t m_traverseBVHIndex;
public:
    TreeNode();

    size_t m_arrayIndex;
    BVHNode<Index> m_bvhNode;
    TreeNode<Item, Index> *m_left, *m_right, *m_par; 
    void buildTree(const std::unique_ptr<BVH<Item, Index>> bvh, TreeNode<Item, Index> *par);
}; 

template<typename Item, typename Index> 
inline TreeNode<Item, Index>::TreeNode() : m_traverseBVHIndex(0)
{} 

template<typename Item, typename Index> 
class BVH2QBVHInfo
{
public:
    std::vector<TreeNode<Item, Index>> collapseBVHs;
    std::vector< std::array<int, SIMD_VECTOR_WIDTH - 1> >splitAxises;
};
    
template<typename Item, typename Index> 
class QBVH
{

public:

	QBVH(
		int traversalCost, 
		int intersectionCost,
		float emptyBonus,
		std::size_t maxNodeItems);

	//void build(std::vector<Item>&& items);
	//bool isIntersecting(const Ray& ray, HitProbe& probe) const;
	//void getAABB(AABB3D* out_aabb) const;
    std::unique_ptr<BVH<Item, Index>> buildBVH(    
        std::vector<Item>& items
    );

    void buildQBVH(
        std::vector<Item>& items
    );



    void collapseBVHTreeRecursive(
        const TreeNode<Item, Index> *root,
        const size_t k_level,
        const int depth,
        const int TreeNodeCategory
    );   

    void upTraverseNode(
        const TreeNode<Item, Index> *root, 
        const size_t digLevel, 
        std::vector<TreeNode<Item, Index>>& out_collapseBVH,
        std::array<int, SIMD_VECTOR_WIDTH - 1>& out_splitAxis,
        int TreeNodeCategory
    );


    bool vectorIsAllLeaf(const int vectorIndex);

    void buildQBVHNodeRecursive(const std::size_t nodeIndex,const int currentNodeDepth);


private:

	int m_traversalCost;
	int m_intersectionCost;
	float m_emptyBonus;
	std::size_t m_maxNodeItems;
	std::size_t m_maxNodeDepth;
	AABB3D m_rootAABB;
	std::vector<QBVHNode<Index>> m_nodeBuffer;
	std::size_t m_numNodes;
	std::vector<Index> m_itemIndices;

    std::size_t m_NumBVHNodes;
    int *m_AddedNodes;//init at buildBVH
    //std::vector< std::vector<TreeNode<Index>> > m_collapseBVH;
    std::vector< BVH2QBVHInfo<Item, Index> > m_collapseBVHInfo;
};

template<typename Item, typename Index>
inline QBVH<Item, Index>::QBVH(
    const int traversalCost,
	const int intersectionCost,
	const float emptyBonus,
	const std::size_t maxNodeItems
    ) :
    m_traversalCost(traversalCost),
	m_intersectionCost(intersectionCost),
	m_emptyBonus(emptyBonus),
	m_maxNodeItems(maxNodeItems),
	m_maxNodeDepth(0),
    m_rootAABB(),
	m_nodeBuffer(),
	m_numNodes(0),
	m_itemIndices(),
    m_NumBVHNodes(0)
    //m_collapseBVH()
{}

template<typename Item, typename Index>
inline std::unique_ptr<BVH<Item, Index>> QBVH<Item, Index>::buildBVH(	
    std::vector<Item>& items
)
{
    std::unique_ptr<BVH<Item, Index>> bvh = std::make_unique<BVH<Item, Index>>(m_traversalCost, m_intersectionCost, m_emptyBonus, int(m_maxNodeItems/4));
    bvh.get()->build(items);
    return bvh;
}
    
template<typename Item, typename Index>
inline void QBVH<Item, Index>::buildQBVH(
    std::vector<Item>& items
)
{
    std::unique_ptr<BVH<Item, Index>> bvh = buildBVH(items);

    m_NumBVHNodes = bvh.get()->m_nodeBuffer.size();
    m_AddedNodes = new int[m_NumBVHNodes];
    m_collapseBVHInfo.reserve(int(float(m_NumBVHNodes)/SIMD_VECTOR_WIDTH));
    
    
    //size_t k_level = floor(log2(depth));
    //fixed k_level = 2
    const size_t k_level = 2;
    
    TreeNode<Item, Index> root;
    //root.buildTree(bvh, &root);
    
    //collapseBVHTreeRecursive(&root, k_level, 0, ROOTTREENODE);
    
    //buildQVHNodeRecursive();
    
}    

template<typename Item, typename Index>
inline void TreeNode<Item, Index>::buildTree(const std::unique_ptr<BVH<Item, Index>> bvh, TreeNode<Item, Index> *par)
{
    m_traverseBVHIndex++;
    
    if(!bvh.get()->m_nodeBuffer[m_traverseBVHIndex].isLeaf())
    {   
        this->bvhNode = bvh.get()->m_nodeBuffer[m_traverseBVHIndex];
        
        this->m_arrayIndex = m_traverseBVHIndex;
        this->par = par;
        
        this->left = new TreeNode<Item, Index>();        
        this->left->buildTree(bvh, this);
        
        this->right = new TreeNode<Item, Index>();
        this->right->buildTree(bvh, this);
    }
    else
    {
        this->bvhNode = bvh.get()->m_nodeBuffer[m_traverseBVHIndex];
        this->par = par;
        
        this->left = NULL;        
        this->right = NULL;
    }
}

    
    
//This code only words at k_level = 2    
template<typename Item, typename Index>
inline void QBVH<Item, Index>::collapseBVHTreeRecursive(
    const TreeNode<Item, Index> *root,
    const size_t k_level,
    const int depth,
    const int TreeNodeCategory
)    
{
    //build QBVH from collapsing the BVH
    
    if(root == NULL)
        return;
    else if(depth % k_level == 0 && TreeNodeCategory == LEFTTREENODE && m_AddedNodes[root->m_arrayIndex] == 0)
    {       
        //since collapseBVHTreeRecursive will go left and right, therefore make sure the candidate node should not already be added
        TreeNode<Item, Index> *temp = root;

        //to collapse the save level nodes (its cousins), it should travese its ancestors SIMD_VECTOR_WIDTH/2 times.
        const int upTraverseTimes = SIMD_VECTOR_WIDTH/2;
        
        for(int i = 0; i < upTraverseTimes; i++)
        {
            temp = temp->par;
        }

        std::vector<TreeNode<Item, Index>> collapseBVH;
        std::array<int, SIMD_VECTOR_WIDTH - 1> splitAxis;
       
        splitAxis[ROOTTREENODE] = temp->m_bvhNode.splitAxisIndex();
        QBVH<Item, Index>::upTraverseNode(temp , upTraverseTimes, collapseBVH, splitAxis);  
        
        m_collapseBVHInfo[int(depth/k_level)].collapseBVHs.push_back(collapseBVH);
        m_collapseBVHInfo[int(depth/k_level)].splitAxises.push_back(splitAxis);
        

    }
    
    collapseBVHTreeRecursive(root->left, k_level, 0, LEFTTREENODE);            
    collapseBVHTreeRecursive(root->right, k_level, 0, RIGHTTREENODE);

 
    
}
    
template<typename Item, typename Index>
inline void QBVH<Item, Index>::upTraverseNode(
    const TreeNode<Item, Index> *root, 
    const size_t digLevel, 
    std::vector<TreeNode<Item, Index>>& out_collapseBVH,
    std::array<int, SIMD_VECTOR_WIDTH - 1>& out_splitAxis,
    int TreeNodeCategory
)
{
    if(digLevel != 0)
    {
        if(root->left != NULL)
        {
            upTraverseNode(root->left, digLevel - 1, out_collapseBVH, out_splitAxis, LEFTTREENODE);
        }
            
        if(root->right != NULL)
        {
            upTraverseNode(root->right, digLevel - 1, out_collapseBVH, out_splitAxis, RIGHTTREENODE);
        }
        
    }
    else
    {
        //at the bottom of collapseBVH, their will be left split axis and right split axis. Find it and add to 
        //out_splitAxis, it will be used at QBVHNode.
        if(TreeNodeCategory == LEFTTREENODE)
        {
            out_splitAxis[LEFTTREENODE] = root->m_bvhNode.splitAxisIndex();
        }
        else if(TreeNodeCategory == RIGHTTREENODE)
        {
            out_splitAxis[RIGHTTREENODE] = root->m_bvhNode.splitAxisIndex();
        }
        m_AddedNodes[root->m_arrayIndex] = 1;
        out_collapseBVH.push_back(root->m_bvhNode);
    }        
}

template<typename Item, typename Index>
inline bool QBVH<Item, Index>::vectorIsAllLeaf(const int vectorIndex)
{
    for(int i = 0; i < m_collapseBVHInfo[vectorIndex].size(); i++)
    {
        if(!m_collapseBVHInfo[vectorIndex].m_collapseBVH[i].isLeaf())
        {
            return false;
        }
    }
    return true;
}
    
template<typename Item, typename Index>    
void QBVH<Item, Index>::buildQBVHNodeRecursive(const std::size_t nodeIndex,const int currentNodeDepth)
{
    if(currentNodeDepth > m_maxNodeDepth || currentNodeDepth >= m_collapseBVHInfo.size() || vectorIsAllLeaf(currentNodeDepth))
        return;
    
    
    
    ++m_numNodes;
    
	if(m_numNodes > m_nodeBuffer.size())
	{
		m_nodeBuffer.resize(m_numNodes * 2);
	}
    
    int begin = 0;
    
    //Inside m_collapseBVH[currenNodeDepth] are all QBVHnodes that have the save k_level
    for(int i = 0; i < m_collapseBVHInfo[currentNodeDepth].size(); i++)
    {
        //Make a QBVH node every simd_vector_width(4) 
        if((i+1) % SIMD_VECTOR_WIDTH == 0)
        {


            //begin is the start node every simd_vector_width
            //eg:
            //begin = 0, i = 3
            //next time, begin = i+1 = 4, and will start build node at i = 7
            
            m_nodeBuffer[nodeIndex] = QBVHNode<Index>::makeNode(m_collapseBVHInfo[currentNodeDepth], begin, i);    
                
            for(int j = begin; j < i; j++)
            {

                //4 children recursive build
                for(int k = 0; k < SIMD_VECTOR_WIDTH ; k++)
                {                    
                    buildQBVHNodeRecursive(m_numNodes, currentNodeDepth + 1);
                }
                    
            }
            
            
            
            begin = i+1;
        }
            
    }
    
    
}
    
    
}