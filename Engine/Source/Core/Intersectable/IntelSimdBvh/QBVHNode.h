#pragma once
#include "Core/Bound/TAABB3D.h"
#include "Math/TVector3.h"
#include "Core/Intersectable/IntelSimdBvh/BVHNode.h"
#include "Core/Intersectable/IntelSimdBvh/SimdWidth.h"
#include "Common/primitive_type.h"

#include <iostream>
#include "Math/constant.h"
// inline constexpr AxisIndexType X_AXIS       = 0;
// inline constexpr AxisIndexType Y_AXIS       = 1;
// inline constexpr AxisIndexType Z_AXIS       = 2;
namespace ph
{
template<typename Index>   
class QBVHNode
{
    public:
        // a width wide boxes with min-max[2] , xyz[3]
        __m128 m_bboxes[2][3];
        // 4 children
        int m_children[SIMD_VECTOR_WIDTH];
        //top axis, left axis , right axis
        int m_axis[3];
        int m_fill;
        bool isIntersecting(const __m128 bboxes[2][3],
                            const __m128 ray_origin[3],
                            const __m128 idir[3],
                            const int sign[3],
                            __m128 tmin,
                            __m128 tmax);
        QBVHNode makeNode(std::vector< std::vector<BVHNode<Index>> >& currentCollapseBVHInfo, const int begin, const int end);

        
};
 
// bool KDNode::isIntersecting(const Ray& ray, HitProbe& probe) const {

// }  


// void KDNode::calcIntersectionDetail(const Ray& ray, HitProbe& probe, HitDetail* out_detail) const {
// }

// bool KDNode::isIntersectingVolumeConservative(const AABB3D& volume) const {
//     return true;
// }
  
template<typename Index>
inline bool QBVHNode<Index>::isIntersecting(
    const __m128 bboxes[2][3],
    const __m128 ray_origin[3],
    const __m128 idir[3],
    const int sign[3],
    __m128 tmin,
    __m128 tmax

)
{   
    //TODO
    //read ifconstexpr

    //x coordinate
    tmin = _mm_max_ps( tmin, _mm_mul_ps(_mm_sub_ps( bboxes[ sign[0] ][0], ray_origin[0]) , idir[0]) );
    tmax = _mm_min_ps( tmax, _mm_mul_ps(_mm_sub_ps( bboxes[ 1 - sign[0] ][0], ray_origin[0]) , idir[0]) );

    //y cooridinate
    tmin = _mm_max_ps( tmin, _mm_mul_ps(_mm_sub_ps( bboxes[ sign[1] ][1], ray_origin[1]) , idir[1]) );
    tmax = _mm_min_ps( tmax, _mm_mul_ps(_mm_sub_ps( bboxes[ 1 - sign[1] ][1], ray_origin[1]) , idir[1]) );

    //z coordinate
    tmin = _mm_max_ps( tmin, _mm_mul_ps(_mm_sub_ps( bboxes[ sign[2] ][2], ray_origin[2]) , idir[2]) );
    tmax = _mm_min_ps( tmax, _mm_mul_ps(_mm_sub_ps( bboxes[ 1 - sign[2] ][2], ray_origin[2]) , idir[2]) );

    return _mm_movemask_ps(_mm_cmpge_ps(tmax, tmin));
}  



template<typename Index>
inline QBVHNode<Index> QBVHNode<Index>::makeNode(std::vector< std::vector<BVHNode<Index>> >& currentCollapseBVHInfo, const int begin, const int end)
{
    QBVHNode node;
    //TAABB3D aabbs[SIMD_VECTOR_WIDTH];
    Vector3R AABBmin[SIMD_VECTOR_WIDTH];
    Vector3R AABBmax[SIMD_VECTOR_WIDTH];
    
    for(int ct = 0, i = begin; i < end; ct++, i++)
    {
        if(currentCollapseBVHInfo.collapseBVH[i].m_bvhNode.isLeaf()){
            node.m_children[ct] = -1;
            //aabbs[ct] = currentCollapseBVHInfo.collapseBVH[i].m_bvhNode.m_aabb;
            AABBmin[ct] = currentCollapseBVHInfo.collapseBVH[i].m_bvhNode.m_aabb.getMinVertex();
            AABBmax[ct] = currentCollapseBVHInfo.collapseBVH[i].m_bvhNode.m_aabb.getMaxVertex();
        }
        else{
            node.m_children[ct] = currentCollapseBVHInfo.collapseBVH[i].m_arrayIndex;
        }
    }
    
    //min x
    node.m_bboxes[0][0] = _mm_set_ps(AABBmin[0].x, AABBmin[1].x, AABBmin[2].x, AABBmin[3].x);

    //min y
    node.m_bboxes[0][1] = _mm_set_ps(AABBmin[0].y, AABBmin[1].y, AABBmin[2].y, AABBmin[3].y);

    //min z
    node.m_bboxes[0][2] = _mm_set_ps(AABBmin[0].z, AABBmin[1].z, AABBmin[2].z, AABBmin[3].z);

    //max x
    node.m_bboxes[1][0] = _mm_set_ps(AABBmax[0].x, AABBmax[1].x, AABBmax[2].x, AABBmax[3].x);

    //max y
    node.m_bboxes[1][1] = _mm_set_ps(AABBmax[0].y, AABBmax[1].y, AABBmax[2].y, AABBmax[3].y);

    //max z
    node.m_bboxes[1][2] = _mm_set_ps(AABBmax[0].z, AABBmax[1].z, AABBmax[2].z, AABBmax[3].z);

    for(int i = 0; i < 3; i++)
        node.m_axis[i] = currentCollapseBVHInfo.splitAxises[i];
    
    
    return node;

}
    
}