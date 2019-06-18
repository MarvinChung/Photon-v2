#pragma once
#include "Core/Intersectable/IntelSimdBvh/SimdWidth.h"

// #include "Core/Intersectable/PTriangle.h"


//base on https://stackoverflow.com/questions/45599766/fast-sse-ray-4-triangle-intersection

namespace ph
{



class testTriangle {
    public:
        Vector3R m_vertex[3];
        testTriangle()
        {

        }
        testTriangle(const Vector3R& a,const Vector3R& b, const Vector3R& c)
        {
            m_vertex[0] = a;
            m_vertex[1] = b;
            m_vertex[2] = c;
        }

        void setVertex(const Vector3R& a,const Vector3R& b, const Vector3R& c)
        {
            m_vertex[0] = a;
            m_vertex[1] = b;
            m_vertex[2] = c;
        }
};

inline bool RayIntersectsTriangle(Vector3R rayOrigin, 
                           Vector3R rayVector, 
                           testTriangle& inTriangle,
                           Vector3R& outIntersectionPoint)
{
    const float EPSILON = 0.0000001;
    Vector3R vertex0 = inTriangle.m_vertex[0];
    Vector3R vertex1 = inTriangle.m_vertex[1];  
    Vector3R vertex2 = inTriangle.m_vertex[2];
    Vector3R edge1, edge2, h, s, q;
    float a,f,u,v;
    edge1 = vertex1 - vertex0;
    edge2 = vertex2 - vertex0;
    h = rayVector.cross(edge2);
    a = edge1.dot(h);
    if (a > -EPSILON && a < EPSILON)
        return false;    // This ray is parallel to this triangle.
    f = 1.0/a;
    s = rayOrigin - vertex0;
    u = f * (s.dot(h));
    if (u < 0.0 || u > 1.0)
        return false;
    q = s.cross(edge1);
    v = f * rayVector.dot(q);
    if (v < 0.0 || u + v > 1.0)
        return false;
    // At this stage we can compute t to find out where the intersection point is on the line.
    float t = f * edge2.dot(q);
    if (t > EPSILON) // ray intersection
    {
        outIntersectionPoint = rayOrigin + rayVector * t;
        return true;
    }
    else // This means that there is a line intersection but not a ray intersection.
        return false;
}

class PackedTriangle {
    
    public:
        //e2 = v2-v0
        //e1 = v1-v0
        std::array< __m128 , 3>  e1;
        std::array< __m128 , 3>  e2;
        std::array< __m128 , 3>  v0;
        __m128 inactiveMask; // Required. We cant always have 8 triangles per packet.
        PackedTriangle(){};
        //this only works when SIMD_VECTOR_WIDTH = 4, and tris.size = 4;
        void setVertex(const testTriangle* tris) 
        {
            this->e1[0] = _mm_set_ps(tris[0].m_vertex[1].x-tris[0].m_vertex[0].x, tris[1].m_vertex[1].x-tris[1].m_vertex[0].x, tris[2].m_vertex[1].x-tris[2].m_vertex[0].x, tris[3].m_vertex[1].x-tris[3].m_vertex[0].x);
            this->e1[1] = _mm_set_ps(tris[0].m_vertex[1].y-tris[0].m_vertex[0].y, tris[1].m_vertex[1].y-tris[1].m_vertex[0].y, tris[2].m_vertex[1].y-tris[2].m_vertex[0].y, tris[3].m_vertex[1].y-tris[3].m_vertex[0].y);
            this->e1[2] = _mm_set_ps(tris[0].m_vertex[1].z-tris[0].m_vertex[0].z, tris[1].m_vertex[1].z-tris[1].m_vertex[0].z, tris[2].m_vertex[1].z-tris[2].m_vertex[0].z, tris[3].m_vertex[1].z-tris[3].m_vertex[0].z);
            this->e2[1] = _mm_set_ps(tris[0].m_vertex[2].y-tris[0].m_vertex[0].y, tris[1].m_vertex[2].y-tris[1].m_vertex[0].y, tris[2].m_vertex[2].y-tris[2].m_vertex[0].y, tris[3].m_vertex[2].y-tris[3].m_vertex[0].y);
            this->e2[2] = _mm_set_ps(tris[0].m_vertex[2].z-tris[0].m_vertex[0].z, tris[1].m_vertex[2].z-tris[1].m_vertex[0].z, tris[2].m_vertex[2].z-tris[2].m_vertex[0].z, tris[3].m_vertex[2].z-tris[3].m_vertex[0].z);

            this->v0[0] = _mm_set_ps(tris[0].m_vertex[0].x, tris[1].m_vertex[0].x, tris[2].m_vertex[0].x, tris[3].m_vertex[0].x);
            this->v0[1] = _mm_set_ps(tris[0].m_vertex[0].y, tris[1].m_vertex[0].y, tris[2].m_vertex[0].y, tris[3].m_vertex[0].y);
            this->v0[2] = _mm_set_ps(tris[0].m_vertex[0].z, tris[1].m_vertex[0].z, tris[2].m_vertex[0].z, tris[3].m_vertex[0].z);
        }
        
};


class PackedIntersectionResult
{
    public:
        float t = std::numeric_limits<float>::infinity();;
        int idx;

};


inline std::ostream& operator<<(std::ostream& os,const PackedIntersectionResult results)
{
    os << "t:" << results.t << " " << "idx:" << results.idx << "\n";
    return os;
}

class testRay 
{
    public:
        std::array< __m128 , 3> m_origin;
        std::array< __m128 , 3> m_direction;
        __m128 m_length;
        bool isIntersectPackedTriangle(const PackedTriangle& triangle, PackedIntersectionResult& result);
        testRay(const Ray& r);  
        
};


//fm are FMA instructions, others are SSE instructions

template <typename T>
void avx_multi_cross(std::array<T, 3>& result, const std::array<T, 3>& a, const std::array<T, 3>&  b)
{
    result[0] = _mm_fmsub_ps(a[1], b[2], _mm_mul_ps(b[1], a[2]));
    result[1] = _mm_fmsub_ps(a[2], b[0], _mm_mul_ps(b[2], a[0]));
    result[2] = _mm_fmsub_ps(a[0], b[1], _mm_mul_ps(b[0], a[1]));
}

template <typename T>
T avx_multi_dot(const std::array<T, 3>&  a, const std::array<T, 3>&  b)
{
    return _mm_fmadd_ps(a[2], b[2], _mm_fmadd_ps(a[1], b[1], _mm_mul_ps(a[0], b[0])));
}

template <typename T>
void avx_multi_sub(std::array<T, 3>&  result, const std::array<T, 3>&  a, const std::array<T, 3>&  b)
{
    result[0] = _mm_sub_ps(a[0], b[0]);
    result[1] = _mm_sub_ps(a[1], b[1]);
    result[2] = _mm_sub_ps(a[2], b[2]);
}



const __m128 oneM128 = _mm_set1_ps(1.0f);
const __m128 minusOneM128 = _mm_set1_ps(-1.0f);
const __m128 positiveEpsilonM128 =  _mm_set1_ps(1e-6f);;
const __m128 negativeEpsilonM128 = _mm_set1_ps(-1e-6f);;
const __m128 zeroM128 = _mm_set1_ps(0.0f);;

bool testRay::isIntersectPackedTriangle(const PackedTriangle& packedTris, PackedIntersectionResult& result)
{
    //must sort the triangles first
    std::array< __m128 , 3> ray_cross_e2;
    avx_multi_cross(ray_cross_e2, m_direction, packedTris.e2);

    __m128 a = avx_multi_dot(packedTris.e1, ray_cross_e2);

    
    __m128 f = _mm_div_ps(oneM128 , a);

    std::array< __m128 , 3>  s;
    avx_multi_sub(s, m_origin, packedTris.v0);

    __m128 u = _mm_mul_ps(f, avx_multi_dot(s, ray_cross_e2));

    std::array< __m128 , 3>  q;
    avx_multi_cross(q, s, packedTris.e1);

    __m128 v = _mm_mul_ps(f, avx_multi_dot(m_direction, q));

    __m128 t = _mm_mul_ps(f, avx_multi_dot(packedTris.e2, q));

    // Failure conditions
    __m128 failed = _mm_and_ps(
        _mm_cmpgt_ps(a, negativeEpsilonM128) ,
        _mm_cmplt_ps(a, positiveEpsilonM128)
    );
    

    failed = _mm_or_ps(failed, _mm_cmplt_ps(u, zeroM128));
    //this one is trivial
    //failed = simdpp::bit_or(failed, simdpp::cmp_gt(u, oneM128));
    failed = _mm_or_ps(failed, _mm_cmplt_ps(v, zeroM128));
    failed = _mm_or_ps(failed, _mm_cmpgt_ps(_mm_add_ps(u, v), oneM128));
    failed = _mm_or_ps(failed, _mm_cmplt_ps(t, zeroM128));
    failed = _mm_or_ps(failed, _mm_cmpgt_ps(t, m_length));
    failed = _mm_or_ps(failed, packedTris.inactiveMask);

    const __m128 tResults = _mm_blendv_ps(t, minusOneM128, failed);
    
    int mask = _mm_movemask_ps(tResults);
    //simdpp::store(temp, tResults);

    if (mask != 0xFF)
    {
        result.idx = -1;

        float* ptr = (float*)&tResults;
        for (int i = 0; i < SIMD_VECTOR_WIDTH; ++i)
        {
            
            //find the cloest hit point put the t of the ray in result.t
            if (ptr[i] >= 0.0f && ptr[i] < result.t)
            {
                result.t = ptr[i];
                result.idx = i;
            }
        }

        return result.idx != -1;
    }
    
    return false;

}
testRay::testRay(const Ray& r)
{
    Vector3R o = r.getOrigin();
    Vector3R dir = r.getDirection();
    float length = std::abs(r.getMaxT() - r.getMinT()) * dir.length();
    m_origin[0] = _mm_set_ps1(o.x);
    m_origin[1] = _mm_set_ps1(o.y);
    m_origin[2] = _mm_set_ps1(o.z);
    m_direction[0] = _mm_set_ps1(dir.x);
    m_direction[1] = _mm_set_ps1(dir.y);
    m_direction[2] = _mm_set_ps1(dir.z);
    m_length = _mm_set_ps1(length);
}


}


//SIMDPP_MAKE_DISPATCHER(template <typename T> void avx_multi_cross());

