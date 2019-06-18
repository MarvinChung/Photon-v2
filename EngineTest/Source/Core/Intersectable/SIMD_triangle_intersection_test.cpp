#include <Core/Intersectable/IntelSimdBvh/SimdTriangleIntersection.h>
// #define SIMDPP_ARCH_X86_SSE3
// #define SIMDPP_EMIT_DISPATCHER 1
// //#define SIMDPP_DISPATCH_ARCH1 SIMDPP_ARCH_X86_SSE2 
// #define SIMDPP_DISPATCH_ARCH1 SIMDPP_ARCH_X86_SSE3 
// #define SIMDPP_DISPATCH_ARCH2 SIMDPP_ARCH_X86_SSE4_1 
// #define SIMDPP_DISPATCH_ARCH3 SIMDPP_ARCH_X86_AVX
// #define SIMDPP_DISPATCH_ARCH4 SIMDPP_ARCH_X86_AVX2

#include <Core/Ray.h>
#include <Math/TVector3.h>

#include <gtest/gtest.h>
//#include "simdpp/simd.h"
// #include <simdpp/dispatch/get_arch_gcc_builtin_cpu_supports.h>
// #include <simdpp/dispatch/get_arch_raw_cpuid.h>
// #include <simdpp/dispatch/get_arch_linux_cpuinfo.h>

// #if SIMDPP_HAS_GET_ARCH_RAW_CPUID
// #define SIMDPP_USER_ARCH_INFO ::simdpp::get_arch_raw_cpuid()
// #elif SIMDPP_HAS_GET_ARCH_GCC_BUILTIN_CPU_SUPPORTS
// #define SIMDPP_USER_ARCH_INFO ::simdpp::get_arch_gcc_builtin_cpu_supports()
// #elif SIMDPP_HAS_GET_ARCH_LINUX_CPUINFO
// #define SIMDPP_USER_ARCH_INFO ::simdpp::get_arch_linux_cpuinfo()
// #else
// #error "Unsupported platform"
// #endif


// namespace SIMDPP_ARCH_NAMESPACE {

// void print_arch()
// {
// 	std::cout << static_cast<unsigned>(simdpp::this_compile_arch()) << '\n';
// }

// } // namespace SIMDPP_ARCH_NAMESPACE

//SIMDPP_MAKE_DISPATCHER_VOID0(print_arch);

TEST(RayWithPackedTriangleTest, HitReturnIsCorrect)
{
	using namespace ph;
	Ray r(Vector3R(1.0f,1.0f,1.0f), Vector3R(0.0f,0.0f,1.0f), 0.0f , 100.0f);
	// std::cout << r.getOrigin().x << "," << r.getOrigin().y << "," << r.getOrigin().z << std::endl;
	// std::cout << r.getDirection().x << "," << r.getDirection().y << "," << r.getDirection().z << std::endl;
	PackedTriangle tri;
	PackedIntersectionResult results;

	// this case should not appear( A triangle is a point ) ! 
	// tri.e1[0] = simdpp::splat(0.0f);
	// tri.e1[1] = simdpp::splat(0.0f);
	// tri.e1[2] = simdpp::splat(0.0f);

	// tri.e2[0] = simdpp::splat(0.0f);
	// tri.e2[1] = simdpp::splat(0.0f);
	// tri.e2[2] = simdpp::splat(0.0f);

	// tri.v0[0] = simdpp::splat(0.0f);
	// tri.v0[1] = simdpp::splat(0.0f);
	// tri.v0[2] = simdpp::splat(0.0f);

	// EXPECT_EQ(ray.isIntersectPackedTriangle(tri, results), false);

	//case : a ray perpendicular hit a triangle's vertex -> false

	Vector3R v0[8];
	Vector3R v1[8];
	Vector3R v2[8];
	

	for(int i = 0 ; i < 8 ; ++i)
	{
		Vector3R tp0(0, 0, i+2);
		Vector3R tp1(0, 3, i+2);
		Vector3R tp2(3,0, i+2);
		v0[i] = tp0;
		v1[i] = tp1;
		v2[i] = tp2;
	}
	// std::cout << "tris" <<std::endl;

	// for(int i =0 ;i < 8 ; i++)
	// {
	// 	std::cout << v0[i].x << "," << v0[i].y << "," << v0[i].z << std::endl;
	// 	std::cout << v1[i].x << "," << v1[i].y << "," << v1[i].z << std::endl;
	// 	std::cout << v2[i].x << "," << v2[i].y << "," << v2[i].z << std::endl;
	// }

	// std::cout << "end tris" << std::endl;
	//normal test
	tri.e1[0] = _mm_set_ps(v1[0].x-v0[0].x, v1[1].x-v0[1].x, v1[2].x-v0[2].x, v1[3].x-v0[3].x);
	tri.e1[1] = _mm_set_ps(v1[0].y-v0[0].y, v1[1].y-v0[1].y, v1[2].y-v0[2].y, v1[3].y-v0[3].y);
	tri.e1[2] = _mm_set_ps(v1[0].z-v0[0].z, v1[1].z-v0[1].z, v1[2].z-v0[2].z, v1[3].z-v0[3].z);
	
	tri.e2[0] = _mm_set_ps(v2[0].x-v0[0].x, v2[1].x-v0[1].x, v2[2].x-v0[2].x, v2[3].x-v0[3].x);
	tri.e2[1] = _mm_set_ps(v2[0].y-v0[0].y, v2[1].y-v0[1].y, v2[2].y-v0[2].y, v2[3].y-v0[3].y);
	tri.e2[2] = _mm_set_ps(v2[0].z-v0[0].z, v2[1].z-v0[1].z, v2[2].z-v0[2].z, v2[3].z-v0[3].z);

	tri.v0[0] = _mm_set_ps(v0[0].x, v0[1].x, v0[2].x, v0[3].x);
	tri.v0[1] = _mm_set_ps(v0[0].y, v0[1].y, v0[2].y, v0[3].y);
	tri.v0[2] = _mm_set_ps(v0[0].z, v0[1].z, v0[2].z, v0[3].z);

	__m128 all_zero = _mm_set_ps1(0.0f);
	tri.inactiveMask = all_zero;
	testRay ray(r);
	EXPECT_EQ(ray.isIntersectPackedTriangle(tri, results), true);
	EXPECT_EQ(results.idx, 0);
	EXPECT_EQ(results.t, 1);

	r.setOrigin(Vector3R(0.0f,0.0f,0.0f));
	EXPECT_EQ(ray.isIntersectPackedTriangle(tri, results), false);

	//case : a ray parrallel hit a triangle
	r.setOrigin(Vector3R(-1, 0, 2));
	r.setDirection(Vector3R(1, 0, 0));
	EXPECT_EQ(ray.isIntersectPackedTriangle(tri, results), false);

	unsigned seed = (unsigned)time(NULL);
	srand(seed);
	const int X = 1000000;
	ph::testTriangle normaltri[200];
	ph::PackedTriangle packedtri[200];
	float r1, r2, r3, r4 ,r5 ,r6 ,r7, r8, r9;
	for(int i = 0 ;i < 200 ;i ++)
	{
		r1 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));
		r2 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));
		r3 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));

		r4 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));
		r5 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));
		r6 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));

		r7 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));
		r8 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));
		r9 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));

		normaltri[i].setVertex(Vector3R(r1, r2, r3), Vector3R(r4, r5, r6), Vector3R(r7 ,r8, r9));
	}


	testTriangle temp[SIMD_VECTOR_WIDTH];
	for(int i = 0 ;i < 200/SIMD_VECTOR_WIDTH ;i ++)
	{
		for(int j = 0; j < SIMD_VECTOR_WIDTH; j ++)
		{
			r1 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));
			r2 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));
			r3 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));

			r4 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));
			r5 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));
			r6 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));

			r7 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));
			r8 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));
			r9 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));

			normaltri[j].setVertex(Vector3R(r1, r2, r3), Vector3R(r4, r5, r6), Vector3R(r7 ,r8, r9));
			temp[j] = normaltri[j];
		}
		packedtri[i].setVertex(temp);
	}


	PackedIntersectionResult tp_results;
	clock_t  begin = clock();

	for(int i = 0; i < X/(200 * SIMD_VECTOR_WIDTH); i++)
	{
		for(int j = 0 ; j < 200 ; j++ )
		{
			ray.isIntersectPackedTriangle(packedtri[j], tp_results);
			EXPECT_EQ(tp_results.t,tp_results.t);
		}
	}

	clock_t end = clock();
	double  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	std::cout << "packed triangle time" << elapsed_secs << std::endl;




	Vector3R outIntersectionPoint;


	begin = clock();

	for(int i = 0; i < X/200; i ++)
	{
		for(int j =0 ;j < 200; j ++)
		{
			RayIntersectsTriangle(r.getOrigin(), 
	                           	r.getDirection(), 
	                           normaltri[j],
	                           outIntersectionPoint);
			EXPECT_EQ(outIntersectionPoint.x, outIntersectionPoint.x);
		}		

	}



	end = clock();
	elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	std::cout << "normal triangle time" << elapsed_secs << std::endl;



	//print_arch();
	
}



