#pragma once
// #define SIMDPP_ARCH_X86_SSE4_1 
// #define SIMDPP_ARCH_X86_SSE2
// #define SIMDPP_ARCH_X86_AVX2
// #define SIMDPP_ARCH_X86_AVX
// #define SIMDPP_ARCH_X86_SSE3
// #define SIMDPP_ARCH_X86_SSSE3
// #define SIMDPP_ARCH_X86_POPCNT_INSN
// #define SIMDPP_ARCH_X86_FMA3
// #define SIMDPP_ARCH_X86_FMA4
// #define SIMDPP_ARCH_X86_XOP
// #define SIMDPP_ARCH_X86_AVX512F
// #define SIMDPP_ARCH_X86_AVX512BW	
// #define SIMDPP_ARCH_X86_AVX512DQ
// #define SIMDPP_ARCH_X86_AVX512VL
// #define SIMDPP_ARCH_ARM_NEON
// #define SIMDPP_ARCH_ARM_NEON_FLT_SP
// #define SIMDPP_ARCH_ARM_NEON
// #define SIMDPP_ARCH_ARM_NEON_FLT_SP
// #define SIMDPP_ARCH_POWER_ALTIVEC
// #define SIMDPP_ARCH_POWER_VSX_206
// #define SIMDPP_ARCH_POWER_VSX_207
// #define SIMDPP_ARCH_MIPS_MSA


// #define SIMDPP_ARCH_X86_SSE3
// #define SIMDPP_EMIT_DISPATCHER 1
// #define SIMDPP_DISPATCH_ARCH1 SIMDPP_ARCH_X86_SSE2 
// //#define SIMDPP_DISPATCH_ARCH1 SIMDPP_ARCH_X86_SSE3 
// #define SIMDPP_DISPATCH_ARCH2 SIMDPP_ARCH_X86_SSE4_1 
// #define SIMDPP_DISPATCH_ARCH3 SIMDPP_ARCH_X86_AVX
// #define SIMDPP_DISPATCH_ARCH4 SIMDPP_ARCH_X86_AVX2





#include <limits>
#include <iostream>
#include <cmath>
#include <vector>
#include <cstdint>
#include "Core/Ray.h"
#include <bitset>
#include "Common/config.h"


// #include "simdpp/simd.h"
// #include <simdpp/dispatch/get_arch_gcc_builtin_cpu_supports.h>
// #include <simdpp/dispatch/get_arch_raw_cpuid.h>
// #include <simdpp/dispatch/get_arch_linux_cpuinfo.h>

constexpr int SIMD_VECTOR_WIDTH = 4;
#include <xmmintrin.h>
#include <immintrin.h>

// ifconstexpr