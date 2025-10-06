/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
* Copyright (C) 2019 by Gianluca Frison.                                                          *
* Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              *
* All rights reserved.                                                                            *
*                                                                                                 *
* The 2-Clause BSD License                                                                        *
*                                                                                                 *
* Redistribution and use in source and binary forms, with or without                              *
* modification, are permitted provided that the following conditions are met:                     *
*                                                                                                 *
* 1. Redistributions of source code must retain the above copyright notice, this                  *
*    list of conditions and the following disclaimer.                                             *
* 2. Redistributions in binary form must reproduce the above copyright notice,                    *
*    this list of conditions and the following disclaimer in the documentation                    *
*    and/or other materials provided with the distribution.                                       *
*                                                                                                 *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 *
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          *
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR                 *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                  *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    *
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     *
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   *
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    *
*                                                                                                 *
* Author: Ian McInerney                                                                           *
*                                                                                                 *
**************************************************************************************************/

#include <blasfeo_processor_features.h>
#include <blasfeo_target.h>

#if defined(TARGET_X64_INTEL_HASWELL) \
    || defined(TARGET_X64_INTEL_SANDY_BRIDGE) \
    || defined(TARGET_X64_INTEL_CORE) \
    || defined(TARGET_X64_AMD_BULLDOZER) \
    || defined(TARGET_X86_AMD_JAGUAR) \
    || defined(TARGET_X86_AMD_BARCELONA)
#if defined(__GNUC__) || defined(__clang__)
#include <cpuid.h>
// define missing bit_AVX2 (e.g. in case of clang compiler)
#ifndef bit_AVX2
#define bit_AVX2 (1 << 5)
#endif
#endif
#endif

// x86-64 CPU features
#define BLASFEO_PROCESSOR_FEATURE_AVX  0x0001    /// AVX instruction set
#define BLASFEO_PROCESSOR_FEATURE_AVX2 0x0002    /// AVX2 instruction set
#define BLASFEO_PROCESSOR_FEATURE_FMA  0x0004    /// FMA instruction set
#define BLASFEO_PROCESSOR_FEATURE_SSE3 0x0008    /// SSE3 instruction set

// ARM CPU features
#define BLASFEO_PROCESSOR_FEATURE_VFPv3  0x0100  /// VFPv3 instruction set
#define BLASFEO_PROCESSOR_FEATURE_NEON   0x0100  /// NEON instruction set
#define BLASFEO_PROCESSOR_FEATURE_VFPv4  0x0100  /// VFPv4 instruction set
#define BLASFEO_PROCESSOR_FEATURE_NEONv2 0x0100  /// NEONv2 instruction set



void blasfeo_processor_feature_string( int features, char* featureString )
{
    int idx = 0;
    if( features & BLASFEO_PROCESSOR_FEATURE_AVX )
    {
        featureString[idx++] = ' ';
        featureString[idx++] = 'A';
        featureString[idx++] = 'V';
        featureString[idx++] = 'X';
    }

    if( features & BLASFEO_PROCESSOR_FEATURE_AVX2 )
    {
        featureString[idx++] = ' ';
        featureString[idx++] = 'A';
        featureString[idx++] = 'V';
        featureString[idx++] = 'X';
        featureString[idx++] = '2';
    }

    if( features & BLASFEO_PROCESSOR_FEATURE_FMA )
    {
        featureString[idx++] = ' ';
        featureString[idx++] = 'F';
        featureString[idx++] = 'M';
        featureString[idx++] = 'A';
    }

    if( features & BLASFEO_PROCESSOR_FEATURE_SSE3 )
    {
        featureString[idx++] = ' ';
        featureString[idx++] = 'S';
        featureString[idx++] = 'S';
        featureString[idx++] = 'E';
        featureString[idx++] = '3';
    }

    featureString[idx] = 0;
}


void blasfeo_processor_library_string( char* featureString )
{
    int features = 0;
    blasfeo_processor_library_features( &features );
    blasfeo_processor_feature_string( features, featureString );
}


void blasfeo_processor_library_features( int* features )
{
    *features = 0;

    #if defined(TARGET_NEED_FEATURE_AVX)
    *features |= BLASFEO_PROCESSOR_FEATURE_AVX;
    #endif

    #if defined(TARGET_NEED_FEATURE_AVX2)
    *features |= BLASFEO_PROCESSOR_FEATURE_AVX2;
    #endif

    #if defined(TARGET_NEED_FEATURE_FMA)
    *features |= BLASFEO_PROCESSOR_FEATURE_FMA;
    #endif

    #if defined(TARGET_NEED_FEATURE_SSE3)
    *features |= BLASFEO_PROCESSOR_FEATURE_SSE3;
    #endif
}


int blasfeo_processor_cpu_features( int* features )
{
    *features = 0;

#if defined(TARGET_X64_INTEL_HASWELL) \
    || defined(TARGET_X64_INTEL_SANDY_BRIDGE) \
    || defined(TARGET_X64_INTEL_CORE) \
    || defined(TARGET_X64_AMD_BULLDOZER) \
    || defined(TARGET_X86_AMD_JAGUAR) \
    || defined(TARGET_X86_AMD_BARCELONA)

// GCC and clang provide the __get_cpuid function
#if defined(__GNUC__) || defined(__clang__)
    unsigned int reg_eax, reg_ebx, reg_ecx, reg_edx;

    // Test for normal features first in leaf 1
    __get_cpuid( 1, &reg_eax, &reg_ebx, &reg_ecx, &reg_edx );

    // AVX is in the ECX register of leaf 1
    if( reg_ecx & bit_AVX )
        *features |= BLASFEO_PROCESSOR_FEATURE_AVX;

    // FMA is in the ECX register of leaf 1
    if( reg_ecx & bit_FMA )
        *features |= BLASFEO_PROCESSOR_FEATURE_FMA;

    // SSE3 is in the ECX register of leaf 1
    if( reg_ecx & bit_SSE3 )
        *features |= BLASFEO_PROCESSOR_FEATURE_SSE3;

    // Test for extended features next in leaf 7 (subleaf 0)
#if __GNUC__>5
    __get_cpuid_count( 7, 0, &reg_eax, &reg_ebx, &reg_ecx, &reg_edx );
#else
	reg_eax = 7;
	reg_ecx = 0;
//	asm volatile(
	__asm__ volatile(
		"cpuid\n"
		: "=a" (reg_eax), "=b" (reg_ebx), "=c" (reg_ecx), "=d" (reg_edx)
		: "a" (reg_eax), "c" (reg_ecx));
#endif

    // AVX2 is in the EBX register of leaf 7
    if( reg_ebx & bit_AVX2 )
        *features |= BLASFEO_PROCESSOR_FEATURE_AVX2;
#endif  // #if defined(__GNUC__) || defined(__clang__)

#endif // x86 processors

    // Get the features needed by the current library
    int libraryFeatures = 0;
    blasfeo_processor_library_features( &libraryFeatures );

    return ( libraryFeatures == ( *features & libraryFeatures ) ) ? 1 : 0;
}
