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
* Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             *
*                                                                                                 *
**************************************************************************************************/

#include "../include/blasfeo_timing.h"

#if (defined _WIN32 || defined _WIN64) && !(defined __MINGW32__ || defined __MINGW64__)

	void blasfeo_tic(blasfeo_timer* t) {
		QueryPerformanceFrequency(&t->freq);
		QueryPerformanceCounter(&t->tic);
	}

	double blasfeo_toc(blasfeo_timer* t) {
		QueryPerformanceCounter(&t->toc);
		return ((t->toc.QuadPart - t->tic.QuadPart) / (double)t->freq.QuadPart);
	}

#elif(defined __APPLE__)
	void blasfeo_tic(blasfeo_timer* t) {
		/* read current clock cycles */
		t->tic = mach_absolute_time();
	}

	double blasfeo_toc(blasfeo_timer* t) {
		uint64_t duration; /* elapsed time in clock cycles*/

		t->toc = mach_absolute_time();
		duration = t->toc - t->tic;

		/*conversion from clock cycles to nanoseconds*/
		mach_timebase_info(&(t->tinfo));
		duration *= t->tinfo.numer;
		duration /= t->tinfo.denom;

		return (double)duration / 1e9;
	}

#elif(defined __DSPACE__)

	void blasfeo_tic(blasfeo_timer* t) {
		ds1401_tic_start();
		t->time = ds1401_tic_read();
	}

	double blasfeo_toc(blasfeo_timer* t) {
		return ds1401_tic_read() - t->time;
	}

#elif defined(__XILINX_NONE_ELF__)

	void blasfeo_tic(blasfeo_timer* t) {
		XTime_GetTime(&(t->tic));
	}

	double blasfeo_toc(blasfeo_timer* t) {
		uint64_t toc;
		XTime_GetTime(&toc);
		t->toc = toc;

		/* time in s */
		return (double) (toc - t->tic) / (COUNTS_PER_SECOND);  
	}
#elif defined(__XILINX_ULTRASCALE_NONE_ELF_JAILHOUSE__)

#define mfcp(reg)	({long long unsigned int rval = 0U;\
			__asm__ __volatile__("mrs	%0, " #reg : "=r" (rval));\
			rval;\
			})
	void blasfeo_tic(blasfeo_timer* t) { 
		t->tic = mfcp(CNTPCT_EL0);
	}

	double blasfeo_toc(blasfeo_timer* t) {
		uint64_t toc;
		toc = mfcp(CNTPCT_EL0);
		t->toc = toc;

		/* time in s */
		return (double) (toc - t->tic) / (COUNTS_PER_SECOND);  
	}
#undef mfcp

#else

	#if __STDC_VERSION__ >= 199901L  // C99 Mode

		/* read current time */
		void blasfeo_tic(blasfeo_timer* t) {
			gettimeofday(&t->tic, 0);
		}

		/* return time passed since last call to tic on this timer */
		double blasfeo_toc(blasfeo_timer* t) {
			struct timeval temp;

			gettimeofday(&t->toc, 0);

			if ((t->toc.tv_usec - t->tic.tv_usec) < 0) {
				temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec - 1;
				temp.tv_usec = 1000000 + t->toc.tv_usec - t->tic.tv_usec;
			} else {
				temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec;
				temp.tv_usec = t->toc.tv_usec - t->tic.tv_usec;
			}

			return (double)temp.tv_sec + (double)temp.tv_usec / 1e6;
		}

	#else  // ANSI C Mode

		/* read current time */
		void blasfeo_tic(blasfeo_timer* t) {
			clock_gettime(CLOCK_MONOTONIC, &t->tic);
		}


		/* return time passed since last call to tic on this timer */
		double blasfeo_toc(blasfeo_timer* t) {
			struct timespec temp;

			clock_gettime(CLOCK_MONOTONIC, &t->toc);

			if ((t->toc.tv_nsec - t->tic.tv_nsec) < 0) {
				temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec - 1;
				temp.tv_nsec = 1000000000+t->toc.tv_nsec - t->tic.tv_nsec;
			} else {
				temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec;
				temp.tv_nsec = t->toc.tv_nsec - t->tic.tv_nsec;
			}

			return (double)temp.tv_sec + (double)temp.tv_nsec / 1e9;
		}

	#endif  // __STDC_VERSION__ >= 199901L

#endif  // (defined _WIN32 || _WIN64)
