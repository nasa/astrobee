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

#include <stdlib.h>
#include <stdio.h>


#include <blasfeo.h>



int main()
	{

	int nn, ii, rep, nrep;

	blasfeo_timer timer;
	double tmp_time, bandwidth_load_sequential, bandwidth_copy_sequential;

	int size[] = {  16, 32, 48,
					64, 80,
					96, 112, 1*1024/8,
					2*1024/8, 3*1024/8, 4*1024/8,
					6*1024/8, 8*1024/8, 12*1024/8,
					16*1024/8, 20*1024/8, 24*1024/8,
					28*1024/8, 30*1024/8, 32*1024/8, 34*1024/8, 36*1024/8,
					40*1024/8, 48*1024/8, 64*1024/8,
					96*1024/8, 128*1024/8, 136*1024/8, 144*1024/8, 160*1024/8, 192*1024/8, 256*1024/8,
					40960, 49152, 65536,
					81920, 98304, 1024*1024/8,
					1536*1024/8, 2*1024*1024/8, 3*1024*1024/8, 
					4*1024*1024/8, 6*1024*1024/8, 8*1024*1024/8, 10*1024*1024/8};

	int nnrep[] = { 4000000, 4000000, 4000000,
					2000000, 2000000,
					1000000, 1000000, 1000000,
					400000, 400000, 400000,
					200000, 200000, 200000,
					100000, 100000, 100000,
					40000, 40000, 40000, 40000, 40000,
					20000, 20000, 20000,
					10000, 10000, 10000, 10000, 10000, 10000, 10000,
					4000, 4000, 4000,
					2000, 2000, 2000,
					1000, 1000, 1000,
					400, 400, 400, 400};

	int n_size = 45;


	printf("\ntest #\tvec size (KB)\tload sequential\tcopy sequential\n");
	for(nn=0; nn<n_size; nn++)
//	for(nn=2; nn<3; nn++)
		{

		nrep = 8*nnrep[nn];

		double *x; blasfeo_malloc_align((void **) &x, size[nn]*sizeof(double));
		double *y; blasfeo_malloc_align((void **) &y, size[nn]*sizeof(double));

#if 0
		for(ii=0; ii<size[nn]; ii++)
			x[ii] = ii;

		d_print_mat(1, size[nn], x, 1);

		kernel_dveccp_inc1(7, x, y);

		d_print_mat(1, size[nn], y, 1);
		
		exit(1);
#endif

		/* load benchmark sequential */

		// load
		kernel_dvecld_inc1(size[nn], x);

		blasfeo_tic(&timer);

		for(rep=0; rep<nrep; rep++)
			{
			kernel_dvecld_inc1(size[nn], x);
			}

		tmp_time = blasfeo_toc(&timer) / nrep;

		bandwidth_load_sequential = size[nn] * 8.0 / tmp_time;


		/* copy benchmark sequential */

		// copy
		kernel_dveccp_inc1(size[nn], x, y);

		blasfeo_tic(&timer);

		for(rep=0; rep<nrep; rep++)
			{
			kernel_dveccp_inc1(size[nn], x, y);
			}

		tmp_time = blasfeo_toc(&timer) / nrep;

		bandwidth_copy_sequential = size[nn] * 2 * 8.0 / tmp_time;


		// print
		printf("%d\t%f\t%e\t%e\n", nn, size[nn]*8.0/1024.0, bandwidth_load_sequential, bandwidth_copy_sequential);

		blasfeo_free_align(x);
		blasfeo_free_align(y);

		}
	
	return 0;

	}
