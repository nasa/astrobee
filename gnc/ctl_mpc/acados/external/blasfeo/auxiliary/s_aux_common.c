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
#include <math.h>

#include <blasfeo_common.h>
#include <blasfeo_block_size.h>
#include <blasfeo_s_aux.h>



/* Explicitly panel-major */

// return the memory size (in bytes) needed for a strmat
size_t blasfeo_pm_memsize_smat(int ps, int m, int n)
	{
	int nc = 4; // hard coded !!! S_PLD
	int al = ps*nc;
	int pm = (m+ps-1)/ps*ps;
	int cn = (n+nc-1)/nc*nc;
	int tmp = m<n ? (m+al-1)/al*al : (n+al-1)/al*al; // al(min(m,n)) // XXX max ???
	size_t memsize = (pm*cn+tmp)*sizeof(float);
	return memsize;
	}



// create a matrix structure for a matrix of size m*n by using memory passed by a pointer
void blasfeo_pm_create_smat(int ps, int m, int n, struct blasfeo_pm_smat *sA, void *memory)
	{
	sA->mem = memory;
	sA->ps = ps;
	int nc = 4; // hard coded !!! S_PLD
	int al = ps*nc;
	sA->m = m;
	sA->n = n;
	int pm = (m+ps-1)/ps*ps;
	int cn = (n+nc-1)/nc*nc;
	sA->pm = pm;
	sA->cn = cn;
	float *ptr = (float *) memory;
	sA->pA = ptr;
	ptr += pm*cn;
	int tmp = m<n ? (m+al-1)/al*al : (n+al-1)/al*al; // al(min(m,n)) // XXX max ???
	sA->dA = ptr;
	ptr += tmp;
	sA->memsize = (pm*cn+tmp)*sizeof(float);
	sA->use_dA = 0; // invalidate stored inverse diagonal
	return;
	}



/* Explicitly column-major */

// return the memory size (in bytes) needed for a strmat
size_t blasfeo_cm_memsize_smat(int m, int n)
	{
	int tmp = m<n ? m : n; // al(min(m,n)) // XXX max ???
	size_t memsize = (m*n+tmp)*sizeof(float);
	return memsize;
	}



// create a matrix structure for a matrix of size m*n by using memory passed by a pointer
void blasfeo_cm_create_smat(int m, int n, struct blasfeo_pm_smat *sA, void *memory)
	{
	sA->mem = memory;
	sA->m = m;
	sA->n = n;
	sA->use_dA = 0; // invalidate stored inverse diagonal
	float *ptr = (float *) memory;
	sA->pA = ptr;
	ptr += m*n;
	int tmp = m<n ? m : n; // al(min(m,n)) // XXX max ???
	sA->dA = ptr;
	ptr += tmp;
	sA->use_dA = 0;
	sA->memsize = (m*n+tmp)*sizeof(float);
	return;
	}




