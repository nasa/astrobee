/**************************************************************************************************
*                                                                                                 *
* This file is part of HPIPM.                                                                     *
*                                                                                                 *
* HPIPM -- High-Performance Interior Point Method.                                                *
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



#include "../include/hpipm_tree.h"
#include "../include/hpipm_scenario_tree.h"



static int ipow(int base, int exp)
	{
	int result = 1;
	while(exp)
		{
		if(exp & 1)
			result *= base;
		exp >>= 1;
		base *= base;
		}
	return result;
	}



static int sctree_node_number(int md, int Nr, int Nh)
	{
	int n_nodes;
	if(md==1) // i.e. standard block-banded structure
		n_nodes = Nh+1;
	else
		n_nodes = (Nh-Nr)*ipow(md,Nr) + (ipow(md,Nr+1)-1)/(md-1);
	return n_nodes;
	}



hpipm_size_t sctree_memsize(int md, int Nr, int Nh)
	{

	int Nn = sctree_node_number(md, Nr, Nh);

	hpipm_size_t size = 0;

	size += Nn*sizeof(struct node); // root
	size += Nn*sizeof(int); // kids

	return size;

	}



void sctree_create(int md, int Nr, int Nh, struct sctree *st, void *memory)
	{

	st->memsize = sctree_memsize(md, Nr, Nh);

	int Nn = sctree_node_number(md, Nr, Nh);

	st->md = md;
	st->Nr = Nr;
	st->Nh = Nh;
	st->Nn = Nn;

	struct node *n_ptr = (struct node *) memory;
	st->root = n_ptr;
	n_ptr += Nn; // root

	int *i_ptr = (int *) n_ptr;
	st->kids = i_ptr;
	i_ptr += Nn; // kids

	int ii;
	int idx, dad, stage, real, nkids, idxkid;
	int tkids;
	struct node *node0, *node1;

	tkids = 0;
	idxkid = 0;

	// root
	node0 = st->root+0;
	node0->idx = 0;
	node0->stage = 0;
	node0->dad = -1;
	node0->real = -1;
	node0->idxkid = 0;

	// kids
	for(idx=0; idx<Nn; idx++)
		{
		node0 = st->root+idx;
		stage = node0->stage;
		if(stage<Nr)
			nkids = md;
		else if(stage<Nh)
			nkids = 1;
		else 
			nkids = 0;
		node0->nkids = nkids;
		if(nkids>0)
			{
			node0->kids = st->kids+tkids;
			tkids += nkids;
			if(nkids>1)
				{
				for(ii=0; ii<nkids; ii++)
					{
					idxkid++;
					node0->kids[ii] = idxkid;
					node1 = st->root+idxkid;
					node1->idx = idxkid;
					node1->stage = stage+1;
					node1->dad = idx;
					node1->real = ii;
					node1->idxkid = ii;
					}
				}
			else // nkids==1
				{
				idxkid++;
				node0->kids[0] = idxkid;
				node1 = st->root+idxkid;
				node1->idx = idxkid;
				node1->stage = stage+1;
				node1->dad = idx;
				node1->real = node0->real;
				node1->idxkid = 0;
				}
			}
		}

	return;

	}



void sctree_cast_to_tree(struct sctree *st, struct tree *tt)
	{

	tt->root = st->root;
	tt->kids = st->kids;
	tt->Nn = st->Nn;
	tt->memsize = st->memsize;

	return;

	}
