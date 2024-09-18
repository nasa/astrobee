// CLASS_GEQRF/GELQF
//
void call_routines(struct RoutineArgs *args)
	{

	// unpack args

	// allocate memory for work
	int memsize = WORKSIZE(BLASFEO(ROUTINE))(args->m, args->n);
	void *mem;
	v_zeros_align(&mem, memsize);

	int ref_memsize = WORKSIZE(BLASFEO(REF(ROUTINE)))(args->m, args->n);
	void *ref_mem;
	v_zeros_align(&ref_mem, ref_memsize);


	// routine call
	//
	BLASFEO(ROUTINE)(
		args->m, args->n,
		args->sA_po, args->ai, args->aj,
		args->sD, args->di, args->dj,
		mem
		);

	BLASFEO(REF(ROUTINE))(
		args->m, args->n,
		args->rA_po, args->ai, args->aj,
		args->rD, args->di, args->dj,
		ref_mem
		);
	
	// free memory
	v_free_align(mem);
	v_free_align(ref_mem);

	}



void print_routine(struct RoutineArgs *args)
	{
	// unpack args

	printf("%s\n", string(ROUTINE));
	printf(
		"Solving A[%d:%d,%d:%d] = (QR) LQ[%d:%d,%d:%d]\n",
		args->ai, args->m, args->aj,  args->n,
		args->di, args->m, args->dj, args->n
	);

	}



void print_routine_matrices(struct RoutineArgs *args)
	{
		printf("\nPrint A:\n");
		blasfeo_print_xmat_debug(args->m, args->n, args->sA_po, args->ai, args->aj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->m, args->n, args->rA_po, args->ai, args->aj, 0, 0, 0, "REF");

		printf("\nPrint LU:\n");
		blasfeo_print_xmat_debug(args->m, args->n, args->sD, args->ai, args->aj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->m, args->n, args->rD, args->ai, args->aj, 0, 0, 0, "REF");
	}



void set_test_args(struct TestArgs *targs)
	{
	targs->nis = 9;
	targs->njs = 9;
	targs->nks = 9;

	targs->alphas = 1;
	}
