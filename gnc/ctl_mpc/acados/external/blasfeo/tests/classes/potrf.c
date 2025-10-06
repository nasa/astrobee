// CLASS_POTRF
//

void call_routines(struct RoutineArgs *args)
	{
	// routine call
	//
	BLASFEO(ROUTINE)(
		args->m,
		args->sA_po, args->ai, args->aj,
		args->sD, args->di, args->dj
		);

	BLASFEO(REF(ROUTINE))(
		args->m,
		args->rA_po, args->ai, args->aj,
		args->rD, args->di, args->dj
		);
	}



void print_routine(struct RoutineArgs *args)
	{
	printf("blasfeo_%s(%d, A, %d, %d, D, %d, %d);\n", string(ROUTINE), args->m, args->ai, args->aj, args->di, args->dj);
	}



void print_routine_matrices(struct RoutineArgs *args)
	{
	printf("\nPrint A:\n");
	blasfeo_print_xmat_debug(args->m, args->n, args->sA_po, args->ai, args->aj, 0, 0, 0, "HP");
	blasfeo_print_xmat_debug(args->m, args->n, args->rA_po, args->ai, args->aj, 0, 0, 0, "REF");

	printf("\nPrint D:\n");
	blasfeo_print_xmat_debug(args->m, args->n, args->sD, args->ai, args->aj, 0, 0, 0, "HP");
	blasfeo_print_xmat_debug(args->m, args->n, args->rD, args->ai, args->aj, 0, 0, 0, "REF");
	}



void set_test_args(struct TestArgs *targs)
	{
	targs->nis = 21;
	}
