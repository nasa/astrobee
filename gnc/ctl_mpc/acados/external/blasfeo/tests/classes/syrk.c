// CLASS_SYRK
//
void call_routines(struct RoutineArgs *args)
	{
	// routine call
	//
	BLASFEO(ROUTINE)(
		args->m, args->n, args->alpha,
		args->sA, args->ai, args->aj,
		args->sB, args->bi, args->bj, args->beta,
		args->sC, args->ci, args->cj,
		args->sD, args->di, args->dj);

	BLASFEO(REF(ROUTINE))(
		args->m, args->n, args->alpha,
		args->rA, args->ai, args->aj,
		args->rB, args->bi, args->bj, args->beta,
		args->rC, args->ci, args->cj,
		args->rD, args->di, args->dj);

	}



void print_routine(struct RoutineArgs *args)
	{
	printf("blasfeo_%s(%d, %d, %f, A, %d, %d, B, %d, %d, %f, C, %d, %d, D, %d, %d);\n", string(ROUTINE), args->m, args->n, args->alpha, args->ai, args->aj, args->bi, args->bj, args->beta, args->ci, args->cj, args->di, args->dj);
	}



void print_routine_matrices(struct RoutineArgs *args)
	{
	if(!strcmp(string(ROUTINE), "dsyrk_ln") || !strcmp(string(ROUTINE), "dsyrk_un") || !strcmp(string(ROUTINE), "ssyrk_ln") || !strcmp(string(ROUTINE), "ssyrk_un"))
		{
		printf("\nPrint A:\n");
		blasfeo_print_xmat_debug(args->m, args->n, args->sA, args->ai, args->aj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->m, args->n, args->rA, args->ai, args->aj, 0, 0, 0, "REF");

		printf("\nPrint B:\n");
		blasfeo_print_xmat_debug(args->m, args->n, args->sB, args->bi, args->bj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->m, args->n, args->rB, args->bi, args->bj, 0, 0, 0, "REF");
		}
	else
		{
		printf("\nPrint A:\n");
		blasfeo_print_xmat_debug(args->n, args->m, args->sA, args->ai, args->aj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->n, args->m, args->rA, args->ai, args->aj, 0, 0, 0, "REF");

		printf("\nPrint B:\n");
		blasfeo_print_xmat_debug(args->n, args->m, args->sB, args->bi, args->bj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->n, args->m, args->rB, args->bi, args->bj, 0, 0, 0, "REF");
		}

	printf("\nPrint C:\n");
	blasfeo_print_xmat_debug(args->m, args->m, args->sC, args->ai, args->aj, 0, 0, 0, "HP");
	blasfeo_print_xmat_debug(args->m, args->m, args->rC, args->ai, args->aj, 0, 0, 0, "REF");

	printf("\nPrint D:\n");
	blasfeo_print_xmat_debug(args->m, args->m, args->sD, args->ai, args->aj, 0, 0, 0, "HP");
	blasfeo_print_xmat_debug(args->m, args->m, args->rD, args->ai, args->aj, 0, 0, 0, "REF");
	}



void set_test_args(struct TestArgs *targs)
	{
	targs->nis = 21;
	targs->njs = 9;
	}
