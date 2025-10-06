// CLASS_TRM
//
void call_routines(struct RoutineArgs *args)
	{

	// unpack args

	// routine call
	//
	BLASFEO(ROUTINE)(
		args->m, args->n, args->alpha,
		args->sA, args->ai, args->aj,
		args->sB, args->bi, args->bj,
		args->sD, args->di, args->dj);

	BLASFEO(REF(ROUTINE))(
		args->m, args->n, args->alpha,
		args->rA, args->ai, args->aj,
		args->rB, args->bi, args->bj,
		args->rD, args->di, args->dj);

	}



void print_routine(struct RoutineArgs *args)
	{
	printf("blasfeo_%s(%d, %d, %f, A, %d, %d, B, %d, %d, D, %d, %d);\n", string(ROUTINE), args->m, args->n, args->alpha, args->ai, args->aj, args->bi, args->bj, args->ci, args->cj);
	}



void print_routine_matrices(struct RoutineArgs *args)
	{
	if(	!strcmp(string(ROUTINE), "dtrsm_llnn") | !strcmp(string(ROUTINE), "strsm_llnn") |
		!strcmp(string(ROUTINE), "dtrsm_llnu") | !strcmp(string(ROUTINE), "strsm_llnu") |
		!strcmp(string(ROUTINE), "dtrsm_lltn") | !strcmp(string(ROUTINE), "strsm_lltn") |
		!strcmp(string(ROUTINE), "dtrsm_lltu") | !strcmp(string(ROUTINE), "strsm_lltu") |
		!strcmp(string(ROUTINE), "dtrsm_lunn") | !strcmp(string(ROUTINE), "strsm_lunn") |
		!strcmp(string(ROUTINE), "dtrsm_lunu") | !strcmp(string(ROUTINE), "strsm_lunu") |
		!strcmp(string(ROUTINE), "dtrsm_lutn") | !strcmp(string(ROUTINE), "strsm_lutn") |
		!strcmp(string(ROUTINE), "dtrsm_lutu") | !strcmp(string(ROUTINE), "strsm_lutu") )
		{
		printf("\nPrint A:\n");
		blasfeo_print_xmat_debug(args->n, args->n, args->sA, args->ai, args->aj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->n, args->n, args->rA, args->ai, args->aj, 0, 0, 0, "REF");
		}
	else
		{
		printf("\nPrint A:\n");
		blasfeo_print_xmat_debug(args->m, args->m, args->sA, args->ai, args->aj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->m, args->m, args->rA, args->ai, args->aj, 0, 0, 0, "REF");
		}

	printf("\nPrint B:\n");
	blasfeo_print_xmat_debug(args->m, args->n, args->sB, args->ai, args->aj, 0, 0, 0, "HP");
	blasfeo_print_xmat_debug(args->m, args->n, args->rB, args->ai, args->aj, 0, 0, 0, "REF");

	printf("\nPrint D:\n");
	blasfeo_print_xmat_debug(args->m, args->n, args->sD, args->ai, args->aj, 0, 0, 0, "HP");
	blasfeo_print_xmat_debug(args->m, args->n, args->rD, args->ai, args->aj, 0, 0, 0, "REF");
	}



void set_test_args(struct TestArgs *targs)
	{
	targs->nis = 21;
	targs->njs = 21;
//	targs->nks = 20;

//	targs->ni0 = 10;
//	targs->nj0 = 10;
//	targs->nk0 = 10;

	targs->alphas = 1;
	}
