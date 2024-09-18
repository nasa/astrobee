static char* test_two_by_two()
{
  //A matrix data
  QDLDL_int Ap[]  = {0,1,3};
  QDLDL_int Ai[]  = {0, 0, 1};
  QDLDL_float Ax[] = {1.0, 1.0, -1.0};
  QDLDL_int An = 2;

  // RHS and solution to Ax = b
  QDLDL_float b[]    = {2,4};
  QDLDL_float xsol[] = {3,-1};

  //x replaces b during solve
  int status = ldl_factor_solve(An,Ap,Ai,Ax,b);

  mu_assert("Factorisation failed", status >= 0);
  mu_assert("Solve accuracy failed", vec_diff_norm(b,xsol,An) < QDLDL_TESTS_TOL);

  return 0;
}
