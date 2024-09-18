static char* test_sym_structure()
{
  //A matrix data
  QDLDL_int Ap[]  = {0, 2, 4};
  QDLDL_int Ai[]  = {0, 1, 0, 1};
  QDLDL_float Ax[] = {5.0,1.0,1.0,5.0};
  QDLDL_int An = 2;

  // RHS for Ax = b
  QDLDL_float b[]    = {1,1};

  //x replaces b during solve
  int status = ldl_factor_solve(An,Ap,Ai,Ax,b);

  mu_assert("Fully symmetric input not detected", status < 0);

  return 0;
}
