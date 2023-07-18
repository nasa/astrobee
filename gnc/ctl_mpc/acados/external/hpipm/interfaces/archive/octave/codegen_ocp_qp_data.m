%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                                 %
% This file is part of HPIPM.                                                                     %
%                                                                                                 %
% HPIPM -- High Performance Interior Point Method.                                                %
% Copyright (C) 2017 by Gianluca Frison.                                                          %
% Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              %
% All rights reserved.                                                                            %
%                                                                                                 %
% HPIPM is free software; you can redistribute it and/or                                          %
% modify it under the terms of the GNU Lesser General Public                                      %
% License as published by the Free Software Foundation; either                                    %
% version 2.1 of the License, or (at your option) any later version.                              %
%                                                                                                 %
% HPIPM is distributed in the hope that it will be useful,                                        %
% but WITHOUT ANY WARRANTY; without even the implied warranty of                                  %
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                                            %
% See the GNU Lesser General Public License for more details.                                     %
%                                                                                                 %
% You should have received a copy of the GNU Lesser General Public                                %
% License along with HPIPM; if not, write to the Free Software                                    %
% Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA                  %
%                                                                                                 %
% Author: Gianluca Frison, giaf (at) dtu.dk                                                       %
%                                                                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function codegen_ocp_qp_data( dims, qp, sol_guess );


% filename
filename = "ocp_qp_data.c";
% open file
fid = fopen(filename, "w");


% extract dims
N = dims.N;
nx = dims.nx;
nu = dims.nu;
nbx = dims.nbx;
nbu = dims.nbu;
ng = dims.ng;
ns = dims.ns;

% N
fprintf(fid, "int N = %d;\n", N);
% nx
fprintf(fid, "static int nnx[] = {");
for ii=1:N
	fprintf(fid, "%d, ", nx(ii));
end
fprintf(fid, "%d};\n", nx(ii+1));
% nu
fprintf(fid, "static int nnu[] = {");
for ii=1:N
	fprintf(fid, "%d, ", nu(ii));
end
fprintf(fid, "%d};\n", nu(ii+1));
% nbx
fprintf(fid, "static int nnbx[] = {");
for ii=1:N
	fprintf(fid, "%d, ", nbx(ii));
end
fprintf(fid, "%d};\n", nbx(ii+1));
% nbu
fprintf(fid, "static int nnbu[] = {");
for ii=1:N
	fprintf(fid, "%d, ", nbu(ii));
end
fprintf(fid, "%d};\n", nbu(ii+1));
% ng
fprintf(fid, "static int nng[] = {");
for ii=1:N
	fprintf(fid, "%d, ", ng(ii));
end
fprintf(fid, "%d};\n", ng(ii+1));
% ns
fprintf(fid, "static int nns[] = {");
for ii=1:N
	fprintf(fid, "%d, ", ns(ii));
end
fprintf(fid, "%d};\n", ns(ii+1));
%
fprintf(fid, "\n");


% A
for ii=1:N
	fprintf(fid, "static double A%d[] = {", ii-1);
	for jj=1:nx(ii+1)*nx(ii)
		fprintf(fid, "%18.15e, ", qp.A{ii}(:)(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% B
for ii=1:N
	fprintf(fid, "static double B%d[] = {", ii-1);
	for jj=1:nx(ii+1)*nu(ii)
		fprintf(fid, "%18.15e, ", qp.B{ii}(:)(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% b
for ii=1:N
	fprintf(fid, "static double b%d[] = {", ii-1);
	for jj=1:nx(ii+1)
		fprintf(fid, "%18.15e, ", qp.b{ii}(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% Q
for ii=1:N+1
	fprintf(fid, "static double Q%d[] = {", ii-1);
	for jj=1:nx(ii)*nx(ii)
		fprintf(fid, "%18.15e, ", qp.Q{ii}(:)(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% R
for ii=1:N+1
	fprintf(fid, "static double R%d[] = {", ii-1);
	for jj=1:nu(ii)*nu(ii)
		fprintf(fid, "%18.15e, ", qp.R{ii}(:)(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% S
for ii=1:N+1
	fprintf(fid, "static double S%d[] = {", ii-1);
	for jj=1:nu(ii)*nx(ii)
		fprintf(fid, "%18.15e, ", qp.S{ii}(:)(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% q
for ii=1:N+1
	fprintf(fid, "static double q%d[] = {", ii-1);
	for jj=1:nx(ii)
		fprintf(fid, "%18.15e, ", qp.q{ii}(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% r
for ii=1:N+1
	fprintf(fid, "static double r%d[] = {", ii-1);
	for jj=1:nu(ii)
		fprintf(fid, "%18.15e, ", qp.r{ii}(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% idxb
for ii=1:N+1
	fprintf(fid, "static int idxb%d[] = {", ii-1);
	for jj=1:nbu(ii)
		kk0 = 0;
		for kk=1:nx(ii)
			if kk0==0 && qp.Ju{ii}(jj,kk)!=0
				kk0 = kk;
				fprintf(fid, "%d, ", kk-1);
			end
		end
	end
	for jj=1:nbx(ii)
		kk0 = 0;
		for kk=1:nx(ii)
			if kk0==0 && qp.Jx{ii}(jj,kk)!=0
				kk0 = kk;
				fprintf(fid, "%d, ", nu(ii)+kk-1);
			end
		end
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% lb
for ii=1:N+1
	fprintf(fid, "static double lb%d[] = {", ii-1);
	for jj=1:nbu(ii)
		fprintf(fid, "%18.15e, ", qp.lu{ii}(jj));
	end
	for jj=1:nbx(ii)
		fprintf(fid, "%18.15e, ", qp.lx{ii}(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% ub
for ii=1:N+1
	fprintf(fid, "static double ub%d[] = {", ii-1);
	for jj=1:nbu(ii)
		fprintf(fid, "%18.15e, ", qp.uu{ii}(jj));
	end
	for jj=1:nbx(ii)
		fprintf(fid, "%18.15e, ", qp.ux{ii}(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% C
for ii=1:N+1
	fprintf(fid, "static double C%d[] = {", ii-1);
	for jj=1:ng(ii)*nx(ii)
		fprintf(fid, "%18.15e, ", qp.C{ii}(:)(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% D
for ii=1:N+1
	fprintf(fid, "static double D%d[] = {", ii-1);
	for jj=1:ng(ii)*nu(ii)
		fprintf(fid, "%18.15e, ", qp.D{ii}(:)(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% lg
for ii=1:N+1
	fprintf(fid, "static double lg%d[] = {", ii-1);
	for jj=1:ng(ii)
		fprintf(fid, "%18.15e, ", qp.lg{ii}(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% ug
for ii=1:N+1
	fprintf(fid, "static double ug%d[] = {", ii-1);
	for jj=1:ng(ii)
		fprintf(fid, "%18.15e, ", qp.ug{ii}(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% Zl
for ii=1:N+1
	fprintf(fid, "static double Zl%d[] = {", ii-1);
	for jj=1:ns(ii)
		fprintf(fid, "%18.15e, ", qp.Zl{ii}(jj,jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% Zu
for ii=1:N+1
	fprintf(fid, "static double Zu%d[] = {", ii-1);
	for jj=1:ns(ii)
		fprintf(fid, "%18.15e, ", qp.Zu{ii}(jj,jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% zl
for ii=1:N+1
	fprintf(fid, "static double zl%d[] = {", ii-1);
	for jj=1:ns(ii)
		fprintf(fid, "%18.15e, ", qp.zl{ii}(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% zu
for ii=1:N+1
	fprintf(fid, "static double zu%d[] = {", ii-1);
	for jj=1:ns(ii)
		fprintf(fid, "%18.15e, ", qp.zu{ii}(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% idxs
for ii=1:N+1
	Jt = [qp.Jsbu{ii}; qp.Jsbx{ii}; qp.Jsg{ii}]';
	fprintf(fid, "static int idxs%d[] = {", ii-1);
	for jj=1:ns(ii)
		kk0 = 0;
		for kk=1:nbx(ii)+nbu(ii)+ng(ii)
			if kk0==0 && Jt(jj,kk)!=0
				kk0 = kk;
				fprintf(fid, "%d, ", kk-1);
			end
		end
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% lls
for ii=1:N+1
	fprintf(fid, "static double lls%d[] = {", ii-1);
	for jj=1:ns(ii)
		fprintf(fid, "%18.15e, ", qp.lls{ii}(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% lus
for ii=1:N+1
	fprintf(fid, "static double lus%d[] = {", ii-1);
	for jj=1:ns(ii)
		fprintf(fid, "%18.15e, ", qp.lus{ii}(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");

% u_guess
for ii=1:N+1
	fprintf(fid, "static double u_guess%d[] = {", ii-1);
	for jj=1:nu(ii)
		fprintf(fid, "%18.15e, ", sol_guess.u{ii}(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% x_guess
for ii=1:N+1
	fprintf(fid, "static double x_guess%d[] = {", ii-1);
	for jj=1:nx(ii)
		fprintf(fid, "%18.15e, ", sol_guess.x{ii}(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% sl_guess
for ii=1:N+1
	fprintf(fid, "static double sl_guess%d[] = {", ii-1);
	for jj=1:ns(ii)
		fprintf(fid, "%18.15e, ", sol_guess.sl{ii}(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");
% su_guess
for ii=1:N+1
	fprintf(fid, "static double su_guess%d[] = {", ii-1);
	for jj=1:ns(ii)
		fprintf(fid, "%18.15e, ", sol_guess.su{ii}(jj));
	end
	fprintf(fid, "};\n");
end
fprintf(fid, "\n");



% AA
fprintf(fid, "static double *AA[] = {");
for ii=1:N-1
	fprintf(fid, "A%d, ", ii-1);
end
fprintf(fid, "A%d};\n", ii);
% BB
fprintf(fid, "static double *BB[] = {");
for ii=1:N-1
	fprintf(fid, "B%d, ", ii-1);
end
fprintf(fid, "B%d};\n", ii);
% bb
fprintf(fid, "static double *bb[] = {");
for ii=1:N-1
	fprintf(fid, "b%d, ", ii-1);
end
fprintf(fid, "b%d};\n", ii);
% QQ
fprintf(fid, "static double *QQ[] = {");
for ii=1:N
	fprintf(fid, "Q%d, ", ii-1);
end
fprintf(fid, "Q%d};\n", ii);
% RR
fprintf(fid, "static double *RR[] = {");
for ii=1:N
	fprintf(fid, "R%d, ", ii-1);
end
fprintf(fid, "R%d};\n", ii);
% SS
fprintf(fid, "static double *SS[] = {");
for ii=1:N
	fprintf(fid, "S%d, ", ii-1);
end
fprintf(fid, "S%d};\n", ii);
% qq
fprintf(fid, "static double *qq[] = {");
for ii=1:N
	fprintf(fid, "q%d, ", ii-1);
end
fprintf(fid, "q%d};\n", ii);
% rr
fprintf(fid, "static double *rr[] = {");
for ii=1:N
	fprintf(fid, "r%d, ", ii-1);
end
fprintf(fid, "r%d};\n", ii);
% iidxb
fprintf(fid, "static int *iidxb[] = {");
for ii=1:N
	fprintf(fid, "idxb%d, ", ii-1);
end
fprintf(fid, "idxb%d};\n", ii);
% llb
fprintf(fid, "static double *llb[] = {");
for ii=1:N
	fprintf(fid, "lb%d, ", ii-1);
end
fprintf(fid, "lb%d};\n", ii);
% uub
fprintf(fid, "static double *uub[] = {");
for ii=1:N
	fprintf(fid, "ub%d, ", ii-1);
end
fprintf(fid, "ub%d};\n", ii);
% CC
fprintf(fid, "static double *CC[] = {");
for ii=1:N
	fprintf(fid, "C%d, ", ii-1);
end
fprintf(fid, "C%d};\n", ii);
% DD
fprintf(fid, "static double *DD[] = {");
for ii=1:N
	fprintf(fid, "D%d, ", ii-1);
end
fprintf(fid, "D%d};\n", ii);
% llg
fprintf(fid, "static double *llg[] = {");
for ii=1:N
	fprintf(fid, "lg%d, ", ii-1);
end
fprintf(fid, "lg%d};\n", ii);
% uug
fprintf(fid, "static double *uug[] = {");
for ii=1:N
	fprintf(fid, "ug%d, ", ii-1);
end
fprintf(fid, "ug%d};\n", ii);
% ZZl
fprintf(fid, "static double *ZZl[] = {");
for ii=1:N
	fprintf(fid, "Zl%d, ", ii-1);
end
fprintf(fid, "Zl%d};\n", ii);
% ZZu
fprintf(fid, "static double *ZZu[] = {");
for ii=1:N
	fprintf(fid, "Zu%d, ", ii-1);
end
fprintf(fid, "Zu%d};\n", ii);
% zzl
fprintf(fid, "static double *zzl[] = {");
for ii=1:N
	fprintf(fid, "zl%d, ", ii-1);
end
fprintf(fid, "zl%d};\n", ii);
% zzu
fprintf(fid, "static double *zzu[] = {");
for ii=1:N
	fprintf(fid, "zu%d, ", ii-1);
end
fprintf(fid, "zu%d};\n", ii);
% iidxs
fprintf(fid, "static int *iidxs[] = {");
for ii=1:N
	fprintf(fid, "idxs%d, ", ii-1);
end
fprintf(fid, "idxs%d};\n", ii);
% llls
fprintf(fid, "static double *llls[] = {");
for ii=1:N
	fprintf(fid, "lls%d, ", ii-1);
end
fprintf(fid, "lls%d};\n", ii);
% llus
fprintf(fid, "static double *llus[] = {");
for ii=1:N
	fprintf(fid, "lus%d, ", ii-1);
end
fprintf(fid, "lus%d};\n", ii);

% uu_guess
fprintf(fid, "static double *uu_guess[] = {");
for ii=1:N
	fprintf(fid, "u_guess%d, ", ii-1);
end
fprintf(fid, "u_guess%d};\n", ii);
% xx_guess
fprintf(fid, "static double *xx_guess[] = {");
for ii=1:N
	fprintf(fid, "x_guess%d, ", ii-1);
end
fprintf(fid, "x_guess%d};\n", ii);
% ssl_guess
fprintf(fid, "static double *ssl_guess[] = {");
for ii=1:N
	fprintf(fid, "sl_guess%d, ", ii-1);
end
fprintf(fid, "sl_guess%d};\n", ii);
% ssu_guess
fprintf(fid, "static double *ssu_guess[] = {");
for ii=1:N
	fprintf(fid, "su_guess%d, ", ii-1);
end
fprintf(fid, "su_guess%d};\n", ii);



fprintf(fid, "\n");



fprintf(fid, "int *nu = nnu;\n");
fprintf(fid, "int *nx = nnx;\n");
fprintf(fid, "int *nbu = nnbu;\n");
fprintf(fid, "int *nbx = nnbx;\n");
fprintf(fid, "int *ng = nng;\n");
fprintf(fid, "int *ns = nns;\n");

fprintf(fid, "double **hA = AA;\n");
fprintf(fid, "double **hB = BB;\n");
fprintf(fid, "double **hb = bb;\n");
fprintf(fid, "double **hQ = QQ;\n");
fprintf(fid, "double **hR = RR;\n");
fprintf(fid, "double **hS = SS;\n");
fprintf(fid, "double **hq = qq;\n");
fprintf(fid, "double **hr = rr;\n");
fprintf(fid, "int **hidxb = iidxb;\n");
fprintf(fid, "double **hlb = llb;\n");
fprintf(fid, "double **hub = uub;\n");
fprintf(fid, "double **hC = CC;\n");
fprintf(fid, "double **hD = DD;\n");
fprintf(fid, "double **hlg = llg;\n");
fprintf(fid, "double **hug = uug;\n");
fprintf(fid, "double **hZl = ZZl;\n");
fprintf(fid, "double **hZu = ZZu;\n");
fprintf(fid, "double **hzl = zzl;\n");
fprintf(fid, "double **hzu = zzu;\n");
fprintf(fid, "int **hidxs = iidxs;\n");
fprintf(fid, "double **hlls = llls;\n");
fprintf(fid, "double **hlus = llus;\n");
fprintf(fid, "double **hu_guess = uu_guess;\n");
fprintf(fid, "double **hx_guess = xx_guess;\n");
fprintf(fid, "double **hsl_guess = ssl_guess;\n");
fprintf(fid, "double **hsu_guess = ssu_guess;\n");



% close file
fclose(fid);


return;
