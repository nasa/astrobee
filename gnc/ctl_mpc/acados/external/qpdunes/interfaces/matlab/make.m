%%
%	This file is part of qpDUNES.
%
%	qpDUNES -- A DUal NEwton Strategy for convex quadratic programming.
%	Copyright (C) 2012 by Janick Frasch, Hans Joachim Ferreau et al. 
%	All rights reserved.
%
%	qpDUNES is free software; you can redistribute it and/or
%	modify it under the terms of the GNU Lesser General Public
%	License as published by the Free Software Foundation; either
%	version 2.1 of the License, or (at your option) any later version.
% 
% 	qpDUNES is distributed in the hope that it will be useful,
% 	but WITHOUT ANY WARRANTY; without even the implied warranty of
% 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
% 	See the GNU Lesser General Public License for more details.
% 
% 	You should have received a copy of the GNU Lesser General Public
% 	License along with qpDUNES; if not, write to the Free Software
% 	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%
 



%
%	Filename:  interfaces/matlab/make.m
%	Author:    Janick Frasch, Hans Joachim Ferreau
%	Version:   1.0beta
%	Date:      2012
%


%%
% consistency check
if ( exist( [pwd, '/make.m'],'file' ) == 0 )
	disp( 'ERROR: Run this make script directly within the directory' );
	disp( '       <qpDUNES-inst-dir>/interfaces/matlab, please.' );
	return;
end


QPDUNESPATH = '../../';
QPOASESPATH = [ QPDUNESPATH , '/externals/qpOASES-3.0beta/'];

IFLAGS  = [ '-I.',' ', ...
            '-I',QPDUNESPATH,'include',' ', ...
            '-I',QPDUNESPATH,'externals/qpOASES-3.0beta/include',' ', ...
            '-I',QPDUNESPATH,'interfaces/mpc',' ' ];

OMPFLAGS = [ '', ' ', ...   %-openmp
			 ];

if ( ispc == 0 )
%  	CPPFLAGS  = [ IFLAGS, OMPFLAGS, '-largeArrayDims -D__cpluplus -D__MATLAB__ -cxx -O -D__NO_COPYRIGHT__ -DLINUX CFLAGS=''$CFLAGS -std=c99 -fPIC''', ' ' ]; %% -D__SUPPRESSANYOUTPUT__
	CPPFLAGS  = [ IFLAGS, '-largeArrayDims -D__DEBUG__ -D__cplusplus -D__MATLAB__ -cxx -O -D__NO_COPYRIGHT__ -DLINUX CFLAGS=''$CFLAGS -fPIC -std=c99''', ' ' ]; %% -D__DEBUG__ -D__SUPPRESSANYOUTPUT__
else
	CPPFLAGS  = [ IFLAGS, '-largeArrayDims -D__cplusplus -D__MATLAB__ -cxx -O -DWIN32', ' ' ]; %% -D__NO_COPYRIGHT__ -D__SUPPRESSANYOUTPUT__
end

QPDUNES_OBJECTS = [	QPDUNESPATH, 'src/stage_qp_solver_clipping.c ',...
					QPDUNESPATH, 'src/stage_qp_solver_qpoases.cpp ',...
					QPDUNESPATH, 'src/qpdunes_utils.c ',...
					QPDUNESPATH, 'src/dual_qp.c ',...
					QPDUNESPATH, 'src/matrix_vector.c ',...
					QPDUNESPATH, 'src/setup_qp.c ',...
					QPDUNESPATH, 'interfaces/mpc/setup_mpc.c ',...
                    ];
QPOASES_OBJECTS = [	QPOASESPATH, 'src/BLASReplacement.cpp ',...
					QPOASESPATH, 'src/LAPACKReplacement.cpp ',...
                    QPOASESPATH, 'src/LoggedSQProblem.cpp ',...
					QPOASESPATH, 'src/SQProblem.cpp ',...
					QPOASESPATH, 'src/QProblem.cpp ',...
					QPOASESPATH, 'src/QProblemB.cpp ',...
					QPOASESPATH, 'src/Bounds.cpp ',...
					QPOASESPATH, 'src/Constraints.cpp ',...
					QPOASESPATH, 'src/SubjectTo.cpp ',...
					QPOASESPATH, 'src/Indexlist.cpp ',...
					QPOASESPATH, 'src/Flipper.cpp ',...
					QPOASESPATH, 'src/Utils.cpp ',...
					QPOASESPATH, 'src/Options.cpp ',...
					QPOASESPATH, 'src/Matrices.cpp ',...
					QPOASESPATH, 'src/MessageHandling.cpp ',...
					];

DEBUGFLAGS = ' ';
% DEBUGFLAGS = ' -g -v CXXDEBUGFLAGS=''$CXXDEBUGFLAGS -Wall -pedantic -Wshadow'' ';

MEXOBJS = { 'qpDUNES', 
            'mpcDUNES' }

;
for i = 1:length(MEXOBJS)
% 	cmd = [ 'mex -v CC="gcc" CXX="gcc" LD="gcc" COPTIMFLAGS="$COPTIMFLAGS -O3 -fopenmp" -lgomp  -output ', MEXOBJS{i}, ' ', DEBUGFLAGS, CPPFLAGS, [MEXOBJS{i},'.cpp ',QPDUNES_OBJECTS] ]
    cmd = [ 'mex -v COPTIMFLAGS="$COPTIMFLAGS -O3" -output ', MEXOBJS{i}, ' ', DEBUGFLAGS, CPPFLAGS, [MEXOBJS{i},'.cpp ',QPOASES_OBJECTS QPDUNES_OBJECTS] ]
	
	eval( cmd );
	disp( [ MEXOBJS{i}, '.', eval('mexext'), ' successfully created!'] );
end

path( path,pwd );


clear QPDUNESPATH IFLAGS CPPFLAGS QPDUNES_OBJECTS DEBUGFLAGS MEXOBJS



%%
%%	end of file
%%
