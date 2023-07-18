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
%	Filename:  interfaces/matlab/qpDUNES_options.m
%	Author:    Janick Frasch, Hans Joachim Ferreau
%	Version:   1.0beta
%	Date:      2012
%

%---------------------------------------------------------------------------------
%
%Returns a struct containing values for all options to be used within qpDUNES.
%
%Call
%    options = qpDUNES_options( 'default' );
%    ...
%to obtain a set of default options or a pre-defined set of options 
%
%Call
%    options = qpDUNES_options( 'option1',value1,'option2',value2,... )
%to obtain a set of default options but with 'option1' set to value1 etc.
%
%Call
%    options = qpDUNES_options( oldopts,'option1',value1,... )
%to obtain a copy of the options struct oldopts but with 'option1' set to 
%value1 etc.
%
%Call
%    options = qpDUNES_options( 'default', 'option1',value1,... )
%to obtain a set of default options, but with 'option1' set to value1 etc.
%
%
%qpDUNES features the following options:
%  maxIter                    -  Maximum number of iterations (if set
%                                to -1, a value is chosen heuristically)
%  printLevel                 -  0: no printed output,
%                                1: only error messages are printed,
%                                2: iterations and error messages are printed,
%                                3: all available messages are printed.
%
%  ... more to come
%
%
%
function [ options ] = qpDUNES_options( varargin )

	firstIsStructOrScheme = 0;

	if ( nargin == 0 ) 
		options = qpDUNES_default_options();
	else
		if ( isstruct( varargin{1} ) )
			if ( mod( nargin,2 ) ~= 1 )
				error('ERROR (qpDUNES_options): Options must be specified in pairs!');
			end
			options = varargin{1};
			firstIsStructOrScheme = 1;
		else
			if ( ischar( varargin{1} ) )
				if ( mod( nargin,2 ) == 0 )
					options = qpDUNES_default_options();
				else
					if ( ( nargin > 1 ) && ( ischar( varargin{nargin} ) ) )
						error('ERROR (qpDUNES_options): Options must be specified in pairs!');
					end

					switch ( varargin{1} )
						case 'default'
							options = qpDUNES_default_options();
						otherwise
							error( ['ERROR (qpDUNES_options): Only the following option schemes are defined: ''default''!'] );
							
					end
					firstIsStructOrScheme = 1;
				end
			else
				error('ERROR (qpDUNES_options): First argument needs to be a string or an options struct!');
			end
		end
	end

	% set options to user-defined values  
	for i=(1+firstIsStructOrScheme):2:nargin

		argName  = varargin{i};
		argValue = varargin{i+1};

		if ( ( isempty( argName ) ) || ( ~ischar( argName ) ) )
			error('ERROR (qpDUNES_options): Argument no. %d has to be a non-empty string!',i );
        end
			
		if ( ( ischar(argValue) ) || ( ~isscalar( argValue ) ) )
			error('ERROR (qpDUNES_options): Argument no. %d has to be a numerical constant!',i+1 );
        end

		if ( ~isfield( options,argName ) )
			error('ERROR (qpDUNES_options): Argument no. %d is an invalid option!',i );
		end

		eval( ['options.',argName,' = ',num2str(argValue),';'] );

	end

end


function [ options ] = qpDUNES_default_options( )

	% setup options struct with default values
	options = struct(	... %iterations
						'maxIter',								100, ...
						'maxNumLineSearchIterations',			19, ...			% 0.3^19 = 1e-10
						'maxNumLineSearchRefinementIterations',	49, ...			% 0.62^49 = 1e-10
                        'maxNumQpoasesIterations',              1000, ...
                        ... %logging
						'logLevel',				0, ...
						... %printing
						'printLevel',				1, ...
						'printIntervalHeader',		20, ...
						'printIterationTiming',		0, ...		% QPDUNES_FALSE
						'printLineSearchTiming',	0, ...		% QPDUNES_FALSE
						... % numerical tolerances
						'stationarityTolerance', 		1.e-6, ...
						'equalityTolerance',     		2.221e-16, ...
						'newtonHessDiagRegTolerance',   1.e-10, ...
						'QPDUNES_ZERO',             		1.e-50, ...
						'QPDUNES_INFTY',            		1.e12, ...
						... % other options
                        'checkForInfeasibility',        0, ...
                        'allowSuboptimalTermination',   0, ...
                        ... % regularization options
						'regType', 		0, ...		% LEVENBERG_MARQUARDT
						'regParam',		1.e-6, ...
						... % line search options
						'lsType',							4, ...			% ACCELERATED_GRADIENT_BISECTION_LS
						'lineSearchReductionFactor',		0.3, ...		% needs to be between 0 and 1
						'lineSearchIncreaseFactor',			1.5, ...		% needs to be greater than 1
						'lineSearchMinAbsProgress',    	2.221e-16, ...
						'lineSearchMinRelProgress',    	1.e-14, ...
						'lineSearchStationarityTolerance', 1.e-3, ...
						'lineSearchMaxStepSize',   			1., ...
						'lineSearchNbrGridPoints',   		5 ...
						);

end


%% END OF FILE


