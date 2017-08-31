% Copyright (c) 2017, United States Government, as represented by the
% Administrator of the National Aeronautics and Space Administration.
%
% All rights reserved.
%
% The Astrobee platform is licensed under the Apache License, Version 2.0
% (the "License"); you may not use this file except in compliance with the
% License. You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
% WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
% License for the specific language governing permissions and limitations
% under the License.

%---------------------------------------------------------------------------
%  MATLAB function for configuration set generated on 07-Sep-2016 07:29:09
%  MATLAB version: 8.6.0.267246 (R2015b)
%---------------------------------------------------------------------------
function cs=astrobee_configuration()

cs = Simulink.ConfigSet;
 
% Original configuration set version: 1.15.1
if cs.versionCompare('1.15.1') < 0
    error('Simulink:MFileVersionViolation', 'The version of the target configuration set is older than the original configuration set.');
end

% % Original environment character encoding: UTF-8
% if ~strcmpi(get_param(0, 'CharacterEncoding'), 'UTF-8')
%     warning('Simulink:EncodingUnMatched', 'The target character encoding (%s) is different from the original (%s).',  get_param(0, 'CharacterEncoding'), 'UTF-8');
% end
 
% Do not change the order of the following commands. There are dependencies between the parameters.
cs.set_param('SystemTargetFile', 'ert.tlc');   % System target file
cs.set_param('GenerateComments', 'on');   % Include comments 
cs.set_param('SimulinkBlockComments', 'on');   % Simulink block / Stateflow object comments 
cs.set_param('ShowEliminatedStatement', 'off');   % Show eliminated blocks 
cs.set_param('OperatorAnnotations', 'off');   % Operator annotations 
cs.set_param('ForceParamTrailComments', 'off');   % Verbose comments for SimulinkGlobal storage class 
cs.set_param('InsertBlockDesc', 'off');   % Simulink block descriptions 
cs.set_param('SimulinkDataObjDesc', 'off');   % Simulink data object descriptions 
cs.set_param('EnableCustomComments', 'off');   % Custom comments (MPT objects only) 
cs.set_param('SFDataObjDesc', 'off');   % Stateflow object descriptions 
cs.set_param('ReqsInCode', 'off');   % Requirements in block comments 
cs.set_param('IncludeHyperlinkInReport', 'off');   % Code-to-model 
cs.set_param('GenerateReport', 'off');   % Create code generation report 
cs.set_param('GenerateTraceInfo', 'off');   % Model-to-code 
cs.set_param('GenerateTraceReport', 'off');   % Eliminated / virtual blocks 
cs.set_param('GenerateTraceReportSl', 'off');   % Traceable Simulink blocks 
cs.set_param('GenerateTraceReportSf', 'off');   % Traceable Stateflow objects 
cs.set_param('GenerateTraceReportEml', 'off');   % Traceable MATLAB functions 
cs.set_param('LaunchReport', 'off');   % Open report automatically 
cs.set_param('GenerateCodeReplacementReport', 'off');   % Summarize which blocks triggered code replacements 
cs.set_param('GenerateCodeMetricsReport', 'off');   % Static code metrics 
cs.set_param('GenerateWebview', 'off');   % Generate model Web view 
cs.set_param('OptimizeBlockIOStorage', 'on');   % Signal storage reuse 
cs.set_param('ExpressionFolding', 'on');   % Eliminate superfluous local variables (expression folding) 
cs.set_param('ParamNamingRule', 'None');   % Parameter naming 
cs.set_param('RTWCompilerOptimization', 'off');   % Compiler optimization level 
cs.set_param('SignalNamingRule', 'None');   % Signal naming 
cs.set_param('UpdateModelReferenceTargets', 'IfOutOfDateOrStructuralChange');   % Rebuild 
cs.set_param('GenerateSampleERTMain', 'on');   % Generate an example main program 
cs.set_param('ProdHWDeviceType', '32-bit Generic');   % Device vendor 
cs.set_param('EnableUserReplacementTypes', 'off');   % Replace data type names in the generated code 
cs.set_param('DefaultParameterBehavior', 'Tunable');   % Default parameter behavior 
cs.set_param('ERTMultiwordTypeDef', 'System defined');   % Multiword type definitions 
cs.set_param('EnableMemcpy', 'on');   % Use memcpy for vector assignment 
cs.set_param('GenerateMakefile', 'off');   % Generate makefile 
cs.set_param('GlobalDataDefinition', 'Auto');   % Data definition 
cs.set_param('GlobalDataReference', 'Auto');   % Data declaration 
cs.set_param('SaveFinalState', 'off');   % Final states 
cs.set_param('SignalLogging', 'off');   % Signal logging 
cs.set_param('TargetUnknown', 'off');   % Target unknown 
cs.set_param('EnableConcurrentExecution', 'off');   % EnableConcurrentExecution 
cs.set_param('LoadExternalInput', 'off');   % Input 
cs.set_param('LoadInitialState', 'off');   % Initial state 
cs.set_param('SaveTime', 'off');   % Time 
cs.set_param('SaveState', 'off');   % States 
cs.set_param('SaveOutput', 'off');   % Output 
cs.set_param('LimitDataPoints', 'on');   % Limit data points to last 
cs.set_param('DSMLogging', 'off');   % Data stores 
cs.set_param('ReturnWorkspaceOutputs', 'off');   % Save simulation output as single object 
cs.set_param('SupportNonInlinedSFcns', 'off');   % non-inlined S-functions 
cs.set_param('PurelyIntegerCode', 'off');   % floating-point numbers 
cs.set_param('SupportNonFinite', 'on');   % non-finite numbers 
cs.set_param('MatFileLogging', 'off');   % MAT-file logging 
cs.set_param('SuppressErrorStatus', 'off');   % Suppress error status in real-time model data structure 
cs.set_param('SupportContinuousTime', 'off');   % continuous time 
cs.set_param('IncludeMdlTerminateFcn', 'on');   % Terminate function required 
cs.set_param('GRTInterface', 'off');   % Classic call interface 
cs.set_param('CombineOutputUpdateFcns', 'on');   % Single output/update function 
cs.set_param('ModelReferenceMinAlgLoopOccurrences', 'off');   % Minimize algebraic loop occurrences 
cs.set_param('PortableWordSizes', 'on');   % Enable portable word sizes 
cs.set_param('SolverType', 'Fixed-step');   % Type 
cs.set_param('AutoInsertRateTranBlk', 'off');   % Automatically handle rate transition for data transfer 
cs.set_param('ExtMode', 'off');   % Interface 
 
% The following commands do not have dependencies.
cs.set_param('Description', '');  % Description
cs.set_param('Name', 'Configuration');  % Name

% Solver pane
cs.set_param('FixedStep', 'astrobee_time_step_size');   % Fixed-step size (fundamental sample time) 
cs.set_param('PositivePriorityOrder', 'off');   % Higher priority value indicates higher task priority 
cs.set_param('SampleTimeConstraint', 'Unconstrained');   % Periodic sample time constraint 
cs.set_param('Solver', 'FixedStepDiscrete');   % Solver 
cs.set_param('SolverMode', 'Auto');   % Tasking mode for periodic sample times 
cs.set_param('StartTime', '0.0');   % Start time 
cs.set_param('StopTime', 'astrobee_stop_time');   % Stop time 

% Data Import/Export pane
cs.set_param('Decimation', '1');   % Decimation 
cs.set_param('LoggingIntervals', '[-inf, inf]');   % Logging intervals 
cs.set_param('MaxDataPoints', '1000');   % Limit data points to last 
cs.set_param('SaveFormat', 'Array');   % Format 
cs.set_param('SignalLoggingName', 'logsout');   % Signal logging name 
cs.set_param('SignalLoggingSaveFormat', 'Dataset');   % Signal logging format 
cs.set_param('StreamToWorkspace', 'off');   % Write streamed signals to workspace 
cs.set_param('StreamVariableName', 'streamout');   % Write streamed signals to workspace 
cs.set_param('VisualizeSimOutput', 'off');   % Enable live streaming of selected signals to Simulation Data Inspector 

% Optimization pane
cs.set_param('AccelVerboseBuild', 'on');   % Verbose accelerator builds 
cs.set_param('BlockReduction', 'on');   % Block reduction 
cs.set_param('BooleanDataType', 'on');   % Implement logic signals as Boolean data (vs. double) 
cs.set_param('ConditionallyExecuteInputs', 'on');   % Conditional input branch execution 
cs.set_param('DefaultUnderspecifiedDataType', 'double');   % Default for underspecified data type 
cs.set_param('EfficientFloat2IntCast', 'off');   % Remove code from floating-point to integer conversions that wraps out-of-range values 
cs.set_param('EfficientMapNaN2IntZero', 'on');   % Remove code from floating-point to integer conversions with saturation that maps NaN to zero 
cs.set_param('InitFltsAndDblsToZero', 'off');   % Use memset to initialize floats and doubles to 0.0 
cs.set_param('LifeSpan', '1');   % Application lifespan (days) 
cs.set_param('NoFixptDivByZeroProtection', 'off');   % Remove code that protects against division arithmetic exceptions 
cs.set_param('OptimizeModelRefInitCode', 'on');   % Optimize initialization code for model reference 
cs.set_param('SimCompilerOptimization', 'off');   % Compiler optimization level 
cs.set_param('UseDivisionForNetSlopeComputation', 'off');   % Use division for fixed-point net slope computation 
cs.set_param('UseFloatMulNetSlope', 'off');   % Use floating-point multiplication to handle net slope corrections 
cs.set_param('UseSpecifiedMinMax', 'off');   % Optimize using the specified minimum and maximum values 
cs.set_param('ZeroExternalMemoryAtStartup', 'on');   % Remove root level I/O zero initialization 
cs.set_param('ZeroInternalMemoryAtStartup', 'on');   % Remove internal data zero initialization 

% Optimization:Signals and Parameters pane
cs.set_param('BooleansAsBitfields', 'off');   % Pack Boolean data into bitfields 
cs.set_param('BufferReuse', 'on');   % Reuse local block outputs 
cs.set_param('GlobalBufferReuse', 'on');   % Reuse global block outputs 
cs.set_param('GlobalVariableUsage', 'None');   % Optimize global data access 
cs.set_param('InlinedParameterPlacement', 'Hierarchical');   % Parameter structure 
cs.set_param('LocalBlockOutputs', 'on');   % Enable local block outputs 
cs.set_param('MaxStackSize', 'Inherit from target');   % Maximum stack size (bytes) 
cs.set_param('MemcpyThreshold', 64);   % Memcpy threshold (bytes) 
cs.set_param('PassReuseOutputArgsAs', 'Structure reference');   % Pass reusable subsystem outputs as 
cs.set_param('RollThreshold', 5);   % Loop unrolling threshold 
cs.set_param('StrengthReduction', 'off');   % Simplify array indexing 

% Optimization:Stateflow pane
cs.set_param('ActiveStateOutputEnumStorageType', 'Native Integer');   % Base storage type for automatically created enumerations 
cs.set_param('DataBitsets', 'off');   % Use bitsets for storing Boolean data 
cs.set_param('StateBitsets', 'off');   % Use bitsets for storing state configuration 

% Diagnostics Solver pane
cs.set_param('AlgebraicLoopMsg', 'warning');   % Algebraic loop 
cs.set_param('ArtificialAlgebraicLoopMsg', 'warning');   % Minimize algebraic loop 
cs.set_param('BlockPriorityViolationMsg', 'warning');   % Block priority violation 
cs.set_param('ConsistencyChecking', 'none');   % Solver data inconsistency 
cs.set_param('MaxConsecutiveZCsMsg', 'error');   % Consecutive zero crossings violation 
cs.set_param('MinStepSizeMsg', 'warning');   % Min step size violation 
cs.set_param('ModelReferenceExtraNoncontSigs', 'error');   % Extraneous discrete derivative signals 
cs.set_param('SimStateInterfaceChecksumMismatchMsg', 'warning');   % SimState interface checksum mismatch 
cs.set_param('SimStateOlderReleaseMsg', 'error');   % SimState object from earlier release 
cs.set_param('SolverPrmCheckMsg', 'warning');   % Automatic solver parameter selection 
cs.set_param('StateNameClashWarn', 'none');   % State name clash 
cs.set_param('TimeAdjustmentMsg', 'none');   % Sample hit time adjusting 
cs.set_param('UnknownTsInhSupMsg', 'warning');   % Unspecified inheritability of sample time 

% Diagnostics:Sample Time pane
cs.set_param('InheritedTsInSrcMsg', 'warning');   % Source block specifies -1 sample time 
cs.set_param('MultiTaskCondExecSysMsg', 'error');   % Multitask conditionally executed subsystem 
cs.set_param('MultiTaskRateTransMsg', 'error');   % Multitask rate transition 
cs.set_param('SigSpecEnsureSampleTimeMsg', 'warning');   % Enforce sample times specified by Signal Specification blocks 
cs.set_param('SingleTaskRateTransMsg', 'none');   % Single task rate transition 
cs.set_param('TasksWithSamePriorityMsg', 'warning');   % Tasks with equal priority 

% Diagnostics:Data Validity pane
cs.set_param('ArrayBoundsChecking', 'none');   % Array bounds exceeded 
cs.set_param('AssertControl', 'UseLocalSettings');   % Model Verification block enabling 
cs.set_param('CheckMatrixSingularityMsg', 'none');   % Division by singular matrix 
cs.set_param('IntegerOverflowMsg', 'warning');   % Wrap on overflow 
cs.set_param('IntegerSaturationMsg', 'warning');   % Saturate on overflow 
cs.set_param('MultiTaskDSMMsg', 'error');   % Multitask data store 
cs.set_param('ParameterDowncastMsg', 'error');   % Detect downcast 
cs.set_param('ParameterOverflowMsg', 'error');   % Detect overflow 
cs.set_param('ParameterPrecisionLossMsg', 'warning');   % Detect precision loss 
cs.set_param('ParameterTunabilityLossMsg', 'warning');   % Detect loss of tunability 
cs.set_param('ParameterUnderflowMsg', 'none');   % Detect underflow 
cs.set_param('RTPrefix', 'error');   % "rt" prefix for identifiers 
cs.set_param('ReadBeforeWriteMsg', 'UseLocalSettings');   % Detect read before write 
cs.set_param('SignalInfNanChecking', 'none');   % Inf or NaN block output 
cs.set_param('SignalRangeChecking', 'none');   % Simulation range checking 
cs.set_param('SignalResolutionControl', 'UseLocalSettings');   % Signal resolution 
cs.set_param('UnderSpecifiedDataTypeMsg', 'none');   % Underspecified data types 
cs.set_param('UnderspecifiedInitializationDetection', 'Simplified');   % Underspecified initialization detection 
cs.set_param('UniqueDataStoreMsg', 'none');   % Duplicate data store names 
cs.set_param('WriteAfterReadMsg', 'UseLocalSettings');   % Detect write after read 
cs.set_param('WriteAfterWriteMsg', 'UseLocalSettings');   % Detect write after write 

% Diagnostics:Type Conversion pane
cs.set_param('FixptConstOverflowMsg', 'none');   % Detect overflow 
cs.set_param('FixptConstPrecisionLossMsg', 'none');   % Detect precision loss 
cs.set_param('FixptConstUnderflowMsg', 'none');   % Detect underflow 
cs.set_param('Int32ToFloatConvMsg', 'warning');   % 32-bit integer to single precision float conversion 
cs.set_param('UnnecessaryDatatypeConvMsg', 'none');   % Unnecessary type conversions 
cs.set_param('VectorMatrixConversionMsg', 'none');   % Vector/matrix block input conversion 

% Diagnostics:Connectivity pane
cs.set_param('BusNameAdapt', 'WarnAndRepair');   % Repair bus selections 
cs.set_param('BusObjectLabelMismatch', 'error');   % Element name mismatch 
cs.set_param('FcnCallInpInsideContextMsg', 'EnableAllAsError');   % Context-dependent inputs 
cs.set_param('InvalidFcnCallConnMsg', 'error');   % Invalid function-call connection 
cs.set_param('NonBusSignalsTreatedAsBus', 'none');   % Non-bus signals treated as bus signals 
cs.set_param('RootOutportRequireBusObject', 'warning');   % Unspecified bus object at root Outport block 
cs.set_param('SignalLabelMismatchMsg', 'none');   % Signal label mismatch 
cs.set_param('StrictBusMsg', 'ErrorLevel1');   % Mux blocks used to create bus signals 
cs.set_param('UnconnectedInputMsg', 'warning');   % Unconnected block input ports 
cs.set_param('UnconnectedLineMsg', 'warning');   % Unconnected line 
cs.set_param('UnconnectedOutputMsg', 'warning');   % Unconnected block output ports 

% Diagnostics:Compatibility pane
cs.set_param('FrameProcessingCompatibilityMsg', 'warning');   % Block behavior depends on frame status of signal 
cs.set_param('SFcnCompatibilityMsg', 'none');   % S-function upgrades needed 

% Diagnostics:Model Referencing pane
cs.set_param('ModelReferenceDataLoggingMessage', 'warning');   % Unsupported data logging 
cs.set_param('ModelReferenceIOMismatchMessage', 'none');   % Port and parameter mismatch 
cs.set_param('ModelReferenceIOMsg', 'none');   % Invalid root Inport/Outport block connection 
cs.set_param('ModelReferenceVersionMismatchMessage', 'none');   % Model block version mismatch 

% Diagnostics:Saving pane
cs.set_param('SaveWithDisabledLinksMsg', 'warning');   % Block diagram contains disabled library links 
cs.set_param('SaveWithParameterizedLinksMsg', 'warning');   % Block diagram contains parameterized library links 

% Diagnostics:Stateflow pane
cs.set_param('SFInvalidInputDataAccessInChartInitDiag', 'warning');   % Invalid input data access in chart initialization 
cs.set_param('SFNoUnconditionalDefaultTransitionDiag', 'warning');   % No unconditional default transitions 
cs.set_param('SFOutputUsedAsStateInMooreChartDiag', 'error');   % Read-before-write to output in Moore chart 
cs.set_param('SFTransitionActionBeforeConditionDiag', 'warning');   % Transition action specified before condition action 
cs.set_param('SFTransitionOutsideNaturalParentDiag', 'warning');   % Transition outside natural parent 
cs.set_param('SFUnconditionalTransitionShadowingDiag', 'warning');   % Transition shadowing 
cs.set_param('SFUndirectedBroadcastEventsDiag', 'warning');   % Undirected event broadcasts 
cs.set_param('SFUnexpectedBacktrackingDiag', 'warning');   % Unexpected backtracking 
cs.set_param('SFUnusedDataAndEventsDiag', 'warning');   % Unused data, events and messages 

% Hardware Implementation pane
cs.set_param('ProdIntDivRoundTo', 'Undefined');   % Signed integer division rounds to 
cs.set_param('ProdLargestAtomicFloat', 'None');   % floating-point 
cs.set_param('ProdLargestAtomicInteger', 'Char');   % integer 

% Model Referencing pane
cs.set_param('EnableParallelModelReferenceBuilds', 'off');   % Enable parallel model reference builds 
cs.set_param('EnableRefExpFcnMdlSchedulingChecks', 'on');   % Enable strict scheduling checks for referenced export-function models 
cs.set_param('ModelDependencies', '');   % Model dependencies 
cs.set_param('ModelReferenceNumInstancesAllowed', 'Multi');   % Total number of instances allowed per top model 
cs.set_param('ModelReferencePassRootInputsByReference', 'on');   % Pass fixed-size scalar root inputs by value for code generation 
cs.set_param('ParallelModelReferenceMATLABWorkerInit', 'None');   % MATLAB worker initialization for builds 
cs.set_param('PropagateSignalLabelsOutOfModel', 'off');   % Propagate all signal labels out of the model 
cs.set_param('PropagateVarSize', 'Infer from blocks in model');   % Propagate sizes of variable-size signals 

% Simulation Target General pane
cs.set_param('SFSimEcho', 'on');   % Echo expressions without semicolons 
cs.set_param('SimBuildMode', 'sf_incremental_build');   % Simulation target build mode 
cs.set_param('SimCtrlC', 'on');   % Ensure responsiveness 
cs.set_param('SimGenImportedTypeDefs', 'off');   % Generate typedefs for imported bus and enumeration types 
cs.set_param('SimIntegrity', 'on');   % Ensure memory integrity 

% Simulation Target:Custom Code pane
cs.set_param('SimCustomHeaderCode', '');   % Header file 
cs.set_param('SimCustomInitializer', '');   % Initialize function 
cs.set_param('SimCustomSourceCode', '');   % Source file 
cs.set_param('SimCustomTerminator', '');   % Terminate function 
cs.set_param('SimParseCustomCode', 'on');   % Parse custom code symbols 
cs.set_param('SimUserIncludeDirs', '');   % Include directories ./../../../../../local/eigen3/Eigen
cs.set_param('SimUserLibraries', '');   % Libraries 
cs.set_param('SimUserSources', '');   % Source files 

% Simulation Target:Symbols pane
cs.set_param('SimReservedNameArray', []);   % Reserved names 

% Code Generation General pane
cs.set_param('CheckMdlBeforeBuild', 'Warning');   % Check model before generating code 
cs.set_param('GenCodeOnly', 'on');   % Generate code only 
cs.set_param('IgnoreCustomStorageClasses', 'off');   % Ignore custom storage classes 
cs.set_param('IgnoreTestpoints', 'off');   % Ignore test point signals 
cs.set_param('ObjectivePriorities', []);   % Prioritized objectives 
cs.set_param('PackageGeneratedCodeAndArtifacts', 'off');   % Package code and artifacts 
cs.set_param('PackageName', '');   % Zip file name 
cs.set_param('TargetLang', 'C++');   % Language 

% Code Generation:Comments pane
cs.set_param('MATLABFcnDesc', 'off');   % MATLAB function help text 
cs.set_param('MATLABSourceComments', 'off');   % MATLAB source code as comments 

% Code Generation:Symbols pane
cs.set_param('CustomSymbolStrBlkIO', 'rtb_$N$M');   % Local block output variables 
cs.set_param('CustomSymbolStrFcn', '$R$N$M$F');   % Subsystem methods 
cs.set_param('CustomSymbolStrFcnArg', 'rt$I$N$M');   % Subsystem method arguments 
cs.set_param('CustomSymbolStrField', '$N$M');   % Field name of global types 
cs.set_param('CustomSymbolStrGlobalVar', '$R$N$M');   % Global variables 
cs.set_param('CustomSymbolStrMacro', '$R$N$M');   % Constant macros 
cs.set_param('CustomSymbolStrTmpVar', '$N$M');   % Local temporary variables 
cs.set_param('CustomSymbolStrType', '$N$R$M_T');   % Global types 
cs.set_param('CustomSymbolStrUtil', '$N$C');   % Shared utilities 
cs.set_param('DefineNamingRule', 'UpperCase');   % #define naming 
cs.set_param('InlinedPrmAccess', 'Literals');   % Generate scalar inlined parameters as 
cs.set_param('InternalIdentifier', 'Shortened');   % System-generated identifiers 
cs.set_param('MangleLength', 1);   % Minimum mangle length 
cs.set_param('MaxIdLength', 31);   % Maximum identifier length 
cs.set_param('ReservedNameArray', []);   % Reserved names 
cs.set_param('UseSimReservedNames', 'off');   % Use the same reserved names as Simulation Target 

% Code Generation:Custom Code pane
cs.set_param('CustomHeaderCode', '');   % Header file 
cs.set_param('CustomInclude', '');   % Include directories 
cs.set_param('CustomInitializer', '');   % Initialize function 
cs.set_param('CustomLibrary', '');   % Libraries 
cs.set_param('CustomSource', '');   % Source files 
cs.set_param('CustomSourceCode', '');   % Source file 
cs.set_param('CustomTerminator', '');   % Terminate function 
cs.set_param('RTWUseSimCustomCode', 'off');   % Use the same custom code settings as Simulation Target 

% Code Generation:Debug pane
cs.set_param('ProfileTLC', 'off');   % Profile TLC 
cs.set_param('RTWVerbose', 'on');   % Verbose build 
cs.set_param('RetainRTWFile', 'off');   % Retain .rtw file 
cs.set_param('TLCAssert', 'off');   % Enable TLC assertion 
cs.set_param('TLCCoverage', 'off');   % Start TLC coverage when generating code 
cs.set_param('TLCDebug', 'off');   % Start TLC debugger when generating code 

% Code Generation:Interface pane
cs.set_param('CodeInterfacePackaging', 'Reusable function');   % Code interface packaging 
cs.set_param('CodeReplacementLibrary', 'None');   % Code replacement library 
cs.set_param('CombineSignalStateStructs', 'off');   % Combine signal/state structures 
cs.set_param('GenerateASAP2', 'off');   % Interface 
cs.set_param('GenerateAllocFcn', 'on');   % Use dynamic memory allocation for model initialization 
cs.set_param('MultiInstanceErrorCode', 'Error');   % Multi-instance code error diagnostic 
cs.set_param('RootIOFormat', 'Individual arguments');   % Pass root-level I/O as 
cs.set_param('SupportAbsoluteTime', 'on');   % absolute time 
cs.set_param('SupportComplex', 'on');   % complex numbers 
cs.set_param('SupportVariableSizeSignals', 'on');   % variable-size signals 
cs.set_param('TargetLangStandard', 'C89/C90 (ANSI)');   % Standard math library 
cs.set_param('UtilityFuncGeneration', 'Shared location');   % Shared code placement 

% Code Generation:Verification pane
cs.set_param('CodeExecutionProfileVariable', 'executionProfile');   % Workspace variable 
cs.set_param('CodeExecutionProfiling', 'off');   % Measure task execution time 
cs.set_param('CodeProfilingInstrumentation', 'off');   % Measure function execution times 
cs.set_param('CodeProfilingSaveOptions', 'SummaryOnly');   % Save options 
cs.set_param('CreateSILPILBlock', 'None');   % Create block 
cs.set_param('SILDebugging', 'off');   % Enable source-level debugging for SIL 

% Code Generation:Code Style pane
cs.set_param('CastingMode', 'Nominal');   % Casting modes 
cs.set_param('ConvertIfToSwitch', 'off');   % Convert if-elseif-else patterns to switch-case statements 
cs.set_param('EnableSignedLeftShifts', 'on');   % Replace multiplications by powers of two with signed bitwise shifts 
cs.set_param('EnableSignedRightShifts', 'on');   % Allow right shifts on signed integers 
cs.set_param('IndentSize', '2');   % Indent size 
cs.set_param('IndentStyle', 'K&R');   % Indent style 
cs.set_param('ParenthesesLevel', 'Nominal');   % Parentheses level 
cs.set_param('PreserveExpressionOrder', 'off');   % Preserve operand order in expression 
cs.set_param('PreserveExternInFcnDecls', 'on');   % Preserve extern keyword in function declarations 
cs.set_param('PreserveIfCondition', 'off');   % Preserve condition expression in if statement 
cs.set_param('SuppressUnreachableDefaultCases', 'off');   % Suppress generation of default cases for Stateflow switch statements if unreachable 

% Code Generation:Templates pane
cs.set_param('ERTCustomFileTemplate', 'example_file_process.tlc');   % File customization template 
cs.set_param('ERTDataHdrFileTemplate', 'ert_code_template.cgt');   % Header file template 
cs.set_param('ERTDataSrcFileTemplate', 'ert_code_template.cgt');   % Source file template 
cs.set_param('ERTHdrFileBannerTemplate', 'ert_code_template.cgt');   % Header file template 
cs.set_param('ERTSrcFileBannerTemplate', 'ert_code_template.cgt');   % Source file template 
cs.set_param('TargetOS', 'BareBoardExample');   % Target operating system 

% Code Generation:Code Placement pane
cs.set_param('ERTFilePackagingFormat', 'Modular');   % File packaging format 
cs.set_param('EnableDataOwnership', 'off');   % Use owner from data object for data definition placement 
cs.set_param('IncludeFileDelimiter', 'Auto');   % #include file delimiter 
cs.set_param('ParamTuneLevel', 10);   % Parameter tune level 
cs.set_param('SignalDisplayLevel', 10);   % Signal display level 

% Code Generation:Memory Sections pane
cs.set_param('MemSecDataConstants', 'Default');   % Constants 
cs.set_param('MemSecDataIO', 'Default');   % Inputs/Outputs 
cs.set_param('MemSecDataInternal', 'Default');   % Internal data 
cs.set_param('MemSecDataParameters', 'Default');   % Parameters 
cs.set_param('MemSecFuncExecute', 'Default');   % Execution 
cs.set_param('MemSecFuncInitTerm', 'Default');   % Initialize/Terminate 
cs.set_param('MemSecFuncSharedUtil', 'Default');   % Shared utility 
cs.set_param('MemSecPackage', '--- None ---');   % Package 

% Parameters in List View only
cs.set_param('SolverName', 'FixedStepDiscrete');   % Solver:
cs.set_param('InspectSignalLogs', 'off');   % Record logged workspace data in Simulation Data Inspector
cs.set_param('AdvancedOptControl', '');   % AdvancedOptControl
cs.set_param('BufferReusableBoundary', 'on');   % BufferReusableBoundary
cs.set_param('AllowSymbolicDim', 'off');   % Allow symbolic dimension specification
cs.set_param('IgnoredZcDiagnostic', 'warning');   % IgnoredZcDiagnostic
cs.set_param('InitInArrayFormatMsg', 'warning');   % InitInArrayFormatMsg
cs.set_param('MaskedZcDiagnostic', 'warning');   % MaskedZcDiagnostic
cs.set_param('ModelReferenceSymbolNameMessage', 'warning');   % ModelReferenceSymbolNameMessage
cs.set_param('ProdEqTarget', 'on');   % Test hardware is the same as production hardware
cs.set_param('TargetPreprocMaxBitsSint', 32);   % TargetPreprocMaxBitsSint
cs.set_param('TargetPreprocMaxBitsUint', 32);   % TargetPreprocMaxBitsUint
cs.set_param('ParallelModelReferenceErrorOnInvalidPool', 'on');   % ParallelModelReferenceErrorOnInvalidPool
cs.set_param('SupportModelReferenceSimTargetCustomCode', 'off');   % SupportModelReferenceSimTargetCustomCode
cs.set_param('SFSimEnableDebug', 'off');   % Allow setting breakpoints during simulation
cs.set_param('Toolchain', 'Automatically locate an installed toolchain');   % Toolchain:
cs.set_param('BuildConfiguration', 'Faster Builds');   % Build configuration:
cs.set_param('PostCodeGenCommand', '');   % Post code generation command
cs.set_param('SaveLog', 'off');   % Save build log
cs.set_param('TLCOptions', '');   % TLC options:
cs.set_param('CommentStyle', 'Auto');   % Comment style
cs.set_param('IncAutoGenComments', 'off');   % IncAutoGenComments
cs.set_param('IncDataTypeInIds', 'off');   % IncDataTypeInIds
cs.set_param('IncHierarchyInIds', 'off');   % IncHierarchyInIds
cs.set_param('InsertPolySpaceComments', 'off');   % Insert Polyspace comments
cs.set_param('PreserveName', 'off');   % PreserveName
cs.set_param('PreserveNameWithParent', 'off');   % PreserveNameWithParent
cs.set_param('CPPClassGenCompliant', 'on');   % CPPClassGenCompliant
cs.set_param('CompOptLevelCompliant', 'on');   % CompOptLevelCompliant
cs.set_param('ConcurrentExecutionCompliant', 'on');   % ConcurrentExecutionCompliant
cs.set_param('ERTCustomFileBanners', 'on');   % ERTCustomFileBanners
cs.set_param('ERTFirstTimeCompliant', 'on');   % ERTFirstTimeCompliant
cs.set_param('GenerateFullHeader', 'on');   % GenerateFullHeader
cs.set_param('InferredTypesCompatibility', 'off');   % InferredTypesCompatibility
cs.set_param('ModelReferenceCompliant', 'on');   % ModelReferenceCompliant
cs.set_param('ModelStepFunctionPrototypeControlCompliant', 'on');   % ModelStepFunctionPrototypeControlCompliant
cs.set_param('MultiwordLength', 2048);   % MultiwordLength
cs.set_param('ParMdlRefBuildCompliant', 'on');   % ParMdlRefBuildCompliant
cs.set_param('TargetFcnLib', 'ansi_tfl_table_tmw.mat');   % TargetFcnLib
cs.set_param('TargetLibSuffix', '');   % TargetLibSuffix
cs.set_param('TargetPreCompLibLocation', '');   % TargetPreCompLibLocation
cs.set_param('UseToolchainInfoCompliant', 'on');   % UseToolchainInfoCompliant
cs.set_param('RTWCAPIParams', 'off');   % parameters
cs.set_param('RTWCAPIRootIO', 'off');   % root-level I/O
cs.set_param('RTWCAPISignals', 'off');   % signals
cs.set_param('RTWCAPIStates', 'off');   % states
cs.set_param('RateGroupingCode', 'on');   % RateGroupingCode
cs.set_param('HoistShortCircuitFcnCalls', 'off');   % Hoist function calls inside short circuits
cs.set_param('BooleanTrueId', 'true');   % Boolean true identifier.
cs.set_param('BooleanFalseId', 'false');   % Boolean false identifier.
cs.set_param('MaxIdInt32', 'MAX_int32_T');   % 32-bit integer maximum identifier
cs.set_param('MinIdInt32', 'MIN_int32_T');   % 32-bit integer minimum identifier
cs.set_param('MaxIdUint32', 'MAX_uint32_T');   % 32-bit unsigned integer maximum identifier
cs.set_param('MaxIdInt16', 'MAX_int16_T');   % 16-bit integer maximum identifier
cs.set_param('MinIdInt16', 'MIN_int16_T');   % 16-bit integer minimum identifier
cs.set_param('MaxIdUint16', 'MAX_uint16_T');   % 16-bit unsigned integer maximum identifier
cs.set_param('MaxIdInt8', 'MAX_int8_T');   % 8-bit integer maximum identifier
cs.set_param('MinIdInt8', 'MIN_int8_T');   % 8-bit integer minimum identifier
cs.set_param('MaxIdUint8', 'MAX_uint8_T');   % 8-bit unsigned integer maximum identifier
cs.set_param('TypeLimitIdReplacementHeaderFile', '');   % Type limit identifier replacement header file
cs.set_param('RecordCoverage', 'off');   % Record coverage for this model
cs.set_param('CovModelRefEnable', 'off');   % Record coverage for referenced models
cs.set_param('CovExternalEMLEnable', 'off');   % Record coverage for MATLAB files
cs.set_param('CovEnableCumulative', 'on');   % Enable cumulative data collection
cs.set_param('CovSaveCumulativeToWorkspaceVar', 'on');   % Save cumulative coverage results in workspace variable
cs.set_param('CovCumulativeVarName', 'covCumulativeData');   % Cumulative coverage variable name
cs.set_param('CovSaveSingleToWorkspaceVar', 'on');   % Save last coverage run in workspace variable
cs.set_param('CovSaveName', 'covdata');   % Last coverage run variable name
cs.set_param('CovNameIncrementing', 'off');   % Increment cvdata variable name with each simulation
cs.set_param('CovReportOnPause', 'on');   % Update coverage results on pause
cs.set_param('CovHtmlReporting', 'on');   % Generate coverage report
cs.set_param('CovHTMLOptions', '');   % Coverage report options
cs.set_param('CovCumulativeReport', 'off');   % Include cumulative data in coverage report
cs.set_param('CovCompData', '');   % Additional data to include in coverage report
cs.set_param('CovForceBlockReductionOff', 'on');   % Force block reduction off
cs.set_param('CovUseTimeInterval', 'off');   % Restrict coverage recording interval
cs.set_param('CovBoundaryAbsTol', 1E-05);   % Relational boundary coverage absolute tolerance
cs.set_param('CovBoundaryRelTol', 0.01);   % Relational boundary coverage relative tolerance
cs.set_param('CovFilter', '');   % Coverage filter filename
