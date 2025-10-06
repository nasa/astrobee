import matlab.unittest.TestSuite;

[root_path,~,~] = fileparts(which('runtest_daqp.m'));
addpath(root_path)
addpath(fullfile(root_path,'utils'));
test_path = fullfile(root_path, 'test');

suite= TestSuite.fromFolder(test_path);
result = run(suite);
