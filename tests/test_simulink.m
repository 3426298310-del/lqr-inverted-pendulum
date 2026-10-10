function tests = test_simulink
tests=functiontests(localfunctions);
end

function testGeneratedModelsMatchIndependentIntegration(t)
root=fileparts(fileparts(mfilename('fullpath'))); addpath(fullfile(root,'src'));
folder=tempname; mkdir(folder);
cleanup=onCleanup(@() cleanupModels(folder)); %#ok<NASGU>
build_models(folder);
names={'cartpole_open','cartpole_lqr','cartpole_pulse'};
scenarios={'open','initial','pulse'};
for i=1:3
 file=fullfile(folder,[names{i} '.slx']); verifyTrue(t,isfile(file));
 load_system(file); out=sim(names{i});
 verifyEqual(t,get_param(names{i},'Creator'),'WANG Zheng');
 verifyEqual(t,get_param(names{i},'LastModifiedBy'),'WANG Zheng');
 truth=simulate_case('A',scenarios{i},.001);
 log=out.x_log; reference=interp1(truth.t,truth.x,log.Time,'pchip');
 % Relative scaling for divergent open-loop; absolute for closed-loop.
 error=max(abs(log.Data-reference),[],'all')/max(1,max(abs(reference),[],'all'));
 verifyLessThan(t,error,2e-6);
 verifyEqual(t,log.Time(end),truth.t(end),'AbsTol',1e-12);
 close_system(names{i},0);
end
end

function cleanupModels(folder)
for name={'cartpole_open','cartpole_lqr','cartpole_pulse'}
 if bdIsLoaded(name{1}), close_system(name{1},0); end
end
if isfolder(folder), rmdir(folder,'s'); end
end
