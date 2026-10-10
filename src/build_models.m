function build_models(folder)
%BUILD_MODELS Generate standalone, self-contained R2024b Simulink models.
% Values live in each Model Workspace, not the user's MATLAB base workspace.
if nargin<1
 folder=fullfile(fileparts(fileparts(mfilename('fullpath'))),'models');
end
if ~isfolder(folder), mkdir(folder); end
[A,B,p]=cartpole_model(); K=design_lqr('A'); %#ok<ASGLU>
names={'cartpole_open','cartpole_lqr','cartpole_pulse'};
load_system('simulink');
for i=1:3
 name=names{i};
 assert(~bdIsLoaded(name),'Close the model %s before rebuilding it.',name);
 new_system(name); cleanup=onCleanup(@() close_system(name,0));
 set_param(name,'Creator','WANG Zheng','ModifiedByFormat','WANG Zheng');
 ws=get_param(name,'ModelWorkspace');
 ws.assignin('A',A); ws.assignin('B',B); ws.assignin('K',K);
 ws.assignin('x0',[0;0;deg2rad(5);0]); ws.assignin('pulse_amplitude',0);
 stop='8';
 if i==1, ws.assignin('K',zeros(1,4)); stop='2'; end
 if i==3, ws.assignin('x0',zeros(4,1)); ws.assignin('pulse_amplitude',2); end
 set_param(name,'Solver','ode45','StopTime',stop,'RelTol','1e-10',...
  'AbsTol','1e-12','MaxStep','0.001','ReturnWorkspaceOutputs','on',...
  'OutputOption','SpecifiedOutputTimes','OutputTimes',['[0:0.001:' stop ']']);
 add_block('simulink/Sources/Step',[name '/Pulse start'],...
  'Time','1','Before','0','After','pulse_amplitude','Position',[40 50 80 80]);
 add_block('simulink/Sources/Step',[name '/Pulse end'],...
  'Time','1.2','Before','0','After','-pulse_amplitude','Position',[40 110 80 140]);
 add_block('simulink/Math Operations/Sum',[name '/Disturbance'],...
  'Inputs','++','Position',[130 65 150 125]);
 add_block('simulink/Math Operations/Sum',[name '/Plant input'],...
  'Inputs','++','Position',[210 180 240 220]);
 add_block('simulink/Continuous/State-Space',[name '/Cart pole'],...
  'A','A','B','B','C','eye(4)','D','zeros(4,1)','X0','x0','Position',[300 175 460 225]);
 add_block('simulink/Math Operations/Gain',[name '/State feedback'],...
  'Gain','-K','Multiplication','Matrix(K*u)','Orientation','left','Position',[320 300 425 350]);
 add_block('simulink/Sinks/To Workspace',[name '/State log'],...
  'VariableName','x_log','SaveFormat','Timeseries','Position',[550 170 650 210]);
 add_block('simulink/Sinks/To Workspace',[name '/Control log'],...
  'VariableName','u_log','SaveFormat','Timeseries','Position',[60 295 165 335]);
 add_block('simulink/Sinks/Scope',[name '/States'],...
  'Position',[550 80 590 120]);
 add_line(name,'Pulse start/1','Disturbance/1','autorouting','on');
 add_line(name,'Pulse end/1','Disturbance/2','autorouting','on');
 add_line(name,'Disturbance/1','Plant input/1','autorouting','on');
 add_line(name,'State feedback/1','Plant input/2','autorouting','on');
 add_line(name,'Plant input/1','Cart pole/1','autorouting','on');
 add_line(name,'Cart pole/1','State feedback/1','autorouting','on');
 add_line(name,'Cart pole/1','State log/1','autorouting','on');
 add_line(name,'Cart pole/1','States/1','autorouting','on');
 add_line(name,'State feedback/1','Control log/1','autorouting','on');
 set_param(name,'Description',['Undergraduate thesis reimplementation. ' ...
  'State=[x v theta omega], SI units; xc=x-l*sin(theta). ' ...
  'Model Workspace contains exact Table 1 parameters and baseline A controller.']);
 save_system(name,fullfile(folder,[name '.slx']));
 clear cleanup
end
end
