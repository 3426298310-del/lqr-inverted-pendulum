function report = run_reproduction()
%RUN_REPRODUCTION Regenerate all numerical outputs, figures and SLX models.
% Run from the repository root: report = run_reproduction;
root=fileparts(mfilename('fullpath')); addpath(fullfile(root,'src'));
out=fullfile(root,'results'); figs=fullfile(out,'figures');
if ~isfolder(figs), mkdir(figs); end
cases={'A','B','C'}; data=cell(1,3); metrics=struct([]);
for i=1:3
 data{i}=simulate_case(cases{i},'initial',.001);
 if i==1, metrics=measure(data{i});
 else, metrics(i)=measure(data{i}); %#ok<AGROW>
 end
 savecsv(data{i},fullfile(out,['initial_' cases{i} '.csv']));
end
pulse=simulate_case('A','pulse',.001); open=simulate_case('A','open',.001);
nonlinear=simulate_case('A','initial',.001,'nonlinear');
savecsv(pulse,fullfile(out,'pulse_A.csv')); savecsv(open,fullfile(out,'open_loop.csv'));
savecsv(nonlinear,fullfile(out,'nonlinear_A_extension.csv'));
[A,B,p]=cartpole_model(); [K,P,Q,R]=design_lqr('A');
report=struct('generated_at',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ss''Z''')),...
 'matlab_version',version,'parameters',p,'state_order',{{'x_m','v_m_s','theta_rad','omega_rad_s'}},...
 'angle_convention','xc=x-l*sin(theta); theta=0 upright',...
 'sample_interval_s',.001,'solver','ode45, RelTol=1e-10, AbsTol=1e-12, MaxStep=0.01; pulse split at 1 and 1.2 s',...
 'A',A,'B',B,'Q',Q,'R',R,'K',K,'CARE_relative_residual',norm(A'*P+P*A-P*B*(R\B')*P+Q,'fro')/norm(Q,'fro'),...
 'open_poles',eig(A),'closed_poles_real_imag',[real(eig(A-B*K)),imag(eig(A-B*K))],...
 'initial',metrics,'pulse',measure(pulse));
mask=pulse.t>=1.2;
report.pulse.angle_recovery_after_pulse_end_s=settling_time(pulse.t(mask),rad2deg(pulse.x(mask,3)),.1)-1.2;
report.pulse.cart_recovery_after_pulse_end_s=settling_time(pulse.t(mask),pulse.x(mask,1),.002)-1.2;
report.nonlinear_extension=measure(nonlinear);
coarse=simulate_case('A','initial',.01);
report.initial_A_angle_settling_at_10ms_s=settling_time(coarse.t,rad2deg(coarse.x(:,3)),.1);
fine=simulate_case('A','pulse',.0005);
report.pulse_half_sample_peak_difference_m=abs(max(abs(fine.x(:,1)))-report.pulse.cart_peak_m);
% Independent exact solution across the complete pulse trajectory.
F=A-B*K; exact=zeros(size(pulse.x));
for j=1:numel(pulse.t)
 time=pulse.t(j); z=zeros(4,1);
 if time>=1, z=F\((expm(F*(time-1))-eye(4))*B*2); end
 if time>=1.2, z=z-F\((expm(F*(time-1.2))-eye(4))*B*2); end
 exact(j,:)=z';
end
report.pulse_analytic_max_abs_state_error=max(abs(exact-pulse.x),[],'all');
assert(report.pulse_analytic_max_abs_state_error<1e-9,'Analytic pulse cross-check failed.');

plotresponse(data{1},'Baseline A | 5 deg initial angle',fullfile(figs,'baseline_A.png'));
plotresponse(pulse,'Baseline A | 2 N pulse at 1.0-1.2 s',fullfile(figs,'pulse_A.png'));
f=figure('Visible','off','Color','w','Position',[100 100 1200 780]); tiledlayout(3,1,'TileSpacing','compact');
labels={'Cart position (m)','Pole angle (deg)','Control force (N)'};
for j=1:3
 nexttile; hold on;
 for i=1:3
  values={data{i}.x(:,1),rad2deg(data{i}.x(:,3)),data{i}.u};
  plot(data{i}.t,values{j},'LineWidth',1.4);
 end
 grid on; ylabel(labels{j}); xlim([0 8]);
 if j==1, title('LQR weight comparison | identical plant and initial state'); legend('A: baseline','B: larger R','C: larger Q'); end
end
xlabel('Time (s)'); exportgraphics(f,fullfile(figs,'weight_comparison.png'),'Resolution',150); close(f);
f=figure('Visible','off','Color','w','Position',[100 100 1100 470]); tiledlayout(1,2);
nexttile; semilogy(open.t,max(abs(rad2deg(open.x(:,3))),1e-8),'LineWidth',1.5); grid on;
xlabel('Time (s)'); ylabel('|Angle| (deg, logarithmic)'); title('Uncontrolled linear model');
subtitle('Large angles are outside the linearization validity range');
nexttile; plot(data{1}.t,rad2deg(data{1}.x(:,3)),'LineWidth',1.5); grid on;
xlabel('Time (s)'); ylabel('Angle (deg)'); title('LQR baseline A');
exportgraphics(f,fullfile(figs,'open_vs_closed.png'),'Resolution',150); close(f);

build_models(fullfile(root,'models'));
names={'cartpole_open','cartpole_lqr','cartpole_pulse'}; truth={open,data{1},pulse}; errors=zeros(1,3);
for i=1:3
 load_system(fullfile(root,'models',[names{i} '.slx']));
 simulation=sim(names{i}); log=simulation.x_log;
 reference=interp1(truth{i}.t,truth{i}.x,log.Time,'pchip');
 errors(i)=max(abs(log.Data-reference),[],'all');
 assert(errors(i)/max(1,max(abs(reference),[],'all'))<2e-6,'Simulink cross-check failed.');
 if i==2, print(['-s' names{i}],'-dpng','-r150',fullfile(figs,'simulink_closed_loop.png')); end
 close_system(names{i},0);
end
report.simulink_max_abs_state_error=struct('open',errors(1),'initial_A',errors(2),'pulse_A',errors(3));
fid=fopen(fullfile(out,'metrics.json'),'w','n','UTF-8'); cleanup=onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true)); clear cleanup;
writetable(struct2table(metrics),fullfile(out,'initial_metrics.csv'));
disp(struct2table(metrics)); disp(report.pulse); disp(report.simulink_max_abs_state_error);
fprintf('Results saved in %s\n',out);
end

function m=measure(s)
m=struct('case',s.case,'cart_peak_m',max(abs(s.x(:,1))),...
 'angle_peak_deg',max(abs(rad2deg(s.x(:,3)))),'control_force_peak_N',max(abs(s.u)),...
 'angle_settling_pm_0p1deg_s',settling_time(s.t,rad2deg(s.x(:,3)),.1),...
 'cart_settling_pm_0p002m_s',settling_time(s.t,s.x(:,1),.002));
end

function savecsv(s,path)
T=array2table([s.t,s.x,s.u,s.disturbance,s.total_force],'VariableNames',...
 {'time_s','cart_m','cart_velocity_m_s','angle_rad','angular_velocity_rad_s','control_N','disturbance_N','total_input_N'});
writetable(T,path);
end

function plotresponse(s,label,path)
f=figure('Visible','off','Color','w','Position',[100 100 1200 800]);
tiledlayout(3,1,'TileSpacing','compact'); values={s.x(:,1),rad2deg(s.x(:,3)),s.u};
labels={'Cart position (m)','Pole angle (deg)','Control force (N)'};
for j=1:3
 nexttile; plot(s.t,values{j},'LineWidth',1.6); grid on; hold on; ylabel(labels{j}); xlim([0 8]);
 if j==1, title(label); end
 if j==2, yline(.1,':'); yline(-.1,':'); end
 if j==3 && strcmp(s.scenario,'pulse')
  plot(s.t,s.disturbance,'--','LineWidth',1.2); legend('Feedback force u','External disturbance d');
 end
end
xlabel('Time (s)'); exportgraphics(f,path,'Resolution',150); close(f);
end
