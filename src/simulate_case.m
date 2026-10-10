function s = simulate_case(id,scenario,dt,plant)
%SIMULATE_CASE Independently integrate thesis experiments using ode45.
% Piecewise integration forces both pulse edges to be respected exactly.
if nargin<3, dt=.001; end
if nargin<4, plant='linear'; end
validateattributes(dt,{'numeric'},{'scalar','real','finite','positive','<=',.1});
assert(ismember(plant,{'linear','nonlinear'}),'Unknown plant.');
[A,B]=cartpole_model(); K=design_lqr(id);
switch scenario
 case 'initial', edges=[0 8]; forces=0; initial=[0;0;deg2rad(5);0];
 case 'pulse', edges=[0 1 1.2 8]; forces=[0 2 0]; initial=zeros(4,1);
 case 'open', edges=[0 2]; forces=0; initial=[0;0;deg2rad(5);0]; K=zeros(1,4);
 otherwise, error('lqr_reproduction:UnknownScenario','Use initial, pulse or open.');
end
t=unique([(0:dt:edges(end))';edges']); x=zeros(numel(t),4);
opts=odeset('RelTol',1e-10,'AbsTol',1e-12,'MaxStep',.01);
for k=1:numel(forces)
 d=forces(k);
 if strcmp(plant,'linear'), rhs=@(~,z) (A-B*K)*z+B*d;
 else, rhs=@(~,z) cartpole_nonlinear(z,-K*z+d);
 end
 sol=ode45(rhs,edges(k:k+1),initial,opts);
 mask=t>=edges(k) & t<=edges(k+1);
 x(mask,:)=deval(sol,t(mask))'; initial=deval(sol,edges(k+1));
end
d=zeros(size(t));
if strcmp(scenario,'pulse'), d(t>=1 & t<1.2)=2; end
u=-x*K';
s=struct('t',t,'x',x,'u',u,'disturbance',d,'total_force',u+d,...
 'case',id,'scenario',scenario,'plant',plant);
end
