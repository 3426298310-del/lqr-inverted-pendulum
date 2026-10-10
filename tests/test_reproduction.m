function tests = test_reproduction
tests = functiontests(localfunctions);
end

function setupOnce(~)
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))),'src'));
end

function testLinearModelAndControllability(t)
[A,B] = cartpole_model();
verifyEqual(t,A,[0 1 0 0;0 -0.181818181818 2.675454545455 0;0 0 0 1;0 -0.454545454545 31.213636363636 0],'AbsTol',1e-10);
verifyEqual(t,B,[0;1.818181818182;0;4.545454545455],'AbsTol',1e-10);
verifyEqual(t,rank(ctrb(A,B)),4);
verifyGreaterThan(t,max(real(eig(A))),5.5);
end

function testNonlinearJacobianMatchesLinearModel(t)
[A,B] = cartpole_model(); h=1e-6; J=zeros(4);
for j=1:4
 e=zeros(4,1); e(j)=h;
 J(:,j)=(cartpole_nonlinear(e,0)-cartpole_nonlinear(-e,0))/(2*h);
end
G=(cartpole_nonlinear(zeros(4,1),h)-cartpole_nonlinear(zeros(4,1),-h))/(2*h);
verifyEqual(t,J,A,'AbsTol',1e-8); verifyEqual(t,G,B,'AbsTol',1e-8);
end

function testRiccatiAndStableFeedback(t)
[A,B]=cartpole_model();
for id=['A','B','C']
 [K,P,Q,R]=design_lqr(id);
 residual=A'*P+P*A-P*B*(R\B')*P+Q;
 verifyLessThan(t,norm(residual,'fro')/norm(Q,'fro'),1e-10);
 verifyLessThan(t,max(real(eig(A-B*K))),0);
end
[K,~,~,~]=design_lqr('A');
verifyEqual(t,K,[-38.7298 -36.3830 160.9258 29.6787],'AbsTol',5e-5);
end

function testSettlingUsesLastExitNotFirstCrossing(t)
verifyEqual(t,settling_time((0:5)',[2;0;2;0;0;0],1),3);
verifyEqual(t,settling_time((0:2)',[0;0;0],1),0);
verifyTrue(t,isnan(settling_time((0:2)',[2;0;2],1)));
end

function testThesisPeakReproduction(t)
s=simulate_case('A','initial',.001);
verifyEqual(t,max(abs(s.x(:,1))),.0628,'AbsTol',5e-5);
verifyEqual(t,max(abs(s.u)),14.0434,'AbsTol',5e-5);
verifyEqual(t,max(abs(rad2deg(s.x(:,3)))),5,'AbsTol',1e-10);
verifyLessThan(t,norm(s.x(end,:)),1e-5);
verifyEqual(t,settling_time(s.t,rad2deg(s.x(:,3)),.1),2.422,'AbsTol',.0011);
end

function testPulseStartsAtEquilibriumAndMatchesExactSolution(t)
s=simulate_case('A','pulse',.001);
verifyEqual(t,s.x(s.t<=1,:),zeros(sum(s.t<=1),4),'AbsTol',1e-12);
[A,B]=cartpole_model(); K=design_lqr('A'); F=A-B*K;
% Independent analytic rectangular-pulse response, not ode45 again.
for time=[1.1 1.2 1.5 3 8]
 exact=F\((expm(F*(time-1))-eye(4))*B*2);
 if time>=1.2, exact=exact-F\((expm(F*(time-1.2))-eye(4))*B*2); end
 [~,idx]=min(abs(s.t-time));
 verifyEqual(t,s.x(idx,:)',exact,'AbsTol',1e-9);
end
verifyEqual(t,max(abs(rad2deg(s.x(:,3)))),.6232,'AbsTol',5e-4);
verifyEqual(t,s.total_force,s.u+s.disturbance,'AbsTol',1e-12);
verifyEqual(t,max(s.disturbance),2);
end

function testInitialLinearSolutionMatchesMatrixExponential(t)
s=simulate_case('A','initial',.01); [A,B]=cartpole_model(); K=design_lqr('A');
reference=expm((A-B*K)*.5)*[0;0;deg2rad(5);0];
verifyEqual(t,s.x(51,:)',reference,'AbsTol',1e-8);
end
