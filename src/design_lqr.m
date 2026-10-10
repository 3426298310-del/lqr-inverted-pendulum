function [K,P,Q,R] = design_lqr(id)
%DESIGN_LQR Thesis Table 3, with u=-K*x. Requires Control System Toolbox.
switch upper(char(id))
 case 'A', Q=diag([120 8 600 12]); R=.08;
 case 'B', Q=diag([120 8 600 12]); R=.20;
 case 'C', Q=diag([150 10 800 16]); R=.08;
 otherwise, error('lqr_reproduction:UnknownCase','Use case A, B or C.');
end
[A,B]=cartpole_model(); [K,P]=lqr(A,B,Q,R);
end
