function [A,B,p] = cartpole_model()
%CARTPOLE_MODEL Exact parameters from thesis Table 1, state [x v theta omega].
% Angle convention agrees with numeric thesis Eq.33/34: xc = x-l*sin(theta).
p=struct('M',.50,'m',.20,'b',.10,'l',.30,'I',.006,'g',9.81);
d=p.I*(p.M+p.m)+p.M*p.m*p.l^2;
A=[0 1 0 0;
   0 -(p.I+p.m*p.l^2)*p.b/d p.m^2*p.g*p.l^2/d 0;
   0 0 0 1;
   0 -p.m*p.l*p.b/d p.m*p.g*p.l*(p.M+p.m)/d 0];
B=[0;(p.I+p.m*p.l^2)/d;0;p.m*p.l/d];
end
