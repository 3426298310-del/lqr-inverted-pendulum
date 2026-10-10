function dx = cartpole_nonlinear(x,u)
%CARTPOLE_NONLINEAR Lagrange equations with xc=x-l*sin(theta), yc=l*cos(theta).
% Additional consistency check, not the thesis's linear State-Space plant.
[~,~,p]=cartpole_model();
v=x(2); theta=x(3); omega=x(4);
mass=[p.M+p.m, -p.m*p.l*cos(theta);...
      -p.m*p.l*cos(theta), p.I+p.m*p.l^2];
rhs=[u-p.b*v-p.m*p.l*sin(theta)*omega^2; p.m*p.g*p.l*sin(theta)];
acc=mass\rhs;
dx=[v;acc(1);omega;acc(2)];
end
