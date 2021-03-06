function out1 = g_dynamics_JSIM(in1)
%G_DYNAMICS_JSIM
%    OUT1 = G_DYNAMICS_JSIM(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    06-Aug-2020 16:07:38

q1 = in1(1,:);
q2 = in1(2,:);
t2 = 3.762312314918354e16;
t3 = atan(1.660044257358011e-1);
t4 = -q1+q2+t3;
t5 = cos(t4);
t6 = t2.*t5.*1.998401444325282e-16;
out1 = reshape([8.6e1./5.0,t6,t6,6.03421313034146],[2,2]);
