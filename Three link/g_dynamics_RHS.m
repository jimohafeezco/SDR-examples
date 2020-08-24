function out1 = g_dynamics_RHS(in1,in2,in3)
%G_DYNAMICS_RHS
%    OUT1 = G_DYNAMICS_RHS(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    19-Aug-2020 19:11:25

q1 = in1(1,:);
q2 = in1(2,:);
u1 = in3(1,:);
u2 = in3(2,:);
v1 = in2(1,:);
v2 = in2(2,:);
t2 = 3.762312314918354e16;
t3 = atan(1.660044257358011e-1);
t4 = -q1+q2+t3;
t5 = sin(t4);
t6 = v1-v2;
out1 = [u1-u2-v1+sin(q1).*2.94e2-t2.*t5.*t6.*v2.*9.992007221626409e-17;u2-v2+cos(q2).*2.011078442754932e1+sin(q2).*1.211460738977886e2-t2.*t5.*t6.*v1.*9.992007221626409e-17];
