function out1 = g_dynamics_Linearization_RHS_A(in1,in2)
%G_DYNAMICS_LINEARIZATION_RHS_A
%    OUT1 = G_DYNAMICS_LINEARIZATION_RHS_A(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    19-Aug-2020 19:11:27

s1 = in1(1,:);
s2 = in1(2,:);
out1 = reshape([0.0,0.0,cos(s1).*2.94e2,0.0,0.0,0.0,0.0,cos(s2).*1.211460738977886e2-sin(s2).*2.011078442754932e1,1.0,0.0,-1.0,0.0,0.0,1.0,0.0,-1.0],[4,4]);
