function out1 = g_dynamics_Linearization_SSIM(in1)
%G_DYNAMICS_LINEARIZATION_SSIM
%    OUT1 = G_DYNAMICS_LINEARIZATION_SSIM(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    06-Aug-2020 16:07:39

s1 = in1(1,:);
s2 = in1(2,:);
s3 = in1(3,:);
s4 = in1(4,:);
t2 = 3.762312314918354e16;
t3 = atan(1.660044257358011e-1);
t4 = -s1+s2+t3;
t5 = sin(t4);
t6 = s3-s4;
t7 = t2.*t5.*t6.*1.998401444325282e-16;
t8 = cos(t4);
t9 = t2.*t8.*1.998401444325282e-16;
out1 = reshape([1.0,0.0,0.0,t7,0.0,1.0,t7,0.0,0.0,0.0,8.6e1./5.0,t9,0.0,0.0,t9,6.03421313034146],[4,4]);