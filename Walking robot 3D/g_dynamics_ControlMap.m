function out1 = g_dynamics_ControlMap(in1)
%G_DYNAMICS_CONTROLMAP
%    OUT1 = G_DYNAMICS_CONTROLMAP(IN1)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    30-Oct-2017 22:26:50

q4 = in1(4,:);
q5 = in1(5,:);
q6 = in1(6,:);
q7 = in1(7,:);
q8 = in1(8,:);
q9 = in1(9,:);
q10 = in1(10,:);
q11 = in1(11,:);
t2 = cos(q4);
t3 = sin(q6);
t4 = t2.*t3;
t5 = cos(q6);
t6 = sin(q4);
t7 = sin(q5);
t8 = cos(q5);
t9 = sin(q10);
t10 = cos(q10);
t11 = t2.*t5;
t12 = t3.*t6.*t7;
t13 = t11+t12;
t14 = t6.^2;
t15 = t4-t5.*t6.*t7;
t16 = t8.^2;
t17 = t7.^2;
t18 = cos(q7);
t19 = sin(q9);
t20 = t18.*t19;
t21 = cos(q9);
t22 = sin(q7);
t23 = sin(q8);
t24 = cos(q8);
t25 = sin(q11);
t26 = cos(q11);
t27 = t18.*t21;
t28 = t19.*t22.*t23;
t29 = t27+t28;
t30 = t22.^2;
t31 = t20-t21.*t22.*t23;
t32 = t24.^2;
t33 = t23.^2;
t34 = t3.*t6;
t35 = t2.*t5.*t7;
t36 = t34+t35;
t37 = t9.*t36;
t38 = t37-t5.*t8.*t10;
t39 = t5.*t6;
t40 = t39-t2.*t3.*t7;
t41 = t9.*t40;
t42 = t3.*t8.*t10;
t43 = t41+t42;
t44 = t7.*t10;
t45 = t2.*t8.*t9;
t46 = t44+t45;
t47 = t19.*t22;
t48 = t18.*t21.*t23;
t49 = t47+t48;
t50 = t25.*t49;
t51 = t50-t21.*t24.*t26;
t52 = t21.*t22;
t53 = t52-t18.*t19.*t23;
t54 = t25.*t53;
t55 = t19.*t24.*t26;
t56 = t54+t55;
t57 = t23.*t26;
t58 = t18.*t24.*t25;
t59 = t57+t58;
out1 = reshape([0.0,0.0,0.0,1.0,0.0,-t7,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t2,t6.*t8,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t6,t2.*t8,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,-t23,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t18,t22.*t24,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t22,t18.*t24,0.0,0.0,0.0,0.0,0.0,t4-t10.*t15-t5.*t6.*t7+t6.*t8.*t9,-t2.*t13+t8.*t14+t13.*(t38.*(t5.*t7.*t9-t2.*t5.*t8.*t10)-t43.*(t3.*t7.*t9-t2.*t3.*t8.*t10)+t46.*(t8.*t9+t2.*t7.*t10))-t6.*t9.*t15-t8.*t10.*t14,-t2.*t3.*t7-t2.*t6.*t16+t5.*t6.*t17+t2.*t3.*t7.*t10-t6.*t7.*t8.*t9+t2.*t6.*t10.*t16-t5.*t6.*t10.*t17+t2.^2.*t3.*t8.*t9-t2.*t5.*t6.*t7.*t8.*t9,0.0,0.0,0.0,t13.*(t38.^2+t43.^2+t46.^2),0.0,0.0,0.0,0.0,0.0,0.0,0.0,t20-t26.*t31-t21.*t22.*t23+t22.*t24.*t25,-t18.*t29+t24.*t30+t29.*(t51.*(t21.*t23.*t25-t18.*t21.*t24.*t26)-t56.*(t19.*t23.*t25-t18.*t19.*t24.*t26)+t59.*(t24.*t25+t18.*t23.*t26))-t22.*t25.*t31-t24.*t26.*t30,-t18.*t19.*t23-t18.*t22.*t32+t21.*t22.*t33+t18.*t19.*t23.*t26-t22.*t23.*t24.*t25+t18.*t22.*t26.*t32-t21.*t22.*t26.*t33+t18.^2.*t19.*t24.*t25-t18.*t21.*t22.*t23.*t24.*t25,0.0,t29.*(t51.^2+t56.^2+t59.^2)],[11,8]);