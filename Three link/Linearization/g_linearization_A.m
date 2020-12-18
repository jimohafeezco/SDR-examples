function A = g_linearization_A(in1,in2,in3,in4)
%G_LINEARIZATION_A
%    A = G_LINEARIZATION_A(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    18-Dec-2020 12:44:40

iH1_1 = in4(1);
iH1_2 = in4(4);
iH1_3 = in4(7);
iH2_1 = in4(2);
iH2_2 = in4(5);
iH2_3 = in4(8);
iH3_1 = in4(3);
iH3_2 = in4(6);
iH3_3 = in4(9);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
u1 = in3(1,:);
u2 = in3(2,:);
u3 = in3(3,:);
v1 = in2(1,:);
v2 = in2(2,:);
v3 = in2(3,:);
t2 = q1+q2+q3;
t3 = sin(q3);
t4 = v1.^2;
t5 = q2+q3;
t6 = sin(t5);
t7 = sin(t2);
t8 = t7.*(4.9e1./2.0);
t9 = q1+q2;
t10 = t4.*t6.*(5.0./4.0);
t11 = sin(t9);
t12 = t11.*(1.47e2./2.0);
t13 = v2.^2;
t14 = sin(q2);
t15 = v3.^2;
t16 = t3.*t4.*(5.0./4.0);
t17 = t3.*t13.*(5.0./4.0);
t18 = t3.*v1.*v2.*(5.0./2.0);
t19 = t8+t10+t16+t17+t18+u3-v3;
t20 = t4.*t14.*(1.5e1./4.0);
t21 = t3.*t15.*(5.0./4.0);
t22 = t3.*v1.*v3.*(5.0./2.0);
t23 = t3.*v2.*v3.*(5.0./2.0);
t24 = sin(q1);
t25 = t13.*t14.*(1.5e1./4.0);
t26 = t6.*t13.*(5.0./4.0);
t27 = t6.*t15.*(5.0./4.0);
t28 = t6.*v1.*v2.*(5.0./2.0);
t29 = t6.*v1.*v3.*(5.0./2.0);
t30 = t6.*v2.*v3.*(5.0./2.0);
t31 = t14.*v1.*v2.*(1.5e1./2.0);
t35 = t24.*(2.45e2./2.0);
t32 = -t8-t12+t21+t22+t23+t25+t26+t27+t28+t29+t30+t31-t35-u1+v1;
t33 = iH2_3.*t19;
t34 = t8+t10+t12+t20-t21-t22-t23+u2-v2;
t36 = t6.*(5.0./2.0);
t37 = iH3_3.*t19;
t38 = iH3_2.*t34;
t44 = iH3_1.*t32;
t39 = t37+t38-t44;
t40 = t6.*(5.0./4.0);
t41 = iH2_2.*t34;
t51 = iH2_1.*t32;
t42 = t33+t41-t51;
t43 = t3.*(5.0./2.0);
t45 = cos(t2);
t46 = t45.*(4.9e1./2.0);
t47 = cos(t9);
t48 = t47.*(1.47e2./2.0);
t49 = t14.*(1.5e1./4.0);
t50 = t40+t49;
t52 = t42.*t50;
t53 = t40+t43;
t54 = t39.*t53;
t55 = t52+t54;
t56 = cos(t5);
t57 = cos(q2);
t58 = cos(q3);
t59 = t58.*(5.0./2.0);
t60 = t56.*(5.0./4.0);
t61 = t56.*v2.*(5.0./4.0);
t62 = t3.*(5.0./4.0);
t63 = t40+t62;
t64 = t36+t43;
t65 = t14.*(1.5e1./2.0);
t66 = t36+t65;
t67 = t6.*v2.*(5.0./4.0);
t68 = t63.*v3;
t69 = t67+t68;
t70 = t64.*v3;
t71 = t66.*v2;
t72 = t70+t71;
t73 = t53.*v3;
t74 = t50.*v2;
t75 = t73+t74;
t77 = t69.*v3;
t78 = t72.*v1;
t79 = t75.*v2;
t76 = t8+t12+t35-t77-t78-t79+u1-v1;
t80 = iH3_1.*t76;
t81 = t37+t38+t80;
t82 = t6.*v1.*(5.0./2.0);
t83 = t14.*v1.*(1.5e1./2.0);
t84 = t3.*v3.*(5.0./2.0);
t85 = v1+v2+v3;
t86 = t39.*t63;
t87 = t42.*t66;
t88 = t39.*t64;
t89 = t87+t88;
t90 = t6.*t42.*(5.0./4.0);
t91 = t86+t90;
t92 = t4.*t56.*(5.0./4.0);
t93 = t3.*v1.*(5.0./2.0);
t94 = t3.*v2.*(5.0./2.0);
t95 = t82+t93+t94;
t96 = t70+t71+1.0;
t97 = t84+1.0;
t98 = t6.*v2.*(5.0./2.0);
t99 = t6.*v3.*(5.0./2.0);
t100 = t14.*v2.*(1.5e1./2.0);
t101 = t82+t83+t84+t98+t99+t100;
t102 = v1+v2;
t103 = t3+t6;
t104 = t82+t83-t84;
A = reshape([0.0,0.0,0.0,t46+t48+cos(q1).*(2.45e2./2.0)+iH1_2.*t55+iH1_1.*t89+iH1_3.*(t86+t6.*(t33+iH2_2.*(t8+t10+t12+t20+u2-v2-t3.*t15.*(5.0./4.0)-t3.*v1.*v3.*(5.0./2.0)-t3.*v2.*v3.*(5.0./2.0))-iH2_1.*t32).*(5.0./4.0)),t46+t48+iH2_2.*t55+iH2_1.*t89+iH2_3.*t91,t46+iH3_2.*t55+iH3_1.*t89+iH3_3.*t91,0.0,0.0,0.0,t46+t48+iH1_1.*t55-t13.*t56.*(5.0./4.0)-t13.*t57.*(1.5e1./4.0)-t15.*t56.*(5.0./4.0)+iH1_2.*t3.*t39.*(5.0./2.0)+iH1_3.*t3.*t39.*(5.0./4.0)-t56.*v1.*v2.*(5.0./2.0)-t56.*v1.*v3.*(5.0./2.0)-t57.*v1.*v2.*(1.5e1./2.0)-t56.*v2.*v3.*(5.0./2.0),t46+t48+t92+iH2_1.*t55+t4.*t57.*(1.5e1./4.0)+iH2_2.*t3.*t39.*(5.0./2.0)+iH2_3.*t3.*t39.*(5.0./4.0),t46+t92+iH3_1.*t55+iH3_2.*t3.*t39.*(5.0./2.0)+iH3_3.*t3.*t39.*(5.0./4.0),0.0,0.0,0.0,t46-v3.*(t61+v3.*(t58.*(5.0./4.0)+t60))-v2.*(t61+v3.*(t59+t60))+iH1_1.*(t63.*t81+t6.*(t33+t41+iH2_1.*t76).*(5.0./4.0))-v1.*(t56.*v2.*(5.0./2.0)+v3.*(t56.*(5.0./2.0)+t59))+iH1_2.*t3.*t81.*(5.0./4.0),t46+t92+iH2_1.*t91-t15.*t58.*(5.0./4.0)+iH2_2.*t3.*t39.*(5.0./4.0)-t58.*v1.*v3.*(5.0./2.0)-t58.*v2.*v3.*(5.0./2.0),t46+t92+iH3_1.*t91+t4.*t58.*(5.0./4.0)+t13.*t58.*(5.0./4.0)+iH3_2.*t3.*t39.*(5.0./4.0)+t58.*v1.*v2.*(5.0./2.0),1.0,0.0,0.0,-iH1_1.*t96+iH1_3.*t95+iH1_2.*(t82+t83-t3.*v3.*(5.0./2.0)),-iH2_1.*t96+iH2_3.*t95+iH2_2.*t104,-iH3_1.*t96+iH3_3.*t95+iH3_2.*t104,0.0,1.0,0.0,-iH1_2.*t97-iH1_1.*t101+iH1_3.*t3.*t102.*(5.0./2.0),-iH2_2.*t97-iH2_1.*t101+iH2_3.*t3.*t102.*(5.0./2.0),-iH3_2.*t97-iH3_1.*t101+iH3_3.*t3.*t102.*(5.0./2.0),0.0,0.0,1.0,-iH1_3-iH1_2.*t3.*t85.*(5.0./2.0)-iH1_1.*t85.*t103.*(5.0./2.0),-iH2_3-iH2_2.*t3.*t85.*(5.0./2.0)-iH2_1.*t85.*t103.*(5.0./2.0),-iH3_3-iH3_2.*t3.*t85.*(5.0./2.0)-iH3_1.*t85.*t103.*(5.0./2.0)],[6,6]);
