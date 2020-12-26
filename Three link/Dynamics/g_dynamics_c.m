function c = g_dynamics_c(in1,in2)
%G_DYNAMICS_C
%    C = G_DYNAMICS_C(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    30-Nov-2020 19:14:20

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
v1 = in2(1,:);
v2 = in2(2,:);
v3 = in2(3,:);
t2 = q2+q3;
t3 = sin(t2);
t4 = sin(q3);
t5 = t3.*(5.0./2.0);
t6 = t3.*(5.0./4.0);
t7 = t4.*(5.0./2.0);
t8 = sin(q2);
t9 = q1+q2+q3;
t10 = sin(t9);
t11 = t10.*(4.9e1./2.0);
t12 = q1+q2;
t13 = sin(t12);
t14 = t13.*(1.47e2./2.0);
t15 = v1.^2;
t16 = t3.*t15.*(5.0./4.0);
c = [t11+t14-v1+sin(q1).*(2.45e2./2.0)-v1.*(v3.*(t5+t7)+v2.*(t5+t8.*(1.5e1./2.0)))-v2.*(v3.*(t6+t7)+v2.*(t6+t8.*(1.5e1./4.0)))-v3.*(t3.*v2.*(5.0./4.0)+v3.*(t4.*(5.0./4.0)+t6));t11+t14+t16-v2+t8.*t15.*(1.5e1./4.0)-t4.*v3.^2.*(5.0./4.0)-t4.*v1.*v3.*(5.0./2.0)-t4.*v2.*v3.*(5.0./2.0);t11+t16-v3+t4.*t15.*(5.0./4.0)+t4.*v2.^2.*(5.0./4.0)+t4.*v1.*v2.*(5.0./2.0)];
