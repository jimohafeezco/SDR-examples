function TaskJacobian = g_InverseKinematics_TaskJacobian(in1)
%G_INVERSEKINEMATICS_TASKJACOBIAN
%    TASKJACOBIAN = G_INVERSEKINEMATICS_TASKJACOBIAN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    18-Dec-2020 12:44:57

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = q1+q2+q3;
t3 = cos(t2);
t4 = q1+q2;
t5 = cos(t4);
t6 = sin(t2);
t7 = sin(t4);
t8 = cos(q1);
t9 = sin(q1);
TaskJacobian = reshape([0.0,t3.*(-1.0./1.2e1)-t5.*(1.0./4.0)-t8.*(5.0./1.2e1),t6.*(-1.0./1.2e1)-t7.*(1.0./4.0)-t9.*(5.0./1.2e1),0.0,t3.*(-1.0./2.0)-t5.*(1.0./2.0)-t8.*(1.0./2.0),t6.*(-1.0./2.0)-t7.*(1.0./2.0)-t9.*(1.0./2.0),0.0,t3.*(-1.0./1.2e1)-t5.*(1.0./4.0),t6.*(-1.0./1.2e1)-t7.*(1.0./4.0),0.0,t3.*(-1.0./2.0)-t5.*(1.0./2.0),t6.*(-1.0./2.0)-t7.*(1.0./2.0),0.0,t3.*(-1.0./1.2e1),t6.*(-1.0./1.2e1),0.0,t3.*(-1.0./2.0),t6.*(-1.0./2.0)],[6,3]);
