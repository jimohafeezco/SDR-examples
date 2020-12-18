function TaskJacobian = g_Constraint_Jacobian(in1)
%G_CONSTRAINT_JACOBIAN
%    TASKJACOBIAN = G_CONSTRAINT_JACOBIAN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    18-Dec-2020 12:44:55

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = q1+q2+q3;
t3 = cos(t2);
t4 = q1+q2;
t5 = cos(t4);
t6 = sin(t2);
t7 = sin(t4);
TaskJacobian = reshape([0.0,t3.*(-1.0./2.0)-t5.*(1.0./2.0)-cos(q1).*(1.0./2.0),t6.*(-1.0./2.0)-t7.*(1.0./2.0)-sin(q1).*(1.0./2.0),0.0,t3.*(-1.0./2.0)-t5.*(1.0./2.0),t6.*(-1.0./2.0)-t7.*(1.0./2.0),0.0,t3.*(-1.0./2.0),t6.*(-1.0./2.0)],[3,3]);
