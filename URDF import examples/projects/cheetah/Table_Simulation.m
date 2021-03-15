close all; clear classes; clc;

InitialPosition = SRD_get('InitialPosition');

Handler_State = SRD_get_handler__state('InitialPosition', InitialPosition, ...
    'InitialVelocity', zeros(size(InitialPosition)));

Handler_IK_Solution = SRD_get('Handler_IK_Solution');

Handler_Time = SRDHandler_Time();

Handler_Desired_State = SRD_get_handler__desired_state(...
    'Handler_ControlInput', Handler_IK_Solution, ...
    'Handler_Time',   Handler_Time);

% %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Handler_dynamics_generalized_coordinates_model = SRD_get('Handler_dynamics_generalized_coordinates_model');
Handler_dynamics_Linearized_Model = SRD_get('Handler_dynamics_Linearized_Model');
Handler_Constraints_Model = SRD_get('Handler_Constraints_Model');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tf = Handler_IK_Solution.TimeExpiration;
% tf = 0.4;
time_table = 0:0.01:tf;

n = Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;


% %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model


[A_table, B_table, c_table, x_table, u_table, dx_table] = ...
    SRD_LinearModel_GenerateTable('Handler_dynamics_generalized_coordinates_model', Handler_dynamics_generalized_coordinates_model, ...
    'Handler_dynamics_Linearized_Model', Handler_dynamics_Linearized_Model, ...
    'Handler_IK_Solution', Handler_IK_Solution, ...
    'TimeTable', time_table);

% n_constrained = 5;
[N_table, G_table, F_table] = SRD_ConstraintsModel_GenerateTable(...
    'Handler_Constraints_Model', Handler_Constraints_Model, ...
    'Handler_dynamics_generalized_coordinates_model', Handler_dynamics_generalized_coordinates_model, ...
    'x_table', x_table, ...
    'new_dimentions', []);
% n_constrained = size(N_table, 2);

% %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controllers
% 
% Q = 100*eye(2 * n);
% R = 0.01*eye(Handler_dynamics_generalized_coordinates_model.dof_control);
% Count = size(A_table, 3);
% K_table = SRD_LQR_GenerateTable(A_table, B_table, repmat(Q, [1, 1, Count]), repmat(R, [1, 1, Count]));


% K_table = SRD_CTC_GenerateTable('Handler_dynamics_generalized_coordinates_model', Handler_dynamics_generalized_coordinates_model, ...
%     'Handler_IK_Solution', Handler_IK_Solution, ...
%     'Kp', 200*eye(n), 'Kd', 100*eye(n), 'TimeTable', time_table);


Q = 100*eye(2 * n);
R = 0.01*eye(Handler_dynamics_generalized_coordinates_model.dof_control);
Count = size(A_table, 3);
K_table = SRD_CLQR_GenerateTable(A_table, B_table, repmat(Q, [1, 1, Count]), repmat(R, [1, 1, Count]), N_table);


% %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[AA_table, cc_table] = SRD_CloseLoop_GenerateTable(A_table, B_table, c_table, K_table, x_table, u_table);

% %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ode_fnc_handle = SRD_get_ode_fnc_from_ClosedLoopLinearSystem(AA_table, cc_table, time_table);

ode_fnc_handle = SRD_get_ode_fnc_from_ClosedLoopConstrainedLinearSystem...
    (AA_table, cc_table, G_table, F_table, time_table);


x0 = [InitialPosition; zeros(size(InitialPosition))];

[time_table_0, solution_tape] = ode45(ode_fnc_handle, time_table, x0);


error = solution_tape'-x_table;
cost= zeros(n, size(time_table,2));
for i=1: length(time_table)
    cost(:,i)= K_table(:,:,i)* error(:,i); 
end
cost_final = norm(cost,1)+norm(error,1);

% figure('Color', 'w', 'Name', 'Positions');

subplot(1, 2, 1)
SRDgraphic_PlotGeneric(time_table, solution_tape(:,1:n), ...
    'NewFigure', false, 'FigureName', 'Generic', ...
    'LableVariable', 'q', 'Title', ["position"]);

subplot(1, 2, 2)
SRDgraphic_PlotGeneric(time_table, solution_tape(:,n+1:n*2), ...
    'NewFigure', false, 'FigureName', 'Generic', ...
    'LableVariable', 'v', 'Title', ["velocity"]);
drawnow;
% hold off;
% 
% figure('Color', 'w')
% plot(time_table_0, solution_tape(:,n+1:2*n), 'LineWidth', 3); 
% figure('Color', 'w')
% plot(time_table', x_table(n+1:n*2,:)', '--', 'LineWidth', 3);
% 
% % figure('Color', 'w')
% % plot(time_table, cc_table, 'LineWidth', 3); hold on;


ToAnimate = true;
LinkArray = SRD_get('LinkArray');
Chain = SRD_Chain(LinkArray);
%  
SymbolicEngine = SRDSymbolicEngine('LinkArray', LinkArray, 'Casadi', true);
SymbolicEngine.InitializeLinkArray();
 
if ToAnimate
% figure();

SRD__make_default_scene('Default')
DrawRobot_function = SRD_DrawRobot_get_function('DrawRobot_Type', 'STL', ... %'Default' or 'STL' or 'Custom'
    'DrawRobot_Custom_handle', [], ...
    'Function_Type', 'DrawGivenPosition', ... %'DrawGivenPosition' or 'DrawInitialPosition'  or 'DrawCurrentPosition'
    'Chain', Chain ...
    ); %use default visualscheck ur ema
 
SRD__animate__vanilla('TimeLog', time_table, ...
    'PositionLog', solution_tape, ...
    'AnimationTimeLog', 0:10*0.01:tf, ...
    'DrawRobot_function', DrawRobot_function, ...
    'NewFigure', true, ...
    'FigureName', 'Animation', ...
    'FileName_visuals_config', []);
end

