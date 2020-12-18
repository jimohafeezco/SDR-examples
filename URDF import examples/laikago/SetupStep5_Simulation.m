close all; clear classes;

InitialPosition = SRD_get('InitialPosition');

Handler_State = SRD_get_handler__state('InitialPosition', InitialPosition, ...
    'InitialVelocity', zeros(size(InitialPosition)));

Handler_IK_Solution = SRD_get('Handler_IK_Solution');


Handler_dynamics_Linearized_Model = SRD_get('Handler_dynamics_Linearized_Model');
Handler_dynamics_generalized_coordinates_model= SRD_get('Handler_dynamics_generalized_coordinates_model');


Handler_dynamics_GC_model_evaluator = SRD_get_handler__dynamics_GC_model_evaluator(...
    'Handler_dynamics_generalized_coordinates_model', Handler_dynamics_generalized_coordinates_model, ...
    'Handler_State', Handler_State, ...
    'UsePinv', true);

Handler_Constraints_Model = SRD_get('Handler_Constraints_Model');

Handler_dynamics_Linear_model_evaluator = SRD_get_handler__dynamics_Linear_model_evaluator(...
    'Handler_dynamics_generalized_coordinates_model', Handler_dynamics_GC_model_evaluator, ...
    'Handler_dynamics_Linearized_Model', Handler_dynamics_Linearized_Model, ...
    'Handler_State', Handler_State, ...
    'Handler_Controller', [], ...
    'ToEvaluate_c', false);





dt = 0.001;
tf = Handler_IK_Solution.TimeExpiration;
tf = 1;

Handler_Simulation = SRD_get_handler__Simulation(...
    'TimeLog', 0:dt:tf);


Handler_InverseDynamics = SRD_get_handler__InverseDynamics_Vanilla__desired_trajectory(...
    'Handler_ControlInput', Handler_IK_Solution, ...
    'Handler_dynamics_generalized_coordinates_model', Handler_dynamics_generalized_coordinates_model, ...
    'Handler_Simulation', Handler_Simulation);

MainController = SRD_get_handler__ComputedTorqueController(...
    'Handler_State', Handler_State, ...
    'Handler_ControlInput', Handler_IK_Solution, ...
    'Handler_dynamics_generalized_coordinates_model', Handler_dynamics_generalized_coordinates_model, ...
    'Handler_Simulation', Handler_Simulation, ...
    'Handler_InverseDynamics', Handler_InverseDynamics, ...
    'Kp', 1000*eye(Handler_IK_Solution.dof_robot), ...
    'Kd', 500*eye(Handler_IK_Solution.dof_robot));
% 
% MainController = SRD_get_handler__NoInputController(...
%     'Handler_dynamics_generalized_coordinates_model', Handler_dynamics_generalized_coordinates_model);


% Handler_solver = SRD_get_handler__solver_Taylor(...
%     'Handler_State', Handler_State, ...
%     'Handler_Controller', MainController, ...
%     'Handler_dynamics_generalized_coordinates_model', Handler_dynamics_generalized_coordinates_model, ...
%     'Handler_Simulation', Handler_Simulation);

% Handler_solver = SRD_get_handler__solver_ImplicitTaylor(...
%     'Handler_State', Handler_State, ...
%     'Handler_Controller', MainController, ...
%     'Handler_dynamics_generalized_coordinates_model', Handler_dynamics_generalized_coordinates_model, ...
%     'Handler_Simulation', Handler_Simulation);

Handler_solver = SRD_get_handler__solver_Taylor_test(...
    'Handler_State', Handler_State, ...
    'Handler_Controller', MainController, ...
    'Handler_dynamics_generalized_coordinates_model', Handler_dynamics_generalized_coordinates_model, ...
    'Handler_Simulation', Handler_Simulation);

% Handler_solver = SRD_get_handler__solver_ODE(...
%     'Handler_State', Handler_State, ...
%     'Handler_Controller', MainController, ...
%     'Handler_dynamics_generalized_coordinates_model', Handler_dynamics_generalized_coordinates_model, ...
%     'Handler_Simulation', Handler_Simulation);



Handler_State_Logger_vanilla = SRD_get_handler__State_Logger__vanilla(...
    'Handler_State', Handler_State, ...
    'Handler_Simulation', Handler_Simulation, ...
    'ToLogAcceleration',  false);

Handler_SimulationTickDisplay = SRD_get_handler__SimulationTickDisplay(...
    'Handler_Simulation', Handler_Simulation);




Handler_Simulation.ControllerArray = {Handler_InverseDynamics; MainController};
Handler_Simulation.SolverArray = {Handler_solver};
Handler_Simulation.LoggerArray = {Handler_State_Logger_vanilla, Handler_SimulationTickDisplay};

Handler_Simulation.Simulate();


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


figure('Color', 'w', 'Name', 'Positions');

subplot(1, 2, 1)
SRDgraphic_PlotGeneric(Handler_Simulation.TimeLog, Handler_State_Logger_vanilla.Log.q, ...
    'NewFigure', false, 'FigureName', 'Generic', ...
    'LableVariable', 'q', 'Title', []);

subplot(1, 2, 2)
SRDgraphic_PlotGeneric(Handler_Simulation.TimeLog, Handler_State_Logger_vanilla.Log.v, ...
    'NewFigure', false, 'FigureName', 'Generic', ...
    'LableVariable', 'v', 'Title', []);

drawnow;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% 
% DrawRobot_function = SRD_DrawRobot_get_function('DrawRobot_Type', 'STL', ... %'Default' or 'STL' or 'Custom'
%     'DrawRobot_Custom_handle', [], ...
%     'Function_Type', 'DrawCurrentPosition', ... %'DrawGivenPosition' or 'DrawInitialPosition'  or 'DrawCurrentPosition'
%     'FileName_visuals_config', 'datafile_visuals_config.mat'); %use default visuals
% % 
% DrawRobot_function = SRD_DrawRobot_get_function('DrawRobot_Type', 'STL', ... %'Default' or 'STL' or 'Custom'
%     'DrawRobot_Custom_handle', [], ...
%     'Function_Type', 'DrawGivenPosition', ... %'DrawGivenPosition' or 'DrawInitialPosition'  or 'DrawCurrentPosition'
%     'Chain', []);


% SRD__animate__vanilla('Handler_Simulation', Handler_Simulation, ...
%     'Handler_Logger', Handler_State_Logger_vanilla, ...
%     'AnimationTimeLog', 0:10*dt:(tf-dt), ...
%     'DrawRobot_function', DrawRobot_function, ...
%     'NewFigure', true, ...
%     'FigureName', 'Animation', ...
%     'FileName_visuals_config', []);


ToAnimate = true;
LinkArray = SRD_get('LinkArray');
Chain = SRD_Chain(LinkArray);
 
SymbolicEngine = SRDSymbolicEngine('LinkArray', LinkArray, 'Casadi', true);
SymbolicEngine.InitializeLinkArray();
 
if ToAnimate
SRD__make_default_scene('STL')
DrawRobot_function = SRD_DrawRobot_get_function('DrawRobot_Type', 'STL', ... %'Default' or 'STL' or 'Custom'
    'DrawRobot_Custom_handle', [], ...
    'Function_Type', 'DrawGivenPosition', ... %'DrawGivenPosition' or 'DrawInitialPosition'  or 'DrawCurrentPosition'
    'Chain', Chain ...
    ); %use default visuals
 
SRD__animate__vanilla('Handler_Simulation', Handler_Simulation, ...
    'Handler_Logger', Handler_State_Logger_vanilla, ...
    'AnimationTimeLog', 0:10*dt:tf, ...
    'DrawRobot_function', DrawRobot_function, ...
    'NewFigure', true, ...
    'FigureName', 'Animation', ...
    'FileName_visuals_config', []);
end
