function [out_profile,out_errors] = Inertial_navigation_NED(in_profile,...
    no_epochs,initialization_errors,IMU_errors)
%Inertial_navigation_NED - Simulates inertial navigation using local
%navigation frame (NED) navigation equations and kinematic model
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 3/4/2012 by Paul Groves
%
% Inputs:
%   in_profile   True motion profile array
%   no_epochs    Number of epochs of profile data
%   initialization_errors
%     .delta_r_eb_n     position error resolved along NED (m)
%     .delta_v_eb_n     velocity error resolved along NED (m/s)
%     .delta_eul_nb_n   attitude error as NED Euler angles (rad)
%   IMU_errors
%     .delta_r_eb_n     position error resolved along NED (m)
%     .b_a              Accelerometer biases (m/s^2)
%     .b_g              Gyro biases (rad/s)
%     .M_a              Accelerometer scale factor and cross coupling errors
%     .M_g              Gyro scale factor and cross coupling errors            
%     .G_g              Gyro g-dependent biases (rad-sec/m)             
%     .accel_noise_root_PSD   Accelerometer noise root PSD (m s^-1.5)
%     .gyro_noise_root_PSD    Gyro noise root PSD (rad s^-0.5)
%     .accel_quant_level      Accelerometer quantization level (m/s^2)
%     .gyro_quant_level       Gyro quantization level (rad/s)
%
% Outputs:
%   out_profile   Navigation solution as a motion profile array
%   out_errors    Navigation solution error array
%
% Format of motion profiles:
%  Column 1: time (sec)
%  Column 2: latitude (rad)
%  Column 3: longitude (rad)
%  Column 4: height (m)
%  Column 5: north velocity (m/s)
%  Column 6: east velocity (m/s)
%  Column 7: down velocity (m/s)
%  Column 8: roll angle of body w.r.t NED (rad)
%  Column 9: pitch angle of body w.r.t NED (rad)
%  Column 10: yaw angle of body w.r.t NED (rad)
%
% Format of error array:
%  Column 1: time (sec)
%  Column 2: north position error (m)
%  Column 3: east position error (m)
%  Column 4: down position error (m)
%  Column 5: north velocity (m/s)
%  Column 6: east velocity (m/s)
%  Column 7: down velocity (m/s)
%  Column 8: attitude error about north (rad)
%  Column 9: attitude error about east (rad)
%  Column 10: attitude error about down = heading error  (rad)

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

% Initialize true navigation solution
old_time = in_profile(1,1);
old_true_L_b = in_profile(1,2);
old_true_lambda_b = in_profile(1,3);
old_true_h_b = in_profile(1,4);
old_true_v_eb_n = in_profile(1,5:7)';
old_true_eul_nb = in_profile(1,8:10)';
old_true_C_b_n = Euler_to_CTM(old_true_eul_nb)';

% Initialize estimated navigation solution
[old_est_L_b,old_est_lambda_b,old_est_h_b,old_est_v_eb_n,old_est_C_b_n] =...
    Initialize_NED(old_true_L_b,old_true_lambda_b,old_true_h_b,...
    old_true_v_eb_n,old_true_C_b_n,initialization_errors);

% Initialize output profile record and errors record
out_profile = zeros(no_epochs,10);
out_errors = zeros(no_epochs,10);

% Generate output profile record
out_profile(1,1) = old_time;
out_profile(1,2) = old_est_L_b;
out_profile(1,3) = old_est_lambda_b;
out_profile(1,4) = old_est_h_b;
out_profile(1,5:7) = old_est_v_eb_n';
out_profile(1,8:10) = CTM_to_Euler(old_est_C_b_n')';

out_errors(1,1) = old_time;
out_errors(1,2:4) = initialization_errors.delta_r_eb_n';
out_errors(1,5:7) = initialization_errors.delta_v_eb_n';
out_errors(1,8:10) = initialization_errors.delta_eul_nb_n';

% Initialize IMU quantization residuals
quant_residuals = [0;0;0;0;0;0];

% Progress bar
dots = '....................';
bars = '||||||||||||||||||||';
rewind = '\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b';
fprintf(strcat('Processing: ',dots));
progress_mark = 0;
progress_epoch = 0;

% Main loop
for epoch = 2:no_epochs

    % Update progress bar
    if (epoch - progress_epoch) > (no_epochs/20)
        progress_mark = progress_mark + 1;
        progress_epoch = epoch;
        fprintf(strcat(rewind,bars(1:progress_mark),...
            dots(1:(20 - progress_mark))));
    end % if epoch    
    
    % Input data from motion profile
    time = in_profile(epoch,1);
    true_L_b = in_profile(epoch,2);
    true_lambda_b = in_profile(epoch,3);
    true_h_b = in_profile(epoch,4);
    true_v_eb_n = in_profile(epoch,5:7)';
    true_eul_nb = in_profile(epoch,8:10)';
    true_C_b_n = Euler_to_CTM(true_eul_nb)';
   
    % Time interval
    tor_i = time - old_time;
    
    % Calculate true specific force and angular rate
    [true_f_ib_b,true_omega_ib_b] = Kinematics_NED(tor_i,true_C_b_n,...
        old_true_C_b_n,true_v_eb_n,old_true_v_eb_n,true_L_b,true_h_b,...
        old_true_L_b,old_true_h_b);
    
    % Simulate IMU errors
    [meas_f_ib_b,meas_omega_ib_b,quant_residuals] = IMU_model(tor_i,...
        true_f_ib_b,true_omega_ib_b,IMU_errors,quant_residuals);
    
    % Update estimated navigation solution
    [est_L_b,est_lambda_b,est_h_b,est_v_eb_n,est_C_b_n] = ...
        Nav_equations_NED(tor_i,old_est_L_b,old_est_lambda_b,old_est_h_b,...
        old_est_v_eb_n,old_est_C_b_n,meas_f_ib_b,meas_omega_ib_b);
   
    % Generate output profile record
    out_profile(epoch,1) = time;
    out_profile(epoch,2) = est_L_b;
    out_profile(epoch,3) = est_lambda_b;
    out_profile(epoch,4) = est_h_b;
    out_profile(epoch,5:7) = est_v_eb_n';
    out_profile(epoch,8:10) = CTM_to_Euler(est_C_b_n')';
    
    % Determine errors and generate output record
    [delta_r_eb_n,delta_v_eb_n,delta_eul_nb_n] = Calculate_errors_NED(...
        est_L_b,est_lambda_b,est_h_b,est_v_eb_n,est_C_b_n,true_L_b,...
        true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);
    out_errors(epoch,1) = time;
    out_errors(epoch,2:4) = delta_r_eb_n';
    out_errors(epoch,5:7) = delta_v_eb_n';
    out_errors(epoch,8:10) = delta_eul_nb_n';
    
    % Reset old values
    old_time = time;
    old_true_L_b = true_L_b;
    old_true_lambda_b = true_lambda_b;
    old_true_h_b = true_h_b;
    old_true_v_eb_n = true_v_eb_n;
    old_true_C_b_n = true_C_b_n;
    old_est_L_b = est_L_b;
    old_est_lambda_b = est_lambda_b;
    old_est_h_b = est_h_b;
    old_est_v_eb_n = est_v_eb_n;
    old_est_C_b_n = est_C_b_n;

end %epoch

% Complete progress bar
fprintf(strcat(rewind,bars,'\n'));
    
% Ends