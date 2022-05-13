SERIAL_PORT = 'COM3';       % change to device port

targetVoltage = 6000;
p_gain = 1;
i_gain = 0;
d_gain = 0;

% connect to the M2 @ SERIAL_PORT
M2 = open_controller(SERIAL_PORT);

% set the target voltage
HighVoltageMessage = set_v(M2,targetVoltage); 

% set the p gain between 0 : 0.25 : 7.75
p_gain_message = set_gain_p(M2,p_gain); 
% set the i gain between 0 : 0.25 : 7.75
i_gain_message = set_gain_i(M2,i_gain); 
% set the d gain between 0 : 0.25 : 7.75
d_gain_message = set_gain_d(M2,d_gain); 

%% control the H_Bridge
%   states:
%       0 - 'H_OFF'     All opto diodes are OFF
%       1 - 'H_LEFT'    H-L and L-R are ON 
%       2 - 'H_RIGHT'   H-R and L-L are ON
%       3 - 'H_DIS'     L-L and L-R are ON

current_state = set_outputs(M2,'H_OFF');
% current_state = set_outputs(M2,0); % alternative 


% disconnect the controller
close_controller(M2); 