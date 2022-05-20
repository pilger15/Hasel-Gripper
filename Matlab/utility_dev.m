
SERIAL_PORT = 'COM4';       % change to device port
STREAM_DATA = true;         % stream the HV_output data
targetVoltage = 5000;

R1 = 1000e6;
R2 = 470e3;
gain = (R1+R2)/R2;
ADC_to_U = 5/1024;
global p_gain; 
global i_gain;
global d_gain;
global M2;

p_gain = 20;
i_gain = 0;
d_gain = 5;

% connect to the M2 @ SERIAL_PORT
if(exist('M2'))
	close_controller(M2); 
end

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

current_state = set_h_bridge(M2,'H_OFF');
% current_state = set_h_bridge(M2,0); % alternative 

if(STREAM_DATA)
    global streaming
    streaming = 0;
    
    %%gui
    PushButton = uicontrol(gcf,'Style', 'push', 'String', 'Stop','Position', [100 10 30 30],'CallBack', @PushB);
    PushButton = uicontrol(gcf,'Style', 'push', 'String', 'Left','Position', [200 10 30 30],'CallBack', @PushL);
    PushButton = uicontrol(gcf,'Style', 'push', 'String', 'Right','Position', [300 10 30 30],'CallBack', @PushR);
    PushButton = uicontrol(gcf,'Style', 'push', 'String', 'Disc','Position', [400 10 30 30],'CallBack', @PushD);
   % uicontrol('style','slider','units','normalized','position',[0.3,0.5,0.2,1.0],'CallBack', @SldP);
    %% data streaming loop
    first = true;
    
    %start streaming
    streaming = set_streaming(M2,1);
    while(streaming) % wait till button is pressed
        n = (M2.NumBytesAvailable/64);
        if(n>=1)
            if(first)
                stream = read(M2,n*32,'int16').*gain*ADC_to_U;
                first = false;
            else
                stream = horzcat(stream,read(M2,n*32,'int16').*gain*ADC_to_U);
            end
            
            pause(0.1);
            time = (1:length(stream))./7812.5;
            plot(time,stream);
        end

    end
end
%stop streaming

% disconnect the controller
close_controller(M2); 


function PushB(hObject,eventdata)
    global M2
    global streaming;
    streaming = 0;
    set_v(M2,0);
    set_streaming(M2,0);
    set_h_bridge(M2,'H_DIS');
    pause(1);
    set_h_bridge(M2,'H_OFF');
    pause(0.1);
    flush(M2);
    write(M2,0xE0,"uint8"); % dump parameters
    disp(["Disconnected..."]);
    disp(["..."]);
    disp(["..."]);
    pause(0.1);
    p = read(M2,1,"uint8");
    i = read(M2,1,"uint8");
    d = read(M2,1,"uint8");
    target = read(M2,1,"uint8");
    disp(['P set to ',num2str(p/4)]);
    disp(['I set to ',num2str(i/4)]);
    disp(['D set to ',num2str(d/4)]);
    disp(['Target set to ',num2str(target)]);
end
function PushL(hObject,eventdata)
    global M2
    set_h_bridge(M2,'H_LEFT');
end
function PushR(hObject,eventdata)
    global M2
    set_h_bridge(M2,'H_RIGHT');
end
function PushD(hObject,eventdata)
    global M2
    set_h_bridge(M2,'H_DIS');
end