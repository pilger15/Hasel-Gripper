%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% SET_GAIN_P(HANDLE,value)
%
% takes:
%   HANDLE is the serial-port ID from OPEN_CONTROLLER.
%   P is the proportional gain (between 0 : 0.25 : 7.75)
%
% create and sends a 8-bit serial messages of the format:
%
% 'CCCV VVVV'
%C = Command bits V = Value bits
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function output = set_gain_p(handle,value)

CommandBits = 0x20; % command for 'P'
if value > 7.75
    value = 7.75;
    warning('Value out of range (0.00 : 0.25 : 7.75).  Set to 7.75')
elseif value < 0
    value = 0;
    warning('Value out of range (0.00 : 0.25 : 7.75).  Set to 0.00')
end
value = value * 4; % scale to 5 bit value
    
    message = CommandBits + value;

    % check matlab version
    if verLessThan('matlab', '9.9')
        fprintf(handle,message);
        output = fscanf(handle);
    else
        write(handle,message,'uint8');
        output = read(handle,1,'uint16');        
    end
    disp(['P set to ',num2str(output/4)]);
    disp(message)
end