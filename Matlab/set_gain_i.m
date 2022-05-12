%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% SET_GAIN_P(HANDLE,value)
%
% takes:
%   HANDLE is the serial-port ID from OPEN_CONTROLLER.
%   I is the integral gain (between 0 : 0.25 : 7.75)
%
% create and sends a 8-bit serial messages of the format:
%
% 'CCCV VVVV'
%C = Command bits V = Value bits
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function output = set_gain_i(handle,value)

CommandBits = 0x40; % command for 'I'
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
        while(handle.NumBytesAvailable < 2)
        end
        output = read(handle,1,'uint16');      
    end
    disp(['I set to ',num2str(output/4)]);
end