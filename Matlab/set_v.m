%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% SET_V(HANDLE,V)
%
% takes:
%   HANDLE	is the serial-port ID from OPEN_CONTROLLER.
%   V       is the desired high voltage (integer between 0 and 10600 resolution is ~ 10 V)
%
% returns
%   output	is the actual value that was bounced back from the controller
%   and represents the target ADC value that is measured at the voltage
%   divider.
%   The relation between this value and the actual HV is:
%
%      ADC value       voltage divider gain
%   output*(5/1024) *   (470k+1000M)/470k         = High voltage 
%
% create and send a 2-byte serial message of the format:
%
% 'CCCx xxVV  VVVV VVVV'
%
% C = Command bits V = Value bits
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function output = set_v(handle,value)
CommandBits = uint8(0x80); % command for 'HV_Target' 16-bit

if value > 10600
    value = 10600;
    warning('Value out of range (0 : 10 : 10600).  Set to 10600')
elseif value < 0
    value = 0;
    warning('Value out of range (0 : 10 : 10600).  Set to 0')
end
% compute the voltage divider gain
R1 = 1000e6;
R2 = 470e3;
gain = R2 / (R1+R2);

LV = value * gain; % compute the respective low voltage ADC value
target = uint16(LV * 1024/5);

    message = typecast(target,'uint8'); % Little endian !
    message(2) = CommandBits + message(2); % add Command bits
 
    % check matlab version
    if verLessThan('matlab', '9.9')
        fprintf(handle,message(2));
        fprintf(handle,message(1));
        while(handle.NumBytesAvailable < 2)
        end
        output = fscanf(handle);
    else
        write(handle,message(2),'uint8');
        write(handle,message(1),'uint8');
        while(handle.NumBytesAvailable < 2)
        end
        output = read(handle,1,'uint16');        
    end
    disp(['HV set to ',num2str(output*(5/1024)/gain),' V']);
    disp(['Target ADC ',num2str(output*(5/1024)),' V']);

end