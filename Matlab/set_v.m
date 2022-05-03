%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% SET_V(HANDLE,V)
%
% takes:
%   HANDLE is the serial-port ID from OPEN_CONTROLLER.
%   V is the desired high voltage (integer between 0 and 9999)
%
% create and send a 5-byte serial message of the format:
%
% 'VXXXX'
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function set_v(handle,value)

if(value<0)
    value = 0;
end
if(value>9999)
    value = 9999;
end
value_string = num2str(value);
if(size(value_string,2)==3)
    value_string = ['0' value_string];
elseif(size(value_string,2)==2)
    value_string = ['00' value_string];
elseif(size(value_string,2)==1)
    value_string = ['000' value_string];
end

% form the message
message = ['V' value_string ' '];

% send the message to the M2
for i = 1:size(message,2)
    fprintf(handle,message(i))
end
disp(fscanf(handle))
