%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% SET_GAIN_P(HANDLE,value)
%
% takes:
%   HANDLE is the serial-port ID from OPEN_CONTROLLER.
%   P is the proportional gain (integer between 0 and 9999)
%
% create and sends a 5-byte serial messages of the format:
%
% 'PXXXX'
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function set_gains(handle,value)

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
message = ['P' value_string ' '];

% send the message to the M2
for i = 1:size(message,2)
    fprintf(handle,message(i))
end
disp(fscanf(handle))
