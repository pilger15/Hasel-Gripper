%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% set_streaming(HANDLE,state)
% activates the streaming mode.
% The M2 will start to continously send the current errror message as int16 
%
%
% takes:
%   HANDLE is the serial-port ID from OPEN_CONTROLLER.
%   state  
%       TRUE = Streaming is activated
%       FALSE = Streaming is deactivated
%
%   
%
% create and sends a 8-bit serial messages of the format:
%
% 'CCCx xxxV'
%C = Command bits V = Value bits
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function output = set_streaming(handle, state)

    CommandBits = 0xC0;

    

    message = CommandBits + state;

    % check matlab version
    if verLessThan('matlab', '9.9')
        fprintf(handle,message);
        while(handle.NumBytesAvailable < 2)
        end
        output = fscanf(handle);
    else
        write(handle,message,'uint8');
        while(handle.NumBytesAvailable < 2)
        end
        output = read(handle,1,'uint16');  
        
    end
    
end

