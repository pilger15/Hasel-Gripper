%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% SET_OUTPUTS(HANDLE,state)
%
% takes:
%   HANDLE is the serial-port ID from OPEN_CONTROLLER.
%   state is a string or the number representing the respective H-bridge
%       configuration
%
%   states:
%       0 - 'H_OFF'     All opto diodes are OFF
%       1 - 'H_LEFT'    H-L and L-R are ON 
%       2 - 'H_RIGHT'   H-R and L-L are ON
%       3 - 'H_DIS'     L-L and L-R are ON
%   
%
% create and sends a 8-bit serial messages of the format:
%
% 'CCCV VVVV'
%C = Command bits V = Value bits
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function output = set_outputs(handle,state)

    CommandBits = 0xA0;

    switch state
        case 'H_OFF'
            SetHValue = 0;

        case 'H_LEFT'
            SetHValue = 1;

        case 'H_RIGHT'
            SetHValue = 2;

        case 'H_DIS'
            SetHValue = 3;

        otherwise
            if ischar(state) || (state> 3 || state < 0)
                 error(sprintf('Error: Unexpected H-Bridge state.\n Please select one of the following states:\n0 - ''H_OFF''\n1 - ''H_LEFT''\n2 - ''H_RIGHT''\n3 - ''H_DIS'''));
            else
                SetHValue = state;            
            end
    end

    message = CommandBits + SetHValue;

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
    switch output
        case 0	% 'H_OFF'
            disp('All diodes are OFF')
        case 1
            disp('H-L and L-R are ON')
        case 2
            disp('H-R and L-L are ON')
        case 3
            disp('L-L and L-R are ON (discharge)')
    end
end

