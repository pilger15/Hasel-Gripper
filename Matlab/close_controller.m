%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CLOSE_CONTROLLER(HANDLE)
%
% takes: 
%   HANDLE is the serial-port ID from OPEN_CONTROLLER.
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function close_controller(handle)
% check matlab version
if verLessThan('matlab', '9.9')
    fclose(handle);
else
    delete(handle);
end