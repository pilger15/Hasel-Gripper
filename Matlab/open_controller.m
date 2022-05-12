%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% [HANDLE] = OPEN_CONTROLLER(PORT)
%
% PORT:
%   Windows:    'COM#'
%   Mac/Linux:	'/dev/tty.usbmodem#' (to find this, run "ls /dev/tty.*" in terminal)
%
% HANDLE:       serial-port ID
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [handle] = open_controller(port)
% check matlab version
if verLessThan('matlab', '9.9')
    handle = serial(port,'BaudRate',9600);
    fopen(handle);
else
    handle = serialport(port,9600);
end

end