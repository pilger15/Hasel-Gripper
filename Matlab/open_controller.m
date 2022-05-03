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

handle = serial(port,'BaudRate',9600);

fopen(handle);

end