%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% SET_OUTPUTS(HANDLE,HL,LL,HR,LR)
%
% takes:
%   HANDLE is the serial-port ID from OPEN_CONTROLLER.
%   HL is the state of the high-left switch (0=OFF, 1=ON)
%   LL is the state of the low-left switch (0=OFF, 1=ON)
%   HR is the state of the high-right switch (0=OFF, 1=ON)
%   LR is the state of the high-right switch (0=OFF, 1=ON)
%
% create and sends a 5-byte serial messages of the format:
%
% 'HXXXX'
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function set_outputs(handle,hl,ll,hr,lr)

hl_string = num2str(hl);
ll_string = num2str(ll);
hr_string = num2str(hr);
lr_string = num2str(lr);

% form the message
message = ['H' hl_string ll_string hr_string lr_string ' '];

% send the message to the M2
for i = 1:size(message,2)
    fprintf(handle,message(i))
end
disp(fscanf(handle))
