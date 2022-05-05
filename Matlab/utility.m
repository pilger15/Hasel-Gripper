SERIAL_PORT = 'COM3';       % change to device port
BAUD_RATE =  115200;
first = true;
test = [1,2,3];
s = serialport(SERIAL_PORT, BAUD_RATE);
clear stream
flush(s);
data = [0,0];
while first
    if floor(s.NumBytesAvailable)> 2*4
    stream = read(s,4,'uint16');
    first = false;
    end
end
plot(1,1);
while true
    n = floor(s.NumBytesAvailable/2);
    if n > 100
        stream = horzcat(stream,read(s,n,'uint16'));
        target = stream(1:4:end);
        filtered = stream(2:4:end);
        error = stream(3:4:end);
        pmw = stream(4:4:end);
     %    data = reshape(stream,[],2);
         subplot(4,1,1); 
         title('output');
         plot(target)
         
         subplot(4,1,2); 
         title('ADC');
         plot(filtered)
  
         subplot(4,1,3);
         title('Target*gain');
         plot(error)
   
         subplot(4,1,4)
         title('PMW');
         plot(pmw)
         %legend('ADC','Error','PWM')
         pause(0.1);
    end
end
