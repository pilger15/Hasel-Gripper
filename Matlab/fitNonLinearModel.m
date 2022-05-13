

% f = 
% 
%      General model Exp2:
%      f(x) = a*exp(b*x) + c*exp(d*x)
%      Coefficients (with 95% confidence bounds):
%        a =  -1.317e+04  (-8.861e+12, 8.861e+12)
%        b =    0.001661  (-109.1, 109.1)
%        c =   1.318e+04  (-8.861e+12, 8.861e+12)
%        d =    0.001661  (-109.1, 109.1)


% p1 =  -2.707e-05  ;
% p2 =     0.03331 ;
% p3 =      -5.616 ;
% fcn = @(x)p1*x.^2 + p2*x + p3;
X = [0.7,0.75,0.8,0.85,0.9,0.95,1]*1023;
Y = [4.34,4,3.54,2.89,2.09,1.13,160e-3]*(1024/5);



y = X;
x = Y;

model = fit(x',y','poly3'); %% quite an "agressive" fit to get high voltages
subplot(3,1,1);plot(model,x,y);
ix = 0:1:1023;
tbl = model(ix);

subplot(3,1,2);plot(ix,tbl);

% %%create lookup table
a = tbl;
a(1:37)= 1023;
abit = uint16(flip(a));
subplot(3,1,3);plot(abit)
writematrix(abit','pwm_lookup.csv');

