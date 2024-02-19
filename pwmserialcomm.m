s=serialport("COM5", 9600);
for i=1:10
    disp("Iteration "+i);
    pwm=i*250/10;
    writeline(s, int2str(pwm));
    pause(2);
end
clear s;
