close all;
%This is mostly done by eye and with the robot held in place so the camera
%data doesnt really matter and will be deleted

%% Setting up the serial communication
s=serialport("COM6", 115200);  %check the serial port and check BAUDRATE
pause(1);  %delaying to allow connection setup
freq=1.0; %actual data that will be sent to the robot
time_s=1/freq; 
time_ms=1000*time_s; %data for slef refernce
disp(["Time in seconds: ", time_s]);
disp(["Time in millisecs: ", time_ms]);
disp(["Frequency sent to the UNo: ", freq]);
output=string(freq);   %setting all the solenoids to 0
writeline(s, output); %sending output
disp("Serial Set-up done and sent Initial Zero data to uno");
% disp("Pausing for 10 seconds to allow setup");
% pause(10);
% disp("Pausing done, Next Step Commence:");

%% Setting up all the variables for data collection
initial_Freq=0.3; final_Freq=1.3; 
freq_steps=0.1;
dataset=(final_Freq-initial_Freq)/freq_steps;

pos=1; %change data set length as needed
points=zeros(dataset, 3);%freq, time_s, time_ms

disp("All Intialisations Complete");
disp("Ready to commence super evil plan");

%% Fin1 serial comms in conjeture with the serial working code
% make sure that the serial working code is actuating the fin 1
%otr=0.5;
time_s=1/initial_Freq;
time_ms=time_s*1000;
points(1, 1)=initial_Freq;
points(1, 2)=time_s;
points(1, 3)=time_ms;
output=string(points(1, 1));

disp("BEGIN FREQUENCY SWEEP!")
tic;
writeline(s, output);
disp("Sent Initial Frequency to Uno. Commence Viewing!");
while(pos<=dataset)
    points(pos,1)=initial_Freq+((pos-1)*steps) ;
    time_s=1/points(pos,1); 
    time_ms=1000*time_s; %data for slef refernce
    disp(["Time in seconds: ", time_s]);
    disp(["Time in millisecs: ", time_ms]);
    disp(["Frequency sent to the UNo: ", points(pos,1)]);
    disp("Pausing for 5 secs to analyse visually");
    pause(5);
    disp("increasing the step");
    pos=pos+1;
end

%% Post session Finish-up
freq=1.0;
output=string(freq);
writeline(s, output);
disp("Uno set to zero, can now disconnect");
clear;