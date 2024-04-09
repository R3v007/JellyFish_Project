close all;
clear;

%% Other cooler camera setup
%setting up camera
cam=webcam("HD Web Camera");
cam.Resolution = '1920x1080';
cameraParams=load("calib_data_second\right_cam_clib.mat");
intrins=cameraParams.cameraParamsRight.Intrinsics;
K = cameraParams.cameraParamsRight.Intrinsics.K;
focalLength = intrins.FocalLength(1, 1);
test_img=snapshot(cam);
imshow(test_img);
pause(4);
close all;
disp("Camera Set-up Done");


%% Setting up the serial communication
s=serialport("COM6", 115200);  %check the serial port and check BAUDRATE
pause(1);  %delaying to allow connection setup
output="0.5;0.5";   %setting to simple vals
writeline(s, output); %sending output
disp("Serial Set-up done and sent Off to uno");
disp("Pausing for 10 seconds to allow setup");
pause(10);
disp("Pausing done, Next Step Commence:");

%% Setting up all the variables for data collection
tagsize=12; 
worldPoints = [0 0 0; tagsize/2 0 0; 0 tagsize/2 0; 0 0 tagsize/2];

start_dc=0.1; final_dc=1;  %set start and end duty cycle values
start_freq=0.5;  final_freq=1.6; %set start and end frequency values

steps_dc=input("Enter the number of steps for Duty Cycle Sweep: \n");
steps_freq=input("Enter the number of steps for Frequency Sweep: \n");
data_Steps=input("Enter the desired datapoints number: \n");%set step sizes


increase_dc=(final_dc-start_dc)/steps_dc;
increase_freq=(final_freq-start_freq)/steps_freq;

dataset=steps_freq*steps_dc; pos=1;

points=zeros((dataset*data_Steps), 3);%x, y, z
datas=zeros((dataset*data_Steps),4);%moving median x yz  with timestamp
displacement=zeros(((dataset*data_Steps)-1),2); %total displacement in 3d with time interval
speed=zeros(((dataset*data_Steps)-1),1); %speed = displacement/timeinterval
sent_data=zeros(dataset, 3);

disp("All Intialisations Complete");

disp("BEGIN!")
tic;
figure("Name", "Live Speed");
for i=1:steps_dc
    for j=1:steps_freq
        %SENDING THE DATA
        sent_data(pos,1)=start_dc+((i-1)*increase_dc);
        sent_data(pos,2)=start_freq+((j-1)*increase_freq);
        disp(["UTR and Freq: ", sent_data(pos,1), sent_data(pos,2)]);
        disp("Pausing for 10 secs to allow for the robot to sink. " + ...
            "Please switch off power supply and let it sink \n");
        pause(10);
        disp("Pausing for 10 secs before sending");
        pause(10);
        sent_data(pos,3)=toc;
        output=string(sent_data(pos,1))+";"+string(sent_data(pos,2));
        writeline(s, output);
        disp("pasusing for the data to be sent");
        pause(1); %pause

        %IMAGE PROCESSING
        k=1;
        while k<data_Steps+1
            % TAG DETECTION AND DATA TAKING FOR 10 STEPS
            img=snapshot(cam);
            pointer=((pos-1)*data_Steps+k);
            path="data\images\image"+string(pointer)+".png";
            imwrite(img, path);
            I = undistortImage(img,intrins, OutputView="same"); %undistorting
            [id,loc,pose] = readAprilTag(I, "tag36h11", intrins, tagsize);
            if(isempty(pose))
                disp("No Tag Detected");
                continue;
            end
            for m=1:length(pose)                
                points(pointer, 1)= (pose(m).Translation(1))/1000;
                points(pointer, 2)= (pose(m).Translation(2))/1000;
                depth= (pose(m).Translation(3))/1000; %obtaining depth
                disp(depth);
                %SANITY CHECK
                disp([pointer pos k j i]);

                points(pointer,3)=depth-0.2;
                disp(["Depth read: ", points(pointer,3)]);
                datas(:, 1)=movmedian(points(:,1), 4, "omitmissing");
                datas(:, 2)=movmedian(points(:,2), 4, "omitmissing");
                datas(:, 3)=movmedian(points(:,3), 4, "omitmissing");
                disp(["Medained depth:",datas(pointer, 3)]);
                if(datas(pointer,3)==0)
                    datas(pointer,3)=points(pointer,3);
                    disp(["Replaced the 0 with: ", datas(pointer,3)]);
                end
                datas(pointer, 4)=toc;
                plot(datas(:,3)); %plotting depth for me to see for issues
               
            end  %end of data taking loop
            k=k+1;
        end %end of the pose loop thingy
        
        pos=pos+1; %pushing index forward

    end %ending the frequency for loop
end %ending the duty cycle for sloop

%% Post session Finish-up
output="0.5;0.5";   %setting to simple vals
writeline(s, output); %sending output
disp("Uno now set to simple vals, can now disconnect");

%% Post-Processing Graphs
figure("Name", "3D Trajectory");
scatter3(datas(:,1), datas(:,2), datas(:,3));
xlabel('X-Pos');
ylabel('Y-pos');
zlabel('Z-pos');


writematrix(datas, 'data\data.xls');
writematrix(points, 'data\points.xls');
writematrix(displacement, 'data\displacement.xls');
writematrix(speed,'data\speed.xls')
writematrix(sent_data, 'data\sent_data.xls')


figure("Name","Points matrix");
plot(datas(:,4),points(:,1), "DisplayName","X-Position");
hold on;
plot(datas(:,4),points(:,2), "DisplayName","Y-Position");
hold on;
plot(datas(:,4),points(:,3), "DisplayName","Z-Position");

figure("Name", "Datas Matrix");
plot(datas(:,4),datas(:,1),"DisplayName","X-Position");
hold on;
plot(datas(:,4),datas(:,2),"DisplayName","Y-Position");
hold on;
plot(datas(:,4),datas(:,3), "DisplayName","Z-Position");

figure("Name", "Displacement");
plot(displacement(:,2),displacement(:,1),"DisplayName","Overall-Displacement");

figure("Name", "Speeds");
plot(displacement(:,2),speed(:),"DisplayName","Speed");