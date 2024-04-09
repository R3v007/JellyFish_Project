close all;
clear;

%% Setting up all the variables for data collection
tagsize=12; 
window=5;
worldPoints = [0 0 0; tagsize/2 0 0; 0 tagsize/2 0; 0 0 tagsize/2];
depth_window=[0.60 0.92];

%% Camera setup
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

%% Setting up Offset


%% Setting up the serial communication
s=serialport("COM6", 115200);  %check the serial port and check BAUDRATE
pause(1);  %delaying to allow connection setup
output="0.5;0.8";   %setting to simple vals
writeline(s, output); %sending output
disp("Serial Set-up done and sent Off to uno");
disp("Pausing for 5 seconds to allow setup");
pause(5);
disp("Pausing done, Next Step Commence:");




dataset=input("Enter dataset for this run: \n"); 
k_p=input("Enter the Proportional constant for this run: \n");
k_i=input("Enter Integral Constant for this run: \n");
k_d=input("Enter the Derivative Constant for this run: \n");
pos=1;

points=zeros(dataset, 3);%x, y, z
datas=zeros((dataset),4);%moving median x yz  with timestamp
sent_data=zeros(dataset, 3);
errors=zeros((dataset), 4);
goal_tracking=zeros((3*dataset),2);
goal_pos=1;

master_id=input("Enter the master's ID: \n");
goal_depth=0;
while(goal_depth==0)
test_img=snapshot(cam);
imshow(test_img);
I = undistortImage(test_img,intrins, OutputView="same"); %undistorting
[id,loc,pose] = readAprilTag(I, "tag36h11", intrins, tagsize);


    for m=1:length(pose)
        if id(m)==master_id
            goal_depth=(pose(1,m).Translation(3));
            goal_depth=(floor(goal_depth)/1000) - 0.22;
            disp(["New Goal is: ", goal_depth]);
        else
            current_depth=((pose(m).Translation(3))/1000) - 0.22; %obtaining depth
            disp(current_depth);
        end
    end
end
close all;
disp("All Intialisations Complete");
disp("BEGIN!")
tic;
figure("Name", "Live Depth");
j=1;
while j<dataset
    %IMAGE PROCESSING
       cumu_error=0;
        img=snapshot(cam);
        path="data\images\image"+string(j)+".png";
        imwrite(img, path);
        I = undistortImage(img,intrins, OutputView="same"); %undistorting
        [id,loc,pose] = readAprilTag(I, "tag36h11", intrins, tagsize);
        disp(id);
        if(isempty(pose))
                disp("No Tag Detected");
                continue;
        end
        for m=1:length(pose)
            if id(m)==master_id || mod(j,10)==0
                goal_depth=(pose(1,m).Translation(3));
                goal_depth=(floor(goal_depth)/1000)-0.22;
                disp(["New Goal is: ", goal_depth]);
                disp(["ID for master is: ", id(m)]);
                goal_tracking(goal_pos,1)=(pose(1,m).Translation(3))/1000-0.22;
                goal_tracking(j,2)=toc;
                goal_pos=goal_pos+1;
                % continue;
            elseif id(m)==0
                    %TAG DETECTION AND DATA TAKING FOR 10 STEPS
                    % disp(pose(m,1).Translation(1));
                    points(j, 1)= (pose(1,m).Translation(1))/1000;
                    points(j, 2)= (pose(1,m).Translation(2))/1000;
                    depth= ((pose(1,m).Translation(3))/1000)-0.22; %obtaining depth
                    disp(depth);
                    points(j,3)=depth;
        
                    disp(["Depth read: ", points(j,3)]);
                    disp(["ID of robot is: ", id(m)]);
                    datas(:, 1)=movmedian(points(:,1), window, "omitmissing");
                    datas(:, 2)=movmedian(points(:,2), window, "omitmissing");
                    datas(:, 3)=movmedian(points(:,3), window, "omitmissing");
                    disp(["Medained depth:",datas(j, 3)]);
                    disp(["Goal Depth: ", goal_depth]);
                    if(datas(j,3)<0.5)
                            datas(j,3)=points(j,3);
                            disp(["Replaced the 0 with: ", datas(j,3)]);
                    end
                    datas(j, 4)=toc;
        
                    plot(datas(:,3)); %plotting depth for me to see for issues
                    hold on;
                    errors(j,1)=goal_depth-datas(j,3);
                    errors(j,4)=toc;
                    if j>3
                        for i=2:j
                            cumu_error=cumu_error + errors(i,1)*(errors(i,3)-errors(i-1,3));
                        end
                    end
        
                    errors(j,2)=cumu_error;
                    if j>1
                        errors(j,3)=(errors(j,1)-errors(j-1,1))/(errors(j,4)-errors(j-1,4));
                    end
        
                    errors(j,4)=toc;
                    disp(["Error now: ", errors(j, 1)]);
                    plot(errors(:,1));
        
                    legend("Depth", "Error");
                    xlabel("Time");
                    ylabel("Depth from camera");
                    hold off;
                    cumu_error=sum(errors(1:j,1));
    
            dc_out=(k_p*-1*errors(j,1))+(k_i*abs(errors(j,2)))+(k_d*errors(j,3));
        
            if dc_out<0
                if errors(j,1)<0
                    dc_out=0.2;
                else
                    dc_out=0;
                end
            elseif   dc_out>0.8
                dc_out=0.5;
            end
            
            disp("Freq 0.8 is maintained");
            disp(["UTR: ", dc_out]);
            sent_data(pos,1)=dc_out;
            sent_data(pos,2)=0.8;
            % disp("Pausing for 2 secs before sending");
            % pause(2);
            sent_data(pos,3)=toc;
            output=string(sent_data(pos,1))+";"+string(sent_data(pos,2));
            writeline(s, output);
            disp("pausing for the data to be sent");
            pause(1); %pause   
            % pos=pos+1; %pushing index forward
            % j=j+1;

            end %ending of if
            
        end  %end of pose loop 
    pos=pos+1; %pushing index forward
    j=j+1;

end %ending the frequency for loop

%% Post session Finish-up
output="0.5;0.8";   %setting to simple vals
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
writematrix(sent_data, 'data\sent_data.xls');
writematrix(errors, 'data\errors.xls');
writematrix(goal_tracking, 'data\goal_pos.xls');


figure("Name","Depths");
plot(datas(:,4),datas(:,3), "DisplayName","Z-Position");
hold on;
plot(goal_tracking(:,2), goal_tracking(:,1));
