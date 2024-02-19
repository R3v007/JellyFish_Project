%% Setting up the camera
cam=webcam;
cam.Resolution = '1280x960';

%% Setting up the serial communication
s=serialport("COM8", 115200);  %check the serial port and check BAUDRATE
pause(1);  %delaying to allow connection setup
output="0;100;0;100;0;100;0;100";   %setting all the solenoids to 0
writeline(s, output); %sending output

%% Camera preprocessing
cameraParams=load("calibs\270124_2.mat");
intrins=cameraParams.cameraParams.Intrinsics;
K = cameraParams.cameraParams.Intrinsics.K;
focalLength = intrins.FocalLength(1, 1);

%% Offset corrections
%Underwater refraction correction data
data_uw=load("offset_Data\28th_jan_20cm_water.mat");
uw_calib_mat=data_uw.uw_calib_Data;
p_uw=polyfit(uw_calib_mat(:,2), uw_calib_mat(:,1), 1);

%Overwater system offset correction data
data=load("offset_Data\28th_Dark.mat");
calib_mat=data.calib_mat;
p_ow=polyfit(calib_mat(:,2), calib_mat(:,1), 1);

%% Defining important points and the goal points
tagsize=17; %% in mm
worldPoints = [0 0 0; tagsize/2 0 0; 0 tagsize/2 0; 0 0 tagsize/2];
goal1points = [0.2831, -0.11685];
goal2points = [-0.265955, -0.02385];
goal_depth= 0.58;


%% Setting up all the variables for data collection
dataset=300; siz=3; pos=1; sit=1; %change data set length as needed
points=zeros(dataset, 4);%x, y, z, and time stamp
datas=zeros((dataset/siz),4);%actual sliding average data with timestamp
sum=[0.0,0.0,0.0]; 
errors=zeros((dataset/siz), 4); %goal x, y, z - robot x, y, z + timestamp
direc=zeros(dataset, 3); %vector showing goal direction with timestamp
disp("All Intialisations Complete");
disp("Ready to commence super evil plan");

%% Seetting up the parameters for the fins outputs
fin1=[0.0, 100];
fin2=[0.0, 100];
fin3=[0.0, 100];
fin4=[0.0, 100];
output=string(fin1(1))+";"+string(fin1(2))+";"+ ...
    string(fin2(1))+";"+string(fin2(2))+";"+...
    string(fin3(1))+";"+string(fin3(2))+";"+...
    string(fin4(1))+";"+string(fin4(2));
writeline(s, output);

tic;
while(pos<=dataset)
    
    %IMAGE PROCESSING
    img=snapshot(cam);
    %imshow(img);
    I = undistortImage(img,intrins, OutputView="same"); %undistorting
    [id,loc,pose] = readAprilTag(I, "tag36h11", intrins, tagsize);
    for i = 1:length(pose)
        imagePoints = world2img(worldPoints,pose(i),intrins);
        points(pos, 1)= (pose(i).Translation(1))/1000;
        sum(1)=sum(1)+points(pos, 1);
        points(pos, 2)= (pose(i).Translation(2))/1000;
        sum(2)=sum(2)+points(pos, 2);
        depth= (pose(i).Translation(3))/1000; %obtaining depth
        disp(depth);
        if depth<0.69 % DEPTH CORRECTION
            disp("overwater corrector used");
            points(pos, 3) = corrector(depth, p_ow); %overwater correxn
            disp(points(pos,3));
        end
        if depth>0.69 % DEPTH CORRECTION
            disp("underwater corrector used");
            points(pos, 3)=corrector(depth, p_uw); %underwater correxn
            disp(points(pos,3));
        end
        points(pos, 4)=toc;
        sum(3)=sum(3)+points(pos, 3);
    end
    %AVERAGING
    if mod(pos, siz)==0
        datas(sit, 1)=sum(1)/siz;
        datas(sit, 2)=sum(2)/siz;
        datas(sit, 3)=sum(3)/siz;
        datas(sit, 4)=toc;
        sum=[0,0,0];
        % CALCULATING THE ERRORS
        errors(sit, 1)=goal1points(1)-datas(sit, 1);
        errors(sit, 2)=goal1points(2)-datas(sit, 2);
        errors(sit, 3)=goal_depth-datas(sit, 3);
        errors(sit, 4)=toc;
        sit=sit+1;

    %% CONTROL SEGMENT COMES HERE

    %% CONTROL SEGMENT ENDS

    end
    %MIGHT BE REMOVED LATER ON BUT FOR NOW, PLOTTING POSITION
    scatter3(datas(:,1), datas(:,2), datas(:,3));
    xlabel('X-Pos');
    ylabel('Y-pos');
    zlabel('Z-pos');
    pos=pos+1;
    %SENDING CONTROL OUTPUT
    output=string(fin1(1))+";"+string(fin1(2))+";"+ ...
    string(fin2(1))+";"+string(fin2(2))+";"+...
    string(fin3(1))+";"+string(fin3(2))+";"+...
    string(fin4(1))+";"+string(fin4(2));
    writeline(s, output);
    
end

%% Post session data saving
hold off;
scatter3(points(:,1), points(:,2), points(:,3));
xlabel('X-Pos');
ylabel('Y-pos');
zlabel('Z-pos');
hold on;
scatter3(datas(:,1), datas(:,2), datas(:,3));
writematrix(datas, 'data\data.xls');
writematrix(errors, 'data\error_data.xls');

%% CORRECTION FUNCTION
function corr_val=corrector(depth, p)
    corr_val=(p(1)*depth)+p(2);
end