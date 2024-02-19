%setting up camera
cam=webcam;
cam.Resolution = '1280x960';
dataset=300;

%setting up the goal points
%goal1=


%preprocessing
cameraParams=load("calibs\270124_2.mat");
intrins=cameraParams.cameraParams.Intrinsics;
K = cameraParams.cameraParams.Intrinsics.K;
average=[0.0, 0.0, 0.0];

%loading the data to correct for underwater refraction
data_uw=load("offset_Data\28th_jan_20cm_water.mat");
uw_calib_mat=data_uw.uw_calib_Data;
p_uw=polyfit(uw_calib_mat(:,2), uw_calib_mat(:,1), 1);

%loading the data to correct for out of water offset
data=load("offset_Data\28th_Dark.mat");
calib_mat=data.calib_mat;
p_ow=polyfit(calib_mat(:,2), calib_mat(:,1), 1);

tagsize=17;

worldPoints = [0 0 0; tagsize/2 0 0; 0 tagsize/2 0; 0 0 tagsize/2];
goal1points = [0, -2];
goal2points = [0, -2];

focalLength = intrins.FocalLength(1, 1);
pos=1;
points=zeros(dataset, 3);
disp("entering while loop");

while(pos<=dataset)

    img=snapshot(cam);
    
    %imshow(img);
    I = undistortImage(img,intrins, OutputView="same"); %undistorting
    [id,loc,pose] = readAprilTag(I, "tag36h11", intrins, tagsize);

    for i = 1:length(pose)
        %disp("in for loop");
        % Get image coordinates for axes.
        % goal1=world2img(goal1points, pose(i), intrins);
        % goal2=world2img(goal2points, pose(i), intrins);
        imagePoints = world2img(worldPoints,pose(i),intrins);
        
        points(pos, 1)= (pose(i).Translation(1))/1000;
        average(1)=((average(1)*(pos-1))+points(pos, 1))/pos;
        points(pos, 2)= (pose(i).Translation(2))/1000;
        average(2)=((average(2)*(pos-1))+points(pos, 2))/pos;
        depth= (pose(i).Translation(3))/1000; %obtaining depth
        disp(depth);
        if depth<0.67
            disp("overwater corrector used");
            points(pos, 3) = corrector(depth, p_ow); %correcting it for overwater
            disp(points(pos, 3));
        end
        if depth>0.67
            disp("underwater corrector used");
            points(pos, 3)= corrector(depth, p_uw); %correcting it for underwater
            disp(points(pos,3));
        end
        average(3)=((average(3)*(pos-1))+points(pos, 3))/pos; %averagin it
        scatter3(points(:,1), points(:,2), points(:,3));
        xlabel('X-Pos');
        ylabel('Y-pos');
        zlabel('Z-pos');
         
    end
    pos=pos+1;
end
hold off;
scatter3(points(:,1), points(:,2), points(:,3));
xlabel('X-Pos');
ylabel('Y-pos');
zlabel('Z-pos');
hold on;
scatter3(0, 0, 0);
writematrix(points, 'data\data.xls');


function corr_val=corrector(depth, p)
    corr_val=(p(1)*depth)+p(2);
end