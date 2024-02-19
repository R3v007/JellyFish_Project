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
goal1points = [0, 0, 0];
goal2points = [-0.2441 -0.0697, 0];
goalxy=[50,-50];

focalLength = intrins.FocalLength(1, 1);
pos=1;
points=zeros(dataset, 3);
disp("entering while loop");

while(pos<=dataset)

    img=snapshot(cam);
    
    %imshow(img);
    I = undistortImage(img,intrins, OutputView="same"); %undistorting
    [id,loc,pose] = readAprilTag(I, "tag36h11", intrins, tagsize);
    [ny,nx]=size(I);
    C = round([nx ny]/2) ;

    for i = 1:length(pose)  
        %disp("in for loop");
        % Get image coordinates for axes.
        goal1=world2img(goal1points, pose(i), intrins);
        goal2=world2img(goal2points, pose(i), intrins);
        imagePoints = world2img(worldPoints,pose(i),intrins);
        
        % Draw colored axes and image editing. aka unecessary
         I = insertShape(I,Line=[imagePoints(1,:) imagePoints(2,:); imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], Color=["red","green","blue"],LineWidth=7);
        %I = insertShape(I,Line=[0 pose(i).Translation(1); 0 pose(i).Translation(2); 0 pose(i).Translation(3)], Color=["red","green","blue"],LineWidth=7);
    
         % = insertMarker(I, [goal1(1) goal1(2)], "plus");
         %I=insertMarker(I, [0 goal2(2)], "x-mark");
         I=insertMarker(I,goal2, "star");
         %I = insertText(I,goal1,1,BoxOpacity=1,FontSize=25);
         %I = insertText(I,goal2,2,BoxOpacity=1,FontSize=25);
          I = insertText(I,[-0.2441 -0.0697],3,BoxOpacity=1,FontSize=25);
        % I = insertText(I,loc(1,:,i),id(i),BoxOpacity=1,FontSize=25);
        % hold on;
        % points(pos, 1)= (pose(i).Translation(1))/1000;
        % average(1)=((average(1)*(pos-1))+points(pos, 1))/pos;
        % points(pos, 2)= (pose(i).Translation(2))/1000;
        % average(2)=((average(2)*(pos-1))+points(pos, 2))/pos;
        % depth= (pose(i).Translation(3)*(-1))/1000; %obtaining depth
        % if depth<0.71
        %     disp("overwater corrector used");
        %     points(pos, 3) = corrector(depth, p_ow); %correcting it for overwater
        % end
        % if depth>0.71
        %     disp("underwater corrector used");
        %     points(pos, 3)= corrector(depth, p_uw); %correcting it for underwater
        % end
        % average(3)=((average(3)*(pos-1))+points(pos, 3))/pos; %averagin it
        %disp(['Z-coordinate not-corrected (depth) of AprilTag: ' num2str(depth)]);
        %disp(['Z-coordinate corrected (depth) of AprilTag: ' num2str(points(pos,3))]);
        % scatter3(points(:,1), points(:,2), points(:,3));
        % xlabel('X-Pos');
        % ylabel('Y-pos');
        % zlabel('Z-pos');
          imshow(I);
          hold on;
         % plot(points(:,1), points(:,2));
        % hold on;
        % scatter3(points(:,1), points(:,2), points(:,3));
        % hold on;
        % % plot(points(pos,1), points(pos,2));
        %hold on;
        plot(0,0,'o');
        %disp(['Z-coordinate camera average depth: ' num2str(average(3))])
        
    end
    %imshow(I);
    %hold on;
    pos=pos+1;
end
hold off;
scatter3(points(:,1), points(:,2), points(:,3));
xlabel('X-Pos');
ylabel('Y-pos');
zlabel('Z-pos');
hold on;
scatter3(-0.2441, -0.0697,-40);
writematrix(points, 'data\data.xls');


function corr_val=corrector(depth, p)
    corr_val=(p(1)*depth)+p(2);
end