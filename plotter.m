data=readmatrix("data\data.xls");
% plot3(data(:,1), data(:,2), data(:,3));
% hold on;
% plot3(0, 0, 0);
% xlabel('X-Pos');
% ylabel('Y-pos');
% zlabel('Z-pos');
coeffs=polyfit(uw_calib_Data(:,1), uw_calib_Data(:,2));
yfit=polyval(coeffs, uw_calib_Data(:,1));
scatter(uw_calib_Data(:,1), yfit);
xlabel("Actual Depth");
ylabel("Depth read by camera");