clear;
close all;
clc;

% -------------------------------------------------------------------------
% Import data
data=xlsread('C:\Users\Dell\Desktop\PhD\IMU\line.xlsx',1);
% -------------------------------------------------------------------------
% Manually frame data

% startTime = 0;
% stopTime = 10;


gyrX = data(:, 1);
gyrY = data(:, 2);
gyrZ = data(:, 3);
accX = data(:, 4);
accY = data(:, 5);
accZ = data(:, 6);
magX= data(:, 7);
magY= data(:, 8);
magZ= data(:, 9);
mag=[magX magY magZ];
accel=[accX accY accZ];

t1=1:length(data(:, 1));
Fs=100;
samplePeriod = 1/100;
time=t1./Fs;
% -------------------------------------------------------------------------
% Detect stationary periods

% Compute accelerometer magnitude
acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);

% HP filter accelerometer data
filtCutOff = 0.001;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
acc_magFilt = filtfilt(b, a, acc_mag);

% Compute absolute value
acc_magFilt = abs(acc_magFilt);

% LP filter accelerometer data
filtCutOff = 5;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
acc_magFilt = filtfilt(b, a, acc_magFilt);

% Threshold detection
stationary = acc_magFilt < 0.05;

%%
num_points=length(mag);
for i = 1:num_points
    mag_ave(i,:)= mean. (mag);
end
mag_ave= mean (mag);
scatter3(mag_ave(:,1), mag_ave(:,2), mag_ave(:,3));
hold all
 %find callibration coefficients
[A,b,expMFS] = magcal(mag_ave);
xCorrected= (mag_ave-b)*A;

 for i= 1:num_points
    mag_calibrated= (mag_ave - b)*A;
    mag_unit=mag_calibrated ./ norm(mag_calibrated);
    
    acc_ave=mean(accel);
    accel_unit=acc_ave ./ norm(acc_ave);


    D = -accel_unit;
    E = cross(D, mag_unit);
    E = E ./ norm(E);
    N= cross(E, D);
    N = N ./norm(N);
    
    C= [N', E', D']; %BUILD DCM
    Q= quaternion(dcm2quat(C));
  end
% -------------------------------------------------------------------------
% Plot data raw sensor data and stationary periods

figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Sensor Data');
ax(1) = subplot(2,1,1);
    hold on;
    plot(time, gyrX, 'r');
    plot(time, gyrY, 'g');
    plot(time, gyrZ, 'b');
    title('Gyroscope');
    xlabel('Time (s)');
    ylabel('Angular velocity (^\circ/s)');
    legend('X', 'Y', 'Z');
    hold off;
ax(2) = subplot(2,1,2);
    hold on;
    plot(time, accX, 'r');
    plot(time, accY, 'g');
    plot(time, accZ, 'b');
    plot(time, acc_magFilt, ':k');
    plot(time, stationary, 'k', 'LineWidth', 2);
    title('Accelerometer');
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    legend('X', 'Y', 'Z', 'Filtered', 'Stationary');
    hold off;
linkaxes(ax,'x');

% -------------------------------------------------------------------------
% Compute orientation
    acc_ave=mean(accel);
    accel_unit=acc_ave ./ norm(acc_ave);
    D = -accel_unit;
    E = cross(D, mag_unit);
    E = E ./ norm(E);
    N= cross(E, D);
    N = N ./norm(N);
    
    C= [N', E', D']; %BUILD DCM
    Q= quaternion(dcm2quat(C));
quat = quaternion.zeros(length(time), 4);
% AHRSalgorithm = AHRS('SamplePeriod', 1/100, 'Kp', 1, 'KpInit', 1);

% Initial convergence
 initPeriod = 2;
 indexSel = 1 : find(sign(time-(time(1)+initPeriod))+1, 1);
 for i = 1:2000
     AHRSalgorithm.UpdateIMU([0 0 0], [mean(accX(indexSel)) mean(accY(indexSel)) mean(accZ(indexSel))]);
 end
 
 % For all data
 for t = 1:length(time)
     if(stationary(t))
         AHRSalgorithm.Kp = 0.5;
     else
         AHRSalgorithm.Kp = 0;
     end
     AHRSalgorithm.UpdateIMU(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]), [accX(t) accY(t) accZ(t)]);
     quat(t,:) = AHRSalgorithm.Quaternion;
end
% 
% -------------------------------------------------------------------------
% Compute translational accelerations

% Rotate body accelerations to Earth frame
acc = quaternRotate([accX accY accZ], quaternConj(quat));

% % Remove gravity from measurements
 acc = acc - [zeros(length(time), 2) ones(length(time), 1)];     % unnecessary due to velocity integral drift compensation

% Convert acceleration measurements to m/s/s
acc = acc * 9.81;

% Plot translational accelerations
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Accelerations');
hold on;
plot(time, accel(:,1), 'r');
plot(time, accel(:,2), 'g');
plot(time, accel(:,3), 'b');
title('Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s/s)');
legend('X', 'Y', 'Z');
hold off;

% -------------------------------------------------------------------------
% Compute translational velocities

accel(:,3) = accel(:,3) - 9.81;

% Integrate acceleration to yield velocity
vel = zeros(size(accel));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + accel(t,:) * samplePeriod;
    if(stationary(t) == 1)
        vel(t,:) = [0 0 0];     % force zero velocity when foot stationary
    end
end


% Compute integral drift during non-stationary periods
velDrift = zeros(size(vel));
stationaryStart = find([0; diff(stationary)] == -1);
stationaryEnd = find([0; diff(stationary)] == 1);
for i = 1:numel(stationaryEnd)
    driftRate = vel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i));
    enum = 1:(stationaryEnd(i) - stationaryStart(i));
    drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
    velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
end

% Remove integral drift
vel = vel - velDrift;

% Plot translational velocity
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Velocity');
hold on;
plot(time, vel(:,1), 'r');
plot(time, vel(:,2), 'g');
plot(time, vel(:,3), 'b');
title('Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('X', 'Y', 'Z');
hold off;

% -------------------------------------------------------------------------
% Compute translational position

% Integrate velocity to yield position
pos = zeros(size(vel));
for t = 2:length(pos)
    pos(t,:) = pos(t-1,:) + vel(t,:) * samplePeriod;    % integrate velocity to yield position
end
[b,a]=butter(5,[1,10]/(Fs/2));
sdata=filtfilt(b,a,pos);
% Plot translational position
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Position');
hold on;
plot(time, sdata(:,1), 'r');
plot(time, sdata(:,2), 'g');
plot(time, sdata(:,3), 'b');
title('Position');
xlabel('Time (s)');
ylabel('Position (m)');
legend('X', 'Y', 'Z');
hold off;

% -------------------------------------------------------------------------
% Plot 3D foot trajectory
scatter3(sdata(:,1),sdata(:,2),sdata(:,3))
% % Remove stationary periods from data to plot
% posPlot = pos(find(~stationary), :);
% quatPlot = quat(find(~stationary), :);
posPlot = pos;
quatPlot = quat;

% Extend final sample to delay end of animation
extraTime = 20;
onesVector = ones(extraTime*(1/samplePeriod), 1);
posPlot = [posPlot; [posPlot(end, 1)*onesVector, posPlot(end, 2)*onesVector, posPlot(end, 3)*onesVector]];
quatPlot = [quatPlot; [quatPlot(end, 1)*onesVector, quatPlot(end, 2)*onesVector, quatPlot(end, 3)*onesVector, quatPlot(end, 4)*onesVector]];

% Create 6 DOF animation
SamplePlotFreq = 4;
Spin = 120;
SixDofAnimation(posPlot, quatern2rotMat(quatPlot), ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
                'Position', [9 39 1280 768], 'View', [(100:(Spin/(length(posPlot)-1)):(100+Spin))', 10*ones(length(posPlot), 1)], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));
