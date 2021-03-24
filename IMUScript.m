clear;
clc;


% -------------------------------------------------------------------------
% Select dataset (comment in/out)
data=xlsread('C:\Users\Dell\Desktop\PhD\IMU\GAIT.xlsx',1);

% -------------------------------------------------------------------------
% Import data
t1=1:length(data(:, 1));
Fs=100;
samplePeriod = 1/100;
time=t1./Fs;

gyrX = data(:, 4);
gyrY = data(:, 5);
gyrZ = data(:, 6);
accX = data(:, 1);
accY = data(:, 2);
accZ = data(:, 3);
magX= data(:, 7);
magY= data(:, 8);
magZ= data(:, 9);

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

quat = zeros(length(time), 4);
AHRSalgorithm = AHRS('SamplePeriod', 1/256, 'Kp', 1, 'KpInit', 1);

% Initial convergence
initPeriod = 2;
indexSel = 1 : find(sign(time-(time(1)+initPeriod))+1, 1);
for i = 1:2000
    AHRSalgorithm.UpdateIMU([0 0 0], [mean(accX(:,:)) mean(accY(:,:)) mean(accZ(:,:))]);
end

% For all datac
for t = 1:length(time)
    if(stationary(t))
        AHRSalgorithm.Kp = 0.5;
    else
        AHRSalgorithm.Kp = 0;
    end
    AHRSalgorithm.UpdateIMU(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]), [accX(t) accY(t) accZ(t)]);
    quat(t,:) = AHRSalgorithm.Quaternion;
end

% -------------------------------------------------------------------------
% Compute translational accelerations

% Rotate body accelerations to Earth frame
acc = quaternRotate([accX accY accZ], quaternConj(quat));

% % Remove gravity from measurements
% acc = acc - [zeros(length(time), 2) ones(length(time), 1)];     % unnecessary due to velocity integral drift compensation

% Convert acceleration measurements to m/s/s
acc = acc * 9.81;

% Plot translational accelerations
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Accelerations');
hold on;
plot(time, acc(:,1), 'r');
plot(time, acc(:,2), 'g');
plot(time, acc(:,3), 'b');
title('Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s/s)');
legend('X', 'Y', 'Z');
hold off;

% -------------------------------------------------------------------------
% Compute translational velocities

acc(:,3) = acc(:,3) - 9.81;

% Integrate acceleration to yield velocity
vel = zeros(size(acc));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + acc(t,:) * samplePeriod;
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

%[b,a]=butter(1,[0.002,10]/(Fs/2));
%velz=filtfilt(b,a,vel(:,:));
%vel=velz;

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

% Plot translational position
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Position');
hold on;
plot(time, pos(:,1), 'r');
plot(time, pos(:,2), 'g');
plot(time, pos(:,3), 'b');
title('Position');
xlabel('Time (s)');
ylabel('Position (m)');
legend('X', 'Y', 'Z');
hold off;


plot3(pos(:,1),pos(:,2),pos(:,3))
title('Position in 3D');
xlabel('meters');

[c,d]=butter(3,[0.002,10]/(120));
sdata=filtfilt(c,d,pos);

%[b,a]=butter(2,(0.02)/(Fs/2), 'high');
%sdata=filtfilt(b,a,pos);

%[b,a]=butter(2,(10)/(Fs/2), 'low');
%sdata1=filtfilt(b,a,sdata);


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
plot3(sdata(:,1),sdata(:,2),sdata(:,3))
title('Position');
xlabel('meters');

% % Remove stationary periods from data to plot
% posPlot = pos(find(~stationary), :);
% quatPlot = quat(find(~stationary), :);
posPlot = sdata;
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
