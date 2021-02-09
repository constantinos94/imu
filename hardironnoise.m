a = arduino('COM5', 'Uno', 'Libraries', 'I2C');
fs = 100; % Sample Rate in Hz   
imu = mpu9250(a,'SampleRate',fs,'OutputFormat','matrix');

num_points = 100;

mag_ave = zeros(num_points, 3);

%Collect calibraation data
for i = 1:num_points
    [accel,gyro,mag] = readSensorDataMPU9250(imu);
    mag_ave(i,:)= mean (mag);
    pause(.2);
end

%Distorted ellipsoid
scatter3(mag_ave(:,1), mag_ave(:,2), mag_ave(:,3));
hold all
 
 %find callibration coefficients
[A,b,expMFS] = magcal(mag_ave);
xCorrected= (mag_ave-b)*A;
 
 % calibrated allipsoid
scatter3(xCorrected(:,1),xCorrected(:,2),xCorrected(:,3));

display(A);
display(b);
