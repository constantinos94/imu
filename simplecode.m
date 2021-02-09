a = arduino('COM5', 'Uno', 'Libraries', 'I2C');
fs = 100; % Sample Rate in Hz   
imu = mpu9250(a,'SampleRate',fs,'OutputFormat','matrix');

viewer = HelperOrientationViewer;

for i= 1:1000
    [accel,gyro,mag] = readSensorDataMPU9250(imu);
    
    mag_ave=mean(mag);
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
    
    viewer(Q)
    pause(.1)
    
end


figure
quiver3(N,D,E,0.5)