
addpath('codes');
filePath = 'Datasets/straight';
[~,imu]=xlsfinfo(filePath);
for k=1:numel(imu)
  data{k}=xlsread(filePath,imu{k});
end

for k = 1:size(data,1)
    imu = data{k,1};  
end

