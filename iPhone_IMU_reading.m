function [data_IMU,data_lables,t] = iPhone_IMU_reading(filename,frq,l_start,l_end,showfig)
R = 1; %start from second column to avaiod the test
C = 0; %srat from first row
data = csvread(filename,R,C);
data_IMU = data(l_start:l_end,:);
data_lables = {'TimeStamp'	'AccX'	'AccY'	'AccZ'	'GravX'	'GravY'	'GravZ'	'MagAccuracy'	'MagX'	'MagY'	'MagZ'	'AttPitch'	'AttYaw' 'AttRoll'	'rotX'	'rotY'	'rotZ'};
t = data_IMU(:,1) - data_IMU(1,1);

%check for time jump
if showfig
    figure(11)
    plot(t(2:end) - t(1:end-1)),title('Time Jump'),xlabel('time(s)'),ylabel('time sample difference(s)'), grid on
end

%ploting accelometer and attitude data
if showfig
    figure(10)
    No = 6; %number of signals to be plotted
    for id = 1:3
        subplot(No,1,id),plot(t,data_IMU(:,id+1)),xlabel('time(s)'),ylabel('m/s2'),title(data_lables{id+1}), grid on
    end
    for idx = 15:17
        subplot(No,1,idx-11),plot(t,data_IMU(:,idx)*180/pi),xlabel('time(s)'),ylabel('degree/s'),title(data_lables{idx}), grid on
    end
end
end