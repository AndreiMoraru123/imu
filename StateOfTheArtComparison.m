clc; close all;
clearvars -except -regexp ^Accel ^Kalman ^Gyro

% openExample('shared_fusion_arduinoio/EstimateOrientationUsingInertialSensorFusionAndMPU9250Example');

cd C:\Users\Andrei\Desktop\Licenta+Lucru\Personale\KalmanFilter\MatlabTracker
port = serialportlist;
a = arduino(port,'Nano3');
fs = 200;
mpu = mpu6050(a,'SampleRate',fs);
i = 1;

f = figure('NumberTitle', 'off', 'Name', 'Unghiuri');
f.Position = [0 0 825 550];
ax1 = subplot(2,1,1);
a1 = animatedline('Color','g','linew',3);
a3 = animatedline('Color','black','linew',3);
gyro_only = animatedline('Color',[0.9290 0.6940 0.1250],'linew',2,'LineStyle',':');
accel_only = animatedline('Color','r','linew',2,'LineStyle',':');
butter_roll = animatedline('Color','blue','linew',2);
grid on
leg1 = legend('filtru propus','filtru MATLAB','giroscop','accelerometru','butterworth + KF','Location','eastoutside','fontsize',15);
ylabel('tangaj (grade)')
xlabel('timp (secunde)')

ax2 = subplot(2,1,2);
a2 = animatedline('Color','g','linew',3);
a4 = animatedline('Color','black','linew',3);
gyro_only2 = animatedline('Color',[0.9290 0.6940 0.1250],'linew',2,'LineStyle',':');
accel_only2 = animatedline('Color','r','linew',2,'LineStyle',':');
butter_pitch = animatedline('Color','blue','linew',2);
ylabel('ruliu (grade)')
xlabel('timp (secunde)')

grid on

kalman_filter = QuaternionKalmanFilterModel();

phi = 0;
theta = 0;
psi = 0;
Q = 0.009 * eye(4);
R = 0.009 * eye(4);
P = 9 * eye(4);
q1Std = Q(1,1); q2Std = Q(2,2); q3Std = Q(3,3); q4Std = Q(4,4);
a1Std = R(1,1); a2Std = R(2,2); a3Std = R(3,3); a4Std = R(4,4);
init_q1_std = P(1,1); init_q2_std = P(2,2); init_q3_std = P(3,3); init_q4_std = P(4,4);
init_on_measurement = false;

FUSE = imufilter('SampleRate',200);

cd C:\Users\Andrei\Documents\MATLAB\Examples\R2020b\shared_fusion_arduinoio\EstimateOrientationUsingInertialSensorFusionAndMPU9250Example

myviewer = HelperOrientationViewer;
mine = gcf; mine.Name = 'Propus';
movegui('southeast')

viewer = HelperOrientationViewer;
matrixlab = gcf; matrixlab.Name = 'MATLAB';
movegui('northeast')

cd C:\Users\Andrei\Desktop\Licenta+Lucru\Personale\KalmanFilter\MatlabTracker

dt = 1/fs;

nyquist = fs/2;

roll_bias = mean(Kalman_Roll);
pitch_bias = mean(Kalman_Pitch);

[b, a] = butter(1, 10/nyquist);
z_roll = deg2rad(roll_bias);
z_pitch = deg2rad(pitch_bias);

% predicted_bias_roll = [];
% predicted_bias_pitch = [];
% 
% matlab_roll_vec = [];
% matlab_pitch_vec = [];
% 
% my_roll_vec = [];
% my_pitch_vec = [];
% 
% filtered_roll_vec = [];
% filtered_pitch_vec = [];
% 
% predicted_roll_vec = [];
% predicted_pitch_vec = [];
% 
% accel_roll_vec = [];
% accel_pitch_vec = [];

tic 

while(true)
    
    reader = read(mpu);
    matr = table2array(reader);
    
    accel = matr(1:end,1:3);
    gyro = matr(1:end,4:6);
    
    acc_x = accel(end,1);
    acc_y = accel(end,2);
    acc_z = accel(end,3);
    
    gx = rad2deg(gyro(end,2));
    gy = rad2deg(gyro(end,1));
    gz = rad2deg(gyro(end,3));
    
    gyro_x = gx + gy * tand(theta) * sind(phi) + gz * tand(theta) * cosd(phi);
    gyro_y = gy * cosd(phi) - gz * sind(phi);
    gyro_z = gz;
    
    omega_body = ([gyro_x; gyro_y; gyro_z]);
    
    acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
    angle_Roll_acc = (asind(double(acc_y)/acc_total_vector));
    angle_Pitch_acc = -(asind(double(acc_x)/acc_total_vector));
    accel_angle = [angle_Roll_acc; angle_Pitch_acc];
%     accel_angle = [atan2d(acc_y,acc_z); atan2d(-acc_x, sqrt(acc_y*acc_y + acc_z*acc_z))];
    
    if (i == 1)        
        kalman_filter.initialise(q1Std, q2Std, q3Std, q4Std, a1Std, a2Std, a3Std, a4Std, init_on_measurement,...
                                init_q1_std, init_q2_std, init_q3_std, init_q4_std);
        Roll = (asin(double(acc_y)/acc_total_vector)); %atan2(acc_y,acc_z);
        Pitch = -(asin(double(acc_x)/acc_total_vector)); %atan2(-acc_x, sqrt(acc_y*acc_y + acc_z*acc_z));
        predicted_roll = Roll;
        predicted_pitch = Pitch;
        filtered_Roll = 0;
        filtered_Pitch = 0;
        
    else        
        kalman_filter.prediction(omega_body,dt);
        predicted_angle = quat2eul(kalman_filter.state');
        predicted_roll = rad2deg(predicted_angle(3));
        predicted_pitch = rad2deg(predicted_angle(2));
        kalman_filter.update(accel_angle);
        real_angle = kalman_filter.attitude_state;
        
        phi = (real_angle(1));
        theta = (real_angle(2));
        psi = (real_angle(3));

        Roll = phi;
        Pitch = theta;
    end
    
    [filtered_Roll, z_roll] = filter(b, a, Roll, z_roll);
    [filtered_Pitch, z_pitch] = filter(b, a, Pitch, z_pitch);
    
    i = i + 1;    
        
    orientation = FUSE(accel,gyro);
    
    cd C:\Users\Andrei\Documents\MATLAB\Examples\R2020b\shared_fusion_arduinoio\EstimateOrientationUsingInertialSensorFusionAndMPU9250Example
    
    myviewer(quaternion(kalman_filter.state'));
    viewer(orientation)
    
    cd C:\Users\Andrei\Desktop\Licenta+Lucru\Personale\KalmanFilter\MatlabTracker
    
    angle_orientation = rad2deg(quat2eul(orientation));
    
    addpoints(a1,toc,rad2deg(Roll) - roll_bias)
    addpoints(a3,toc,double(angle_orientation(end,3)) - mean(Accel_Roll))
    addpoints(gyro_only,toc,predicted_roll - nanmean(Gyro_Roll))
    addpoints(butter_roll,toc,rad2deg(filtered_Roll) - roll_bias)
    addpoints(accel_only,toc,accel_angle(1) - mean(Accel_Roll))
    addpoints(a2,toc,rad2deg(Pitch) - pitch_bias)
    addpoints(a4,toc,double(angle_orientation(end,2))  - mean(Accel_Pitch))
    addpoints(gyro_only2,toc,predicted_pitch - nanmean(Gyro_Pitch))
    addpoints(accel_only2,toc,accel_angle(2) - mean(Accel_Pitch))
    addpoints(butter_pitch,toc,rad2deg(filtered_Pitch) - pitch_bias)
    
    drawnow limitrate
    
%     my_roll_vec = [my_roll_vec, rad2deg(Roll) - roll_bias];
%     my_pitch_vec = [my_pitch_vec, rad2deg(Pitch) - pitch_bias];
%     
%     matlab_roll_vec = [matlab_roll_vec, angle_orientation(end,2) - mean(Accel_Roll)];
%     matlab_pitch_vec = [matlab_pitch_vec, angle_orientation(end,3) - mean(Accel_Pitch)];
%     
%     filtered_roll_vec = [filtered_roll_vec, rad2deg(filtered_Roll) - roll_bias];
%     filtered_pitch_vec = [filtered_pitch_vec , rad2deg(filtered_Pitch) - pitch_bias];
%     
%     predicted_roll_vec = [predicted_roll_vec, -predicted_roll - nanmean(Gyro_Roll)];
%     predicted_pitch_vec = [predicted_pitch_vec , -predicted_pitch - nanmean(Gyro_Pitch)];
%     
%     accel_roll_vec = [accel_roll_vec, -accel_angle(1) - mean(Accel_Roll)];
%     accel_pitch_vec = [accel_pitch_vec , accel_angle(2) - mean(Accel_Pitch)];
end