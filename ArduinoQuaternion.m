clear; clc; close all

cd C:\Users\Andrei\Desktop\Licenta+Lucru\Personale\KalmanFilter\MatlabTracker
port = serialportlist;
a = arduino(port,'Nano3');
fs = 200;
mpu = mpu6050(a,'SampleRate',fs);
i = 1;
figure
a1 = animatedline('Color','red','linew',3);
a3 = animatedline('Color','black','linew',3);
grid on
legend('Pitch','Location','northoutside','fontsize',14)
ylabel('deg')
movegui(gcf,'west')

figure
a2 = animatedline('Color','green','linew',3);
legend('Roll','Location','northoutside','fontsize',14)
ylabel('deg')
movegui(gcf,'east')
grid on

kalman_filter = QuaternionKalmanFilterModel();

phi = 0;
theta = 0;
psi = 0;
Q = 0.09 * eye(4);
R = 0.09 * eye(4);
P = 9 * eye(4);
q1Std = Q(1,1); q2Std = Q(2,2); q3Std = Q(3,3); q4Std = Q(4,4);
a1Std = R(1,1); a2Std = R(2,2); a3Std = R(3,3); a4Std = R(4,4);
init_q1_std = P(1,1); init_q2_std = P(2,2); init_q3_std = P(3,3); init_q4_std = P(4,4);
init_on_measurement = false;

dt = 1/fs;

Accel_Roll = [];
Accel_Pitch = [];

Kalman_Roll = [];
Kalman_Pitch = [];

Gyro_Roll = [];
Gyro_Pitch = [];

for i=1:500
    
    matr = table2array((read(mpu)));
    
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
    
    i = i + 1;
    
    addpoints(a1,i,rad2deg(Roll))
    addpoints(a2,i,rad2deg(Pitch))
    
    Accel_Roll = [Accel_Roll, accel_angle(1)];
    Accel_Pitch = [Accel_Pitch, accel_angle(2)];
    
    Kalman_Roll = [Kalman_Roll, rad2deg(Roll)];
    Kalman_Pitch = [Kalman_Pitch, rad2deg(Pitch)];
    
    Gyro_Roll = [Gyro_Roll, predicted_roll];
    Gyro_Pitch = [Gyro_Pitch, predicted_pitch];
    
    drawnow limitrate
    grid on
end


clearvars -except -regexp ^Accel ^Kalman ^Gyro

cd C:\Users\Andrei\Desktop
if exist('angleData.mat','file')
    delete('angleData.mat')
end

warning('off')
save('angleData.mat');