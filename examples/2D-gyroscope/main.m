close all
clear all
%%%%%%%%%%%%% TODO %%%%%%%%%%%%%
% change dt to actual dt values for each steps, start on the second sample as t+1, so you can get t1-t0 for the dt
% how to find omega
% why the normalization of acc values
% Explain how to find the state transition model (F)
% Cite the source for the accelerometer to angles reading, and check if this is angular speed or displacement that is calculated 
% For phi, theta and psi, change them to pitch yaw and roll
% Explain why the covariance matrix is of interest in the results
% Explain why the Kalman gain value too
% Stay simple in the explanation and if it is more than one line, just put it in the README.md
% For example, for the angles calculation, and the quaternions
% verify that the Euler angles computed from the accelerometer reading are really one of and not some kind of cumulative state aka attitude of the system
% Make sure the example still works

%% FOREWORDS AND DISCLAIMER
% This example is an example present in the book Intuitive Understanding of Kalman Filtering with MATLAB®
% Some of the values have been chosen arbitrarily by the authors, and when this was the case,
% This is stipulated.
% The whole example has been rewritten in a style consistent with our explanation of the Kalman filter,
% but also clarified a lot compared to the original version

%% PARAMETER NAMES
% x : State vector
% P : Covariance matrix of the state vector
% F : State transition model
% B : Control matrix 
% u : Control vector
% Q : Covariance matrix of the process noise
% H : Measurement matrix
% R : Covariance matrix of the sensor noise
% z : Measurement vector
% K : Kalman gain

%% INITIALISATION
% The matrix value below have been selected ad hoc by the author of the book
Q = 0.0001*eye(4); % Covariance matrix of the process noise
R = 0.5 * eye(4); % Covariance matrix of the sensor noise
P0 = 0.01 * eye(4); % Initial value of the covariance matrix of the state vector

%% OPEN AND LOAD DATA
data = csvread("data\data.csv");

timeStamps = data(:,1);
stillness = data(:,2); % This seems to be linked to the overall level of the movement of the IMU
data_gyr = data(:,3:5);
data_acc = data(:,6:8);
data_mag = data(:,13:15);
data_quat = data(:,9:12); % Could be the state of the IMU in space, or the gyroscope rotation in a quaternion form

%% PREPARE DATA FOR FILTERING
nbSamples = length(timeStamps); % Sample number
acqFreq = nbSamples/timeStamps(end); % acquisition frequency
% Do not use this dt
dt = 1/acqFreq; % THIS IS WRONG, dt is inconsistent

% The author normalized the accelerometer data
% it might be for the quaternion transformation
for i=1:1:nbSamples 
    data_acc_norm(i,:) = data_acc(i,:)./norm(data_acc(i,:));
end

%% KALMAN FILTER INITIALISATION
H = eye(4); % Measurement matrix
x = [1 0 0 0]'; % Initial state vector
P = P0; % Associated covariance matrix of the state vector
% As there are no control input in our system,
% Control vector and associated Control matrix are set to zero
u = zeros(nbSamples,1);
B = [0; 0; 0; 0];

% Storing matrix for
%               store_X : predicted state vector 
%               store_P : associated covariance matrix (diagonal only)
%               store_K : Computed Kalman gain (diagonal only)
%          store_angles : Angles from accelerometer readings
% store_filtered_angles : Angles outputed by the Kalman filter
store_X = zeros(4,nbSamples); 
save_P = zeros(4,nbSamples);
store_K = zeros(4,nbSamples); 
store_angles = zeros(nbSamples,3);
store_filtered_angles = zeros(nbSamples,3);

%% KALMAN FILTER
for t=1:1:nbSamples
    dt = 1/acqFreq; %%%%%%%%%%% CHANGE THAT TO REAL VALUE
    % Gyroscope data at time t
    gyr_x = -data_gyr(t,1);
    gyr_y = data_gyr(t,2);
    gyr_z = -data_gyr(t,3);
    
    % How to find Omega 
    omega_mat = [
      0     -gyr_z  -gyr_x  -gyr_y
      gyr_z 0       gyr_y   -gyr_x
      gyr_x -gyr_y  0       gyr_z
      gyr_y gyr_x   -gyr_z  0
    ];

    % Build the state transition model from the gyroscope reading 
    F = eye(4) + (dt/2) * omega_mat;

    % Calculate Euler angles from accelerometer measurement
    % Verify the models : is it the angular velocity or the angular
    % displacement that is calculated?
    acc_angle_x = data_acc_norm(t,3);
    acc_angle_y = data_acc_norm(t,1);

    theta = asin(acc_angle_x);
    phi = asin(-acc_angle_y/(cos(theta)));
    psi = 0;

    % Convert Euler angles into Quaternion to create the measurement vector
    z = [
        cos(phi/2) * cos(theta/2) * cos(psi/2) +...
            sin(phi/2)*sin(theta/2)*sin(psi/2);
        
        sin(phi/2) * cos(theta/2) * cos(psi/2) -...
            cos(phi/2)*sin(theta/2)*sin(psi/2);
        
        cos(phi/2) * sin(theta/2) * cos(psi/2) +...
            sin(phi/2)*cos(theta/2)*sin(psi/2);
        
        cos(phi/2) * cos(theta/2) * sin(psi/2) -...
            sin(phi/2) * sin(theta/2) * cos(psi/2)
    ];

    % KALMAN FILTER ITERATION
    [P_c, x_c, K] = one_dim_kalmanfilt(F,G,Q,H,R,P,x,u(t),z);
    
    % Clarify that step
    filtered_phi = atan2(2*(x_c(3)*x_c(4) + x_c(1)*x_c(2)) ,...
                    1-2*(x_c(2)^2 + x_c(3)^2));
    filtered_theta = -asin(2*(x_c(2)*x_c(4) - x_c(1)*x_c(3)));
    filtered_psi = atan2( 2*(x_c(2)*x_c(3) + x_c(1)*x_c(4)) ,...
                    1-2*(x_c(3)^2 + x_c(4)^2));

    % Explain the steps and explain why /2
    filtered_quaternion(t,:) = [
        cos(filtered_phi/2) * cos(filtered_theta/2) * cos(filtered_psi/2) +...
            sin(filtered_phi/2) * sin(filtered_theta/2) * sin(filtered_psi/2);
        
    	sin(filtered_phi/2) * cos(filtered_theta/2) * cos(filtered_psi/2) -...
            cos(filtered_phi/2) * sin(filtered_theta/2) * sin(filtered_psi/2);
        
    	cos(filtered_phi/2) * sin(filtered_theta/2) * cos(filtered_psi/2) +...
            sin(filtered_phi/2) * cos(filtered_theta/2) * sin(filtered_psi/2);
        
    	cos(filtered_phi/2) * cos(filtered_theta/2) * sin(filtered_psi/2) -...
            sin(filtered_phi/2) * sin(filtered_theta/2) * cos(filtered_psi/2)
    ];

    % Store the current covariance matrix (P) of the state vector (x) diagonal
    store_P(1:4,t) = diag(P_c);
    % Store the current Kalman gain diagonal
    store_K(1:4,t) = diag(K);
    % Store the angles from the accelerometer readings
    % Clarify if it is in radian or deg
    store_angles(t,:) = (180/pi) * [phi, theta, psi];
    % Store the Kalman filtered angles from the accelerometer readings
    filtered_PTP(t,:) = (180/pi) * [filtered_phi, filtered_theta, filtered_psi];

    % The newly calculated state vector (x) and associated covariance matrix (P), 
    % will be used as the input for the next iteration of the Kalman filter
    x = x_c;
    P = P_c;
    
end

% Save a matrix with all the output state vectors from the Kalman filter
store_X = transpose(filtered_quaternion);

%% DISPLAY RESULTS

% Diagonal of the covariance matrix (P) of the state vector (x)
% estimated by the Kalman filter
figure; plot(transpose(store_P),'Linewidth',1.5);
grid; title('Values on the diagonal of P_c');
xlabel('Kalman filter iterations')

% Diagonal of the Kalman gain matrix (K)
figure; plot(transpose(store_K),'Linewidth',1.5);
grid; title('Values on the diagonal of K');
xlabel('Kalman filter iterations')

% Initialize subplot figure
gray6 = [0.6, 0.6, 0.6];
figure;

% Gyroscope measurement
% Angula speed in radian per seconds
sp(1)=subplot(4,1,1); hold on;

plot(timeStamps,-data_gyr(:,3),'Color',gray6,'Linewidth',1);
plot(timeStamps,-data_gyr(:,1),'k--','Linewidth',1);
plot(timeStamps,data_gyr(:,2),'r','Linewidth',1);

title('Gyroscope Measurement');
ylabel('Ang Vel(rad/s)');
axis([timeStamps(1) timeStamps(end) -3 3]);
set(gca,'Xtick',0:2:50);
legend('Roll','Pitch','Yaw','Location','BestOutside');

% Euler angles of the displacement computed from the accelerometer values
% Angles in degree
grid on;
sp(2)=subplot(4,1,2); hold on;

plot(timeStamps,store_angles(:,1),'Color',gray6,'Linewidth',1.5);
plot(timeStamps,store_angles(:,2),'k--','Linewidth',1.5);

title('Euler Angles from Accelerometer Measurement');
ylabel('Angles (deg)');
axis([timeStamps(1) timeStamps(end) -180 180]);
set(gca,'Ytick',-180:45:180,'Xtick',0:2:50);
legend('Phi(\phi)','Theta(\theta)','Location','BestOutside');

% Euler angles of the displacement computed from the accelerometer
% and filtered by the Kalman filter
grid on;
sp(3)=subplot(4,1,3); hold on;

plot(timeStamps,filtered_PTP(:,1),'Color',gray6,'Linewidth',1.5);
plot(timeStamps,filtered_PTP(:,2),'k--','Linewidth',1.5);

title('Euler Angles Output from Kalman Filter');
xlabel('Time (seconds)');
ylabel('Angles (deg)');
axis([timeStamps(1) timeStamps(end) -180 180]);
set(gca,'Ytick',-180:45:180,'Xtick',0:2:50);
legend('Phi(\phi)','Theta(\theta)','Location','BestOutside');

% The state of the system as estimated and corrected by the Kalman filter
grid on;
sp(4)=subplot(4,1,4); hold on;

plot(timeStamps,filtered_quaternion(:,1),'b','Linewidth',1);
plot(timeStamps,filtered_quaternion(:,2),'k--','Linewidth',1);
plot(timeStamps,filtered_quaternion(:,3),'Color',gray6,'Linewidth',1);
plot(timeStamps,filtered_quaternion(:,4),'r','Linewidth',1);

title('Quaternion Output from Kalman Filter');
xlabel('Time (seconds)');
axis([timeStamps(1) timeStamps(end) -1.1 1.1]);
set(gca,'Xtick',0:2:50);
legend('w','x','y','z','Location','BestOutside');

grid on;
linkaxes(sp,'x');
