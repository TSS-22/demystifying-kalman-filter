close all
clear all
%%%%%%%%%%%%% TODO %%%%%%%%%%%%%
% change dt to actual dt values for each steps
% in one_dim_kalmanfilter(), find a better way to express the xA, xB, xM
% everywhere, find a better way to express the one letter variables, for
% example using their actual name: deconfuse this shit

%% INITIALISATION PARAMETERS
% FIND MATHEMATICAL SIGNIFICATION
Q = 0.0001*eye(4); % find what are this
R = 0.5 * eye(4); % find what are this
P0 = 0.01 * eye(4); % find what are this

%% OPEN AND LOAD DATA
data = csvread("data\data129.csv");

timeStamps = data(:,1);
stillness = data(:,2); % what is this shit?
data_gyr = data(:,3:5);
data_acc = data(:,6:8);
data_mag = data(:,13:15);
data_quat = data(:,9:12); % What is it and howdid he got it ? can't I just find it ? Can I get it from the IMU directly ?

%% PREPARE DATA FOR FILTERING
nbSamples = length(timeStamps);
acqFreq = nbSamples/timeStamps(end); % acquisition frequency
% Do not use this dt
dt = 1/acqFreq; % THIS IS WRONG, dt is inconsistent

% Normalisation of acc data, FIND WHY
for i=1:1:nbSamples 
    data_acc_norm(i,:) = data_acc(i,:)./norm(data_acc(i,:));
end

%% KALMAN FILTER INITIALISATION
% Understand this parameters and why their values
H = eye(4); % What is it?
x = [1 0 0 0]'; % What is it?
P = P0; % What is it?
u = zeros(nbSamples,1); % If no control input
PAd = zeros(4,nbSamples); %Storage for all PA matrix diagonals
%xAm = zeros(4,nbSamples); %Storage for all posterior state vect.
KGd = zeros(4,nbSamples); %Storage for all KG matrix diagonals

% Matrix to store angles calculated directly from accellerometer readings
% Phi, Theta and Psi (find corresponding axis)
PTP = zeros(nbSamples,3);

% Matrix to store angles obtained from the quaternion output of KF
% Phi, Theta and Psi
% What is KF
filtPTP = zeros(nbSamples,3);

%% KALMAN FILTER
for t=1:1:nbSamples
    dt = 1/acqFreq; %%%%%%%%%%% CHANGE THAT TO REAL VALUE
    % THE AXIS X, Y and Z MUST BE WRONG
    % FIND THE CORRESPONDANCE BETWEEN (1,2,3) and (x,y,z)
    % Why x and z axis have a minus sign?
    gyr_x = -data_gyr(t,1);
    gyr_y = data_gyr(t,2);
    gyr_z = -data_gyr(t,3);
    
    % How to find Omega, F and G, and what they mean
    omega_mat = [
      0     -gyr_z  -gyr_x  -gyr_y
      gyr_z 0       gyr_y   -gyr_x
      gyr_x -gyr_y  0       gyr_z
      gyr_y gyr_x   -gyr_z  0
    ];
    F = eye(4) + (dt/2) * omega_mat;
    G = [0; 0; 0; 0];

    % Calculate Euler angles from accelerometer measurement
    % Verify the models : is it the angular velocity or the angular
    % displacement that is calculated?
    acc_angle_x = data_acc_norm(t,3);
    acc_angle_y = data_acc_norm(t,1);

    theta = asin(acc_angle_x);
    phi = asin(-acc_angle_y/(cos(theta)));
    % What is psi and the "2" values? and why do we need them?
    psi = 0;
    theta2 = theta/2;
    phi2 = phi/2;
    psi2 = psi/2; % This one is always zero, why is it there

    % Convert Euler angles into Quaternion
    % What is z matrix and why is it like that?
    z = [
        % q0 ?
        cos(phi2) * cos(theta2) * cos(psi2) +...
            sin(phi2)*sin(theta2)*sin(psi2);
        % q1 ?
        sin(phi2) * cos(theta2) * cos(psi2) -...
            cos(phi2)*sin(theta2)*sin(psi2);
        % q2 ?
        cos(phi2) * sin(theta2) * cos(psi2) +...
            sin(phi2)*cos(theta2)*sin(psi2);
        % q3 ?
        cos(phi2) * cos(theta2) * sin(psi2) -...
            sin(phi2) * sin(theta2) * cos(psi2)
    ];

    % Kalman iteration
    % REWRITE THE KALMAN STEP INTO A NICER FUNCTION AND ADD IT HERE
    % What is PA, xA and KG?
    % What is F, G, Q, H, R, O, x, u(t), z?
    [PA, xA, KG] = one_dim_kalmanfilt(F,G,Q,H,R,P,x,u(t),z);
    % Why did he put this ?
    %[PA, xA, KG] =onedkfkg0(F,G,Q,H,R,P,x,u(t),z); with KG=0 
    
    PAd(1:4,t) = diag(PA); % stores current Diag. of PA 
    KGd(1:4,t) = diag(KG); % stores current Diag. of KG 
    
    % The posterior estimate will be used for the following steps
    % Reformulate the previous sentence and clarify what's happening
    x = xA;
    P = PA;
    
    % Clarify that step (I think this is the position angle after filtering
    % which we will use in the following steps (or not as I don't see them anywhere)
    % why atan take two parameters and asin/acos only one?
    filtered_phi = atan2(2*(x(3)*x(4) + x(1)*x(2)) ,...
                    1-2*(x(2)^2 + x(3)^2));
    filtered_theta = -asin(2*(x(2)*x(4) - x(1)*x(3)));
    filtered_psi = atan2( 2*(x(2)*x(3) + x(1)*x(4)) ,...
                    1-2*(x(3)^2 + x(4)^2));

    % Storing reults for display after completion of the loop
    % What is PTP?
    PTP(t,:) = (180/pi) * [phi, theta, psi];
    filtered_PTP(t,:) = (180/pi) * [filtered_phi, filtered_theta, filtered_psi];

    filtered_phi2 = filtered_phi/2;
    filtered_psi2 = filtered_psi/2;
    filtered_theta2 = filtered_theta/2;

    filtered_quaternion(t,:) = [
        % q0 filt ?
        cos(filtered_phi2) * cos(filtered_theta2) * cos(filtered_psi2) +...
            sin(filtered_phi2) * sin(filtered_theta2) * sin(filtered_psi2);
        % q1 filt ?
        sin(filtered_phi2) * cos(filtered_theta2) * cos(filtered_psi2) -...
            cos(filtered_phi2) * sin(filtered_theta2) * sin(filtered_psi2);
        % q2 filt ?
        cos(filtered_phi2) * sin(filtered_theta2) * cos(filtered_psi2) +...
            sin(filtered_phi2) * cos(filtered_theta2) * sin(filtered_psi2);
        % q3 filt ?
        cos(filtered_phi2) * cos(filtered_theta2) * sin(filtered_psi2) -...
            sin(filtered_phi2) * sin(filtered_theta2) * cos(filtered_psi2)
    ];

end

% Save a matrix with all the posterior state vectors
% What does that mean? What is fQuat and xAm?
xAm = transpose(filtered_quaternion);

%% DISPLAY RESULTS ATT2LOOP.M
gray6 = [0.6, 0.6, 0.6];
figure;
sp(1)=subplot(4,1,1); hold on;

plot(timeStamps,-data_gyr(:,3),'Color',gray6,'Linewidth',1);
plot(timeStamps,-data_gyr(:,1),'k--','Linewidth',1);
plot(timeStamps,data_gyr(:,2),'r','Linewidth',1);

title('Gyroscope Measurement');
ylabel('Ang Vel(rad/s)');
axis([timeStamps(1) timeStamps(end) -3 3]);
set(gca,'Xtick',0:2:50);
legend('Roll','Pitch','Yaw','Location','BestOutside');

grid on;
sp(2)=subplot(4,1,2); hold on;

plot(timeStamps,PTP(:,1),'Color',gray6,'Linewidth',1.5);
plot(timeStamps,PTP(:,2),'k--','Linewidth',1.5);

title('Euler Angles from Accelerometer Measurement');
ylabel('Angles (deg)');
axis([timeStamps(1) timeStamps(end) -180 180]);
set(gca,'Ytick',-180:45:180,'Xtick',0:2:50);
legend('Phi(\phi)','Theta(\theta)','Location','BestOutside');

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


%% DISPLAY RESULTS MAIN
figure; plot(transpose(PAd),'Linewidth',1.5);
grid; title('Values on the diagonal of PA');
xlabel('KF iterations')

figure; plot(transpose(KGd),'Linewidth',1.5);
grid; title('Values on the diagonal of KG');
xlabel('KF iterations')

