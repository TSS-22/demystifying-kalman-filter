%% Single discrete time Kalman filter iteration

%% INPUT PARAMETERS
% x_t : State vector input
% P_t : Covariance matrix of the state vector input
% F : State transition model
% B : Control matrix 
% u : Control vector
% Q : Covariance matrix of the process noise
% H : Measurement matrix
% R : Covariance matrix of the sensor noise
% z : Measurement vector

%% OUTPUT PARAMETERS
% x_c : Predicted and corrected state vector
% P_c : Covariance matrix of the predicted and corrected state vector (x_c)
%   K : Kalman gain

function [x_c, P_c, K] = one_dim_kalmanfilt(x_t, P_t, F, B, u, Q, H, R, z)
%% PREDICTION PHASE
% eq (1)
% We predict the new state vector (x_p) from the state transition model (F) applied to the state vector at t (x_t),
% and the control vector (u) and the associated control matrix (B)
x_p = F * x_t + B * u;

% eq (2)
% We predict the covariance matrix (P_p) of the predicted state vector (x_p),
% using the state transition model (F), the covariance matrix (P_t) of the state vector at t (x_t),
% and the covariance matrix of the process noise (Q)
P_p = F * P_t * transpose(F) + Q;

%% CORRECTION/UPDATE PHASE
% eq (3)
% We calculate the Kalman gain (K), using the covariance matrix (P_p) of the predicted state vector (x_p),
% the measurement matrix (H), and the covariance matrix of the sensor noise (R)   
K = P_p * transpose(H) * (inv(H * P_p * transpose(H) + R));

% eq (4)
% We calculate the output state vector (x_c) of the present Kalman filter iteration,
% by correcting the predicted state vector (x_p) with measurement vector (z), and the measurement matrix
% to which the the Kalman gain (K) was applied  
x_c = x_p + K * (z - H * x_p);

% eq (5)
% We calculate the covariance matric (P_c) of the corrected estimated state vector, 
% which is the state vector at t+1 (x_c),
% using the covariance matrix (P_p) of the predicted state vector (x_p),
% the Kalman gain (K) and the measurement matrix (H) 
P_c = P_p - K * H * P_p;

end