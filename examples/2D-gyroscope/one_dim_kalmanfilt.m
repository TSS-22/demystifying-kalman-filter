% Why oned ? one dimension ?
%% Single discrete time Kalman filter iteration
% Explain all the parameters below
% What is process ?
% Which models ?
% Why covariance matrix on the noise ?
% Where is the trust information ?
% F, G = model equations
%    Q = process noise covariance matrix
%    H = matrix for the measurement equation
%    R = measurement noise covariance matrix
%    
% During each iteration the following parameters changes ==>
% What does state vector and its covariance matrix mean ?
%
%   from the previous iteration of the algorithm:
%       x = state vector
%       P = covariance matrix of the state vector
% 
%   from (newly?) received parameters:
%       u = vector input (current)
%       z = measurements vector (current)
% 
% From this parameters, the filter perform two steps:
%   - Prediction
%   - Correction
% 
% Then it returns:
% find a better way to express the idea behind POSTERIOR
%   xA = posterior estimation of the state vector
%   PA = covariance matrix of the posterior estimation of the state vector
%   KG = KG matrix (what is it?)
%
% Is the posterior estimation the results that needs to be used as the new
% state vector ? (aka here, inclination/ angular movement/angular
% displacement, whatever we measure and try to filter?)
%

function [PA, xA, KG] = one_dim_kalmanfilt(F, G, Q, H, R, P, x, u, z)
%% PREDICTION PHASE
% Using the MODEL (which one?)
% eq 8.1
% The MODEL predicts the new state vector (x)
%
% What is F and FT?
% Which model?
FT = transpose(F);
% What is G, u and xM?
xM = F * x + G * u;

% eq 8.2
% The MODEL predicts the new covariance matrix (P) of the state vector (x)
% What is PM, F, P, Q?
PM = F * P * FT + Q;


%% PHASE TRANSITION
% we change the state vector (x) and its covariance matrix (P) by their
% newly MODEL predicted values
xB = xM; % eq 8.3
PB = PM; % eq 8.4


%% CORRECTION/UPDATE PHASE
% The goal here, is to find posterior parameters values (A = after)
% from the anterior (B = before) parameters values,
% through Bayesian estimation
%
% What is HT, H?
HT = transpose(H);

% 1- Calculate the Kalman gain (KG)
% What is it the Kalman gain ?
%
% eq 8.5
KG = PB * HT * (inv(H * PB * HT + R));

% 2- Calculate POSTERIOR ESTIMATE of the state vector (x)
%
% eq 8.6
xA = xB + KG * (z - H * xB);

% 3- Calculate POSTERIOR ESTIMATE of state vector (x) covariance matrix (P)
%
% eq 8.7
PA = PB - KG * H * PB;


end