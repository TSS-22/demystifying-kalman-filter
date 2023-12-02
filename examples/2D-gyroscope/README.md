* Check that the example still work from the get go on matlab

%%%%%%%%%%%%% TODO %%%%%%%%%%%%%
% For phi, theta and psi, change them to pitch yaw and roll
% verify that the Euler angles computed from the accelerometer reading are really one of and not some kind of cumulative state aka attitude of the system

%%%%%%%%%%%%%%%%%%% PUT IN THE README
% Explain how to find the state transition model (F)
% Cite the source for the accelerometer to angles reading, and check if this is angular speed or displacement that is calculated 
% Explain why the covariance matrix is of interest in the results
% Explain why the Kalman gain value too
% Add a little bit on quaternions
% how to find omega
% why the normalization of acc values


# EXAMPLE: Kalman filter for in the context of the 2D attitude of a system


# Sources

* BARRETO, Armando, ADJOUADI, Malek, ORTEGA, Francisco, et al. [Intuitive Understanding of Kalman Filtering with MATLAB®](https://www.taylorfrancis.com/books/mono/10.1201/9780429200656/intuitive-understanding-kalman-filtering-matlab%C2%AE-armando-barreto-malek-adjouadi-francisco-ortega-nonnarit-larnnithipong). ISBN-13: 978-0367191337. CRC Press, 2020.
