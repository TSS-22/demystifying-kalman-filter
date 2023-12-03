%%%%%%%%%%%%% TODO %%%%%%%%%%%%%
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

# Start up

First, we close all figure and clear all variables in the workspace to be sure we don't face any intereferences.

# Initialisation

The covariance of the matrix noise $\mathbf{Q}$ is initialised with the value: 

$$
\mathbf{Q} = 
\begin{pmatrix}
0.0001 & 0 & 0 & 0\\
0 & 0.0001 & 0 & 0\\
0 & 0 & 0.0001 & 0\\
0 & 0 & 0 & 0.0001\\
\end{pmatrix}
$$

It represent the uncertainty associated with the state transition model $\mathbf{F}$. The values chosen by the authors are relatively low, and represent a good trust in the state transition model $\mathbf{F}$ based on the gyroscope measurements. 

All off-diagonal elements were set to $0$, and the diagonal elements were set to be equals, as the authors didn't had any guidelines to follow and didn't had a clear model of the sytsem and associated noise. This was also the building framework for the covariance matrix of the state vector, $\mathbf{P}(0)$, and covariance matrix of the sensor noise, $\mathbf{R}$ values too.

The initial value of the covariance matrix of the state vector, $\mathbf{P}(0)$, is set to:

$$
\mathbf{P}(0) = 
\begin{pmatrix}
0.001 & 0 & 0 & 0\\
0 & 0.001 & 0 & 0 \\
0 & 0 & 0.001 & 0 \\
0 & 0 & 0 & 0.001\\
\end{pmatrix}
$$

The initial value of the state vector, $\vec{x}(0)$, is set to:

$$
\vec{x}(0)=
\begin{pmatrix}
1\\
0\\
0\\
0\\
\end{pmatrix}
$$

The state vector $\vec{x}$ is put in a quaternion form for precision reasons and mathematical form convenience. The first value is the magnitude of the quaternion, and then the three other values represent the rotations on $\hat{x}$, $\hat{y}$ and $\hat{z}$.

The covariance matrix of the sensor noise $\mathbf{R}$ was set to:

$$
\mathbf{R} = 
\begin{pmatrix}
0.5 & 0 & 0 & 0\\
0 & 0.5 & 0 & 0\\
0 & 0 & 0.5 & 0\\
0 & 0 & 0 & 0.5\\
\end{pmatrix}
$$

The value of the covariance matrix of the sensor noise $\mathbf{R}$ is set higher than for the $\mathbf{Q}$ and $\mathbf{P_0}$, as we have a higher trust in those values than in the angles measurement values calculated from the accelerometer values.

# Finding $\mathbf{F}$



# Finding $\vec{z}$



# Results

![Diagonal value of P through time](img/diag_p.png)

![Diagonal value of K through time](img/diag_k.png)

![Angles values of the system through time](img/full_fig.png)

# Sources

* BARRETO, Armando, ADJOUADI, Malek, ORTEGA, Francisco, et al. [Intuitive Understanding of Kalman Filtering with MATLAB®](https://www.taylorfrancis.com/books/mono/10.1201/9780429200656/intuitive-understanding-kalman-filtering-matlab%C2%AE-armando-barreto-malek-adjouadi-francisco-ortega-nonnarit-larnnithipong). ISBN-13: 978-0367191337. CRC Press, 2020.