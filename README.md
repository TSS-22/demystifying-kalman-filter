# The myth, the Kalman, the legend: The Kalman filter

An attempt at demystifying the application of the Kalman filter, in order to stop making it look more complex than it is.

# Table of Content

1. [The goal](#the-goal)
2. [Not the goal](#not-the-goal)
3. [How you can help](#how-you-can-help)
4. [Demystifying the Kalman filter](#demystifying-the-kalman-filter-algorithm)
    1. [Disclaimer](#disclaimer)
5. [Le Kalman filter](#le-kalman-filter)
    1. [Context](#context)
    2. [Steps](#steps)
        1. [Prediction](#prediction)
        2. [Correction/Update](#correction/update)
    3. [Terms definitions](#terms-definitions)
        1. [x : State vector](#x--state-vector)
        2. [P : Covariance matrix of the state vector x](#mathbfp--covariance-matrix-of-the-state-vector-vecx)
        3. [K : Kalman gain](#mathbfk--kalman-gain)
        4. [z : Measurement vector](#vecz--measurement-vector)
        5. [F : State transition model](#mathbff--state-transition-model)
        6. [Q : Covariance matrix of the process noise](#mathbfq--covariance-matrix-of-the-process-noise)
        7. [H : Measurement matrix](#mathbfh--measurement-matrix)
        8. [v : Measurement noise](#vecv--measurement-noise)
        9. [R : Covariance matrix of the sensor noise](#covariance-matrix-of-the-sensor-noise)
        10. [u : Control vector](#vecu--control-vector)
        11. [B : Control matrix](#mathbfb--control-matrix)
        12. [w : External noise, sources of uncertainity](#vecomega--external-noise-sources-of-uncertainity)
6. [Initialisation of the Kalman filter](#initialisation-of-the-kalman-filter)
7. [Diagram summary and meta-example](#diagram-summary-and-meta-example)
8. [How to cite](#how-to-cite)
9. [Sources and recommended reads](#sources-and-recommended-reads)
10. [Legal disclaimer](#legal-disclaimer)



# The goal

The goal is to provide a thorough practical introduction to the application of the Kalman filter to the reader, so as he understand the filter algorithm and can to an extend apply it readily to his use case. The goal is also to provide a clear approach to the Kalman filter as to create a strong fondation on which the dedicated reader will be able to rely in order to dig deeper into more technical ressources on the subject of the Kalman filter.

In the grand scheme of thing, the goal is to help bring useful knowledge to the most people possible. 

Even if we aim to provide an introduction as simple as possible, it will still require investment from the reader, and work on 

# Not the goal

This work does not have as a goal to provide the deep understanding of the Kalman filter, nor about the underlying principles, mainly probability, distribution, Bayesian statistics and so forth. At least, at present, see below.

# How you can help

We invite any reader to open an issue if there is anything unclear, mistakes (be it a typo, a wront equation, whatever), or if a topic related to the Kalman filter needs an explanation. Organically, we are not against trying to dumb down the underlying foundations of the Kalman filter and associated mathematical concepts if there is a demand for it (even just one person).

Now let's get to business.

# Demystifying the Kalman filter algorithm

## Disclaimer

In this work we will refer to the discrete version of the Klaman filter algorithm. As it is hard to explain all the Kalman filter parts in isolation, we strongly invite the reader to go through all the material presented here before coming back again to the part that could be unclear during their first read. For the reader that prefer to have a global view before digging in, we would recommend to read the [Diagram summary and meta-example](#diagram-summary-and-meta-example) part before starting the material as it is written.

This work try to dumb the Klamn filter down to the maximun. It won't go into details, but aims to provide a good understanding of the different parts of the algorithm, and wherever possible, underlying principles. The goal is to allow the maximum of people to try to grasp the concept, even people with limited mathematical background. It will be made as so the reader have very limited need to look at other material while going through this one. In order to do so, steps and terms will explained in a manner that can seems repeating and a bit cumbersome.

The time will be refered to, through the notation $t$. It is to be noted that some text, usually the older ones, will refer to the time using the notation $k$, this will not be the case here.

The following math notation is used in the present work:
|Symbol|Signification|
|---:|:---|
|$\cdot$|Matrix multiplication|
|$\mathbf{M}^{T}$|Transpose of the matrix $\mathbf{M}$|
|$\mathbf{M}^{-1}$|Inverse of the matrix $\mathbf{M}$|
|$\mathbf{M}(t)$|Matrix at time $t$|
|$\sigma_x$|Standard deviation of the element $x$|
|$\sigma^2_x$|Variance of the element $x$|
|$\rho_{xy}$|Covariances of the elements $x$ and $y$|
|$\sim$|Distributed as|
|$\mathcal{N}$|Normal distribution|

# Le Kalman filter

## Context

The Kalman algorithm is an estimator, and not a filter per se. It will estimate the most likely state in which a system is, from measurements and model results, pondered by uncertainities. Thus, as it will aim to discard "errors" in the inputs, it should converge toward the true state of the system. That's why when looked at it from the point of view of signal processing, as it should discard noise and other artifacts presents in the recording and estimation errors from the models used, it looks like it is filtering the input fed to it.

To be consistent with our title, we will limit the use of "synonyms" and to be consistent with why most layman people look into the Kalman algorithm, we will keep calling it the Kalman filter, and not the Kalman estimator. We will aim for a consistent and strict mathematical notation, especially for vector and matrix, something that seem to be somewhat overlook into the physics textbooks.

## Steps

The Kalman filter can be summarized into two steps: *prediction* and *correction*.

### Prediction

1.
$$
\begin{equation}
x_p = \mathbf{F(t)} \cdot \vec{x}(t) + \mathbf{B(t)} \cdot \vec{u}(t) + \vec{\omega}
\end{equation}
$$

2.
$$
\begin{equation}
\mathbf{P_p} = \mathbf{F(t)} \cdot \mathbf{P(t)} \cdot \mathbf{F(t)}^{T} + \mathbf{Q(t)}
\end{equation}
$$


### Correction/Update

3.
$$
\begin{equation}
\mathbf{K} = \mathbf{P_p} \cdot \mathbf{H}^T \cdot (\mathbf{H} \cdot \mathbf{P_p} \cdot \mathbf{H}^T + \mathbf{R})^{-1}
\end{equation}
$$

4.
$$
\begin{equation}
\vec{x_c} = \vec{x_p} + \mathbf{K} \cdot (\vec{z} - \mathbf{H} \cdot \vec{x_p})
\end{equation}
$$

5.
$$
\begin{equation}
\mathbf{P_c} = \mathbf{P_p} - \mathbf{K} \cdot \mathbf{H} \cdot \mathbf{P_p}
\end{equation}
$$

## Terms definitions

Some of the following variables are not measured or calculated from measurements and/or associated logic. Therefore, they have to be establish by the operator: you. This is where the difficulties in using and establishing a good, accurate and valuable Kalman filter lie. It requires theorethical knowledge about the system of interest, but also theoretical knowledge theoretical domains related to the field of the system of interest, as well as pratical, field knowledge of the system of interest, most of the time gained through field study.

The modelling choice will be dependent, to an extend, on the operator choice with the consequences they imply. Be sure to put the adequat level of care and research for your application.

The following domains can be of interest in the task of developping adequat values for the following variables:
* Point estimation theory
* Linear Modelling
* Non-linear modelling
* Time variant modelling

### $x$ : State vector

It is of the form:

$$
\begin{equation}
    \vec{x} = 
        \begin{pmatrix}
            x_0\\
            \\
            x_1\\
            \\
            \vdots\\
            \\
            x_n
        \end{pmatrix}
\end{equation}
$$

With $x_0, x_1, \dotsc, x_n$ being the $N$ values fed to the Kalman filter inputs.

* $\vec{x}(t)$: State vector at time $t$, essentially the input of the Kalman filter iteration

* $\vec{x_p}$: State vector as predicted using model $\mathbf{F}$ and the control vector $\vec{u}$

* $\vec{x_c}$: State vector predicted and corrected by the Kalman filter, the state vector $\vec{x}(t+1)$. It is the output of the Kalman filter iteration and will be used as the state vector input $\vec{x}$ of the next Kalamn filter iteration

The state vector represent the assumption we are making about the state of the model. This is not the raw measurements. The state, and therefore the state vector, depends on the pre-established model $\mathbf{F}$ that will estimate the new state of the system from $t$ to $t+1$ : from $\vec{x(t)}$ to $\vec{x(t+1)}$. It could be for example the velocity and position of your robot.

The state is an estimation of the system state. It is built through the estimation from the state from previous $t$, $\vec{x}(t)$, the estimation from the state transition model, $\mathbf{F}(t)$, mitigated by the control(s), $\vec{u}$ applied to the system, and the external noise/perturbations not accounted by the state transition model, $\vec{\omega}$. All those, being mitigated by their respective level of uncertainty/trust, $\mathbf{B}, \mathbf{P}, \mathbf{Q}, mathbf{R}$, and the measurements $\vec{z}$ made about the system.

### $\mathbf{P}$ : Covariance matrix of the state vector $\vec{x}$

It is simply the covariance matrix linked to the state vector $\vec{x}$.

It is computed as such:

```math
\begin{equation}
\mathbf{P} = 
\begin{pmatrix}
\sigma^{2}_{x_0} & \rho{x_0 x_1} & \cdots & \rho{x_0 x_n}\\
& & & \\
\rho{x_1 x_0} & \sigma^{2}_{x_1} &  & \vdots \\
& & & \\
\vdots & & \ddots & \\
& & & \\
\rho{x_n x_0} & \cdots & & \sigma^{2}_{x_n}\\
\end{pmatrix}
\end{equation}
```

With $\sigma$ the variance and therefore, $x_0, x_1, \dotsc, x_n$ the reference to the elements of the state vector $\vec{x}$. The covariance matrix $\mathbf{P}$ is calculated from the values of the state vector $\vec{x}$.

* $\mathbf{P}(t)$ : the covariance matrix associated to the state vector $\vec{x}(t)$

* $\mathbf{P_p}$ : the covariance matrix associated to the state vector $\vec{x_p}$

* $\mathbf{P_c}$ : the covariance matrix associated to the state vector $\vec{x_c}$

The covariance matrix $\mathbf{P}$ of the state vector $\vec{x}$ represent the uncertainty, or more simply the variability of the values of the state vector $\vec{x}$, for example some measurements could make up the state vector $\vec{x}$. Due to noises and measurements errors from the measurements tools, the measurements contained in $\vec{x}$ differs slightly from the actual value of the thing that is measured, the truth in the physic sense of the term. In this case, the covariance matrix $\mathbf{P}$ of the state vector $\vec{x}$, will represent the variability of those measured values compared to the truth.

For more imformation on covariance and covariance matrix:
* [Covariance](https://fr.wikipedia.org/wiki/Covariance)
* [Covariance matrix](https://en.wikipedia.org/wiki/Covariance_matrix)

### $\mathbf{K}$ : Kalman gain

It is computed through the equation:

$$
\begin{equation}
\mathbf{K} = \mathbf{P_p} \cdot \mathbf{H}^T \cdot (\mathbf{H} \cdot \mathbf{P_p} \cdot \mathbf{H}^T + \mathbf{R})^{-1}
\end{equation}
$$

It dictate the balance of how much measurement, from the measurement vector $\vec{z}$, and estimate, from the predicted state vector $\vec{x}$, are used to create the new state vector $\vec{x}(t+1)$, the estimated **and** corrected state vector.

A nice approach to understand that balance relationship between measurement ($\vec{z}$) and state ($\vec{x}$) ruled by the Kalman gain, [is to look at its limit behavior after rewriting the equation](https://dsp.stackexchange.com/questions/2347/how-to-understand-kalman-gain-intuitively). 

So going from the equation given above we can write the Kalman gain as:

$$
\begin{equation}
\mathbf{K} = \frac{\mathbf{P}_p \cdot \mathbf{H}^T}{\mathbf{H} \cdot \mathbf{P}_p \cdot \mathbf{H}^T + \mathbf{R}}
\end{equation}
$$

Which give us two important limits:

$$
\begin{equation}
\lim\limits_{\mathbf{P} \to 0} \frac{\mathbf{P}_p \cdot \mathbf{H}^T}{\mathbf{H} \cdot \mathbf{P}_p \cdot \mathbf{H}^T + \mathbf{R}} = 0
\end{equation}
$$

Basically meaning, if the state vector $\vec{x}$ don't present a lot of uncertainity: we have a lot of trust in it, (this being expressed by a low magnitude covariance matrix of the state vector $\mathbf{P}$) will yield a low value of the Kalman gain: the filter will bias toward the state vector $\vec{x}$ value. While on the opposite, if their is a lot of uncertainty, therefore low trust, in the state vector values ($\vec{x}$), then the value of the Kalman gain will be high, which will bias the Kalman filter toward the measurement vector $\vec{z}$ values.

$$
\begin{equation}
\lim\limits_{\mathbf{R} \to 0} \frac{\mathbf{P}_p \cdot \mathbf{H}^T}{\mathbf{H} \cdot \mathbf{P}_p \cdot \mathbf{H}^T + \mathbf{R}}= \mathbf{H}^{-1}
\end{equation}
$$

At the same time, if the state vector is know accurately $\mathbf{H} \cdot \mathbf{P}_p \cdot \mathbf{H}^T$ is small, and therefore, if the measurements are not certains, we have low trust in them, as expressed by the covariance matrix of the sensor noise $\mathbf{R}$, it will yield a lower value of $\mathbf{K}$, biasing the Kalman filter towards the state vector $\vec{x}$ values. It can be said that from those equations, it appears that the Kalman filter seems like it have a slight bias toward the state vector $\vec{x}$ values, but a more serious work on the Kalman gain limits (in the mathematical sense) would be needed to confirm or infirm this last statement.

### $\vec{z}$ : Measurement vector

It is the vector composed of the measurements from the sensor(s). It is o the form:

$$
\begin{equation}
\vec{z} = 
    \begin{pmatrix}
        z_0\\
        \\
        z_1\\
        \\
        \vdots\\
        \\
        z_n\\
    \end{pmatrix}
\end{equation}
$$

With $z_0, z_1, \dotsc, z_n$, the measurements from the sensor(s). 

It is linked to the measurement noise $\vec{v}$ and the associated covariance matrix of the sensor noise $\mathbf{R}$. More details about those two variables can be found in their respective section down below.

The measurement vector has the following relation to the state vector:

$$
\begin{equation}
\vec{z}(t) = \mathbf{H}(t) \cdot \vec{x}(t) + \vec{v}(t)
\end{equation}
$$

With $\vec{z}$ the measurement vector, $\mathbf{H}$ the measurement matrix, $\vec{x}$ the state vector and $\vec{v}$ the measurement noise, and $t$ the time.

The measurement matrix $\mathbf{H}$ will map the measurements $\vec{z}$ into the state space to the state vector $\vec{x}$, meaning that the measurement matrix $\mathbf{H}$ express the relationship between the state vector $\vec{x}$ and measurements $\vec{z}$. More details can be found about the measurement matrix $\mathbf{H}$ can be found in the corresponding section down below.

### $\mathbf{F}$ : State transition model

State that F is the transition to point in time t to t+1 

The state transition model $\mathbf{F}$ is the model that will transition the state vector $\vec{x}$ from its value at time $t$ to time $t+1$. It can be time variant, and therefore be specific for each $\Delta t$. 

This would be the case for example when you try to assess the attitude (the state of the system) of a moving system, that is: pitch, roll and yaw. Where you can use the gyroscope measurements, angular speeds, acquired for a specific $\Delta t$ to design the transition model $\mathbf{F}(t)$ that will allow you to transition the state of the system, from $\vec{x}(t)$ to $\vec{x}(t+1)$.

As any other applications specific terms, you will have to provide it yourself, through theoretical and/or empirical knowledge. 

### $\mathbf{Q}$ : Covariance matrix of the process noise

This is the covariance matrix associated to the state transition model $\mathbf{F}$. It represents the uncertainty, or simply the variability, of the model $\mathbf{F}$ and the associated prediction. The larger the values composing $\mathbf{Q}$, the more variability, uncertainty, is present in the state transition model $\mathbf{F}$. And therefore, the less trust is put in it, and his weight into the calculus of the filtered value will be lowered.

More broadly, it is considered link to the uncertainity in the **update** step. And therefore is link to the control ($\vec{u}$, $\mathbf{B}$) and the external noise $\vec{\omega}$.

One way to solve the problem in a brute force manner might be to establish this parameter through some sort of [gradient descent](https://en.wikipedia.org/wiki/Gradient_descent).

To avoid accounting for any uncertainty in the state transition model $\mathbf{F}$, it is always possible to resort to the use of a zero matrix, $\mathbf{0}$ of adequat size.

### $\mathbf{H}$ : Measurement matrix

The measurement matrix $\mathbf{H}$ is the mathematical representation of how your measurements corresponds to your state vector. The shape of your measurement matrix $\mathbf{H}$ will therefore depend on the shape of your measurement vector $\vec{z}$ and state vector $\vec{x}$.

It is made of real values, and represent the relation between, the impact of, the measurements and the state vector. A value of $1$ in the matrix means that the corresponding measurement variable correspond directly to state variable.

The relation it holds between the state vector $\vec{x}$ and the measurement vector $\vec{z}$ is expressed in the following equation:

$$
\begin{equation}
\vec{z}(t) = \mathbf{H}(t) \cdot \vec{x}(t) + \vec{v}(t)
\end{equation}
$$

With $\vec{z}$ the measurement vector, $\mathbf{H}$ the measurement matrix, $\vec{x}$ the state vector and $\vec{v}$ the measurement noise, and $t$ the time.

### $\vec{v}$ : Measurement noise

As the measurement noise $\vec{v}$ will, very most likely, be already integrated in your measurements in the measurement vector $\vec{z}$, you should only be concerned by the characteristics of the noise when "simply" applying the Kalman filter, and not the noise itself.

The measurement noise usually assumed to be a zero-mean Gaussian random variable. But will depend on the sensors you are using, as well as the variables measured and the system of interest in which they are evolving in. The measurement noise $\vec{v}$ is associated to the covariance matrix of the sensor noise, $\mathbf{R}$. While the covariance matrix of the sensor noise $\mathbf{R}$ represent the statistical characteristics of the noise, the measurement noise $\vec{v}$ represent the actual noise of the measurement. 

The measurement noise $\vec{v}$ is computed from the equation:

$$
\begin{equation}
\vec{z}(t) = \mathbf{H}(t) \cdot \vec{x}(t) + \vec{v}(t)
\end{equation}
$$

With $\vec{z}$ the measurement vector, $\mathbf{H}$ the measurement matrix, $\vec{x}$ the state vector and $\vec{v}$ the measurement noise, and $t$ the time.

And is represented by a normal distribution:

$$ 
\begin{equation}
\vec{v} \sim \mathcal{N}(0, \mathbf{R})
\end{equation}
$$

$\mathcal{N}$ the normal distribution, and its first parameter: the mean of the distribution, here  $0$, and its second parameter the variance of the distribution, here represented by $\mathbf{R}$. 

The equation above is only valid if your measurement noise is distributed normally (in the mathematical sense) and mean $0$, as assumed in the Kalman filter. You will have to adapt the representation of $\vec{v}$ to your application. 

Or more generally by the normal distribution:

$$ 
\begin{equation}
\vec{v} \sim \mathcal{N}(
    \begin{pmatrix}
        \bar{v_{z_0}}\\
        \\
        \bar{v_{z_1}}\\
        \\
        \vdots\\
        \\
        \bar{v_{z_n}}\\
    \end{pmatrix}
    , \mathbf{R})
\end{equation}
$$

With $\bar{v_{z_0}}$, $\bar{v_{z_1}}$, $\dotsc$, $\bar{v_{z_n}}$, the mean of the noise from the measurements, $\vec{v}$, from the measurement vector $\vec{z}$.

In the equation of the Kalman filter, the measurement noise would be of the form:

$$
\begin{equation}
\vec{v}(t) = 
    \begin{pmatrix}
        v_{z_0}(t)\\
        \\
        v_{z_1}(t)\\
        \\
        \vdots\\
        \\
        v_{z_n}(t)\\
    \end{pmatrix}
\end{equation}
$$

With $v_{z_0}(t)$, $v_{z_1}(t)$, $\dotsc$, $v_{z_n}(t)$, the noise from the measurements from the measurement vector $\vec{z}$.

If you have to estimate the measurement noise and its characteristics, you will usually need to refer to the sensor documentation, which should provide you with information regarding the noise range of the sensor. Empirical experimentation might be needed to estimate the sensor noise characteristics in your application.

### $\mathbf{R}$ : Covariance matrix of the sensor noise

The covariance matrix $\mathbf{R}$ would be of the form:

```math
\begin{equation}
\mathbf{R} = 
    \begin{pmatrix}
        \sigma^{2}_{v_0} & \rho_{v_0v_1} & \cdots & \rho_{v_0v_n}\\
        \\
        \rho_{v_1v_0} & \sigma^{2}_{v_1} &  & \vdots \\
        \\
        \vdots & & \ddots & \\
        \\
        \rho{v_nv_0} & \cdots & & \sigma^2_{v_n}\\
    \end{pmatrix}
\end{equation}
```

Which, if the measurements are uncorrelated between each others, is of the form:

```math
\begin{equation}
\mathbf{R} = 
    \begin{pmatrix}
        \sigma^{2}_{v_0} & 0 & \cdots & 0\\
        \\
        0 & \sigma^{2}_{v_1} &  & \vdots \\
        \\
        \vdots & & \ddots & \\
        \\
        0 & \cdots & & \sigma^2_{v_n}\\
    \end{pmatrix}
\end{equation}
```

The case could be made that the covariance matrix of the sensor noise $\mathbf{R}$, and to an extend the measurement noise vector $\vec{v}$, are could be put into the **General terms** as in the general case of the Kalman filter: normally distributed noise with a distribution mean of $0$, those variables are somewhat trivial (in the mathematical sense) to compute, especially as the measurement noise vector $\vec{v}$ is contained in the measurement vector $\vec{z}$ that we use. But, as much as we want this introduction to the application of the Kalman filter to be simple and quick, we also want it to be as thorough, to allow the dedicated reader to understand what and why each variables.

### $\vec{u}$ : Control vector

This represent the control applied to the system of interest, susceptible to change the state vector $\vec{x}$ evolution dynamics. More simply, this is the influence voluntarily applied to the system, $\vec{u}$, to induce changes to the state vector $\vec{x}$. 

For example, suppose our system is a free falling rocket. The state vector $\vec{x}$ representing the position, the controle vector $\vec{u}$ would be the acceleration applied by the rocket thrust engines, and applying changes to the system outside of its premiere definition : a free falling object, a system with constant parameters.

### $\mathbf{B}$ : Control matrix

The control matrix $\mathbf{B}$ maps the control vector $\vec{u}$ to the state space of the state vector $\vec{x}$ and define, scale, the magnitude of the impact of the control vector $\vec{u}$ inputs, values.

It works a bit in the same way as the measurement matrix $\mathbf{H}$ in that it maps the control vector $\vec{u}$ values to the state space of the state vector $\vec{x}$ as well as the magnitude of the influence of the control vector $\vec{u}$.

It is sometimes called $\mathbf{G}$ in ressources on Kalman filter.

for example, let say we are interested in the velocity and the position of a system. It will be expressed by the following state vector:

$$
\begin{equation}
\vec{x} = 
    \begin{pmatrix}
        pos(t)\\
        \\
        vel(t)\\
    \end{pmatrix}
\end{equation}
$$

We know that the discrete velocity and position are computed using the following equation: 

```math
\begin{equation}
vel(t+1) \approx vel(t) + acc(t) * \Delta t
\end{equation}
```

```math
\begin{equation}
x(t+1) \approx x(t) + vel(t) * \Delta t + \frac{acc(t)}{2} * \Delta t^2
\end{equation}
```

The predicted transition from $\vec{x(t)}$ to $\vec{x(t+1)}$ will be made through the equation :

```math
\begin{equation}
x_p = \mathbf{F(t)} \cdot \vec{x}(t) + \mathbf{B(t)} \cdot \vec{u}(t) + \vec{\omega}
\end{equation}
```

With

* $\mathbf{F}$, the state transition model is the the expression of the physic formula described above, and therefore add the distance travelled during $\Delta t$ at the velocity $vel(t)$ from the state vector $\vec{x}(t)$, and keep the velocity the same. It is equat to

$$
\begin{pmatrix}
    1 & \Delta t\\
    \\
    0 & 1\\
\end{pmatrix}
$$ 

* For sake of simplicity, we will assume the oexternak sources of uncertainity $\vec{\omega}$ to be null

* For the sake of the argument, the control vector $\vec{u}$ will be of the form of a scalar (a non composite value, a singular value): $\vec{u}(t) = a(t)$, $a(t)$ an arbitrary value of acceleration. The exact value doesn't matter in this case.

In this case the control matrix would be, according to the physic formula stated above:

$$
\begin{equation}
\mathbf{B} = 
    \begin{pmatrix}
        \frac{1}{2} \cdot \Delta t^2\\
        \\
        \Delta t\\
    \end{pmatrix}
\end{equation}
$$

Again, as with any application specific variables, you will have to design your control matrix $\mathbf{B}$ in a way that is adequat to your system and needs.

### $\vec{\omega}$ : External noise, sources of uncertainity

It represent the sources of uncertainity that may influence the dynamic of the state vector from one $t$ to the other.

This parameter is sometimes included, or even entierely represented by the control vector $\vec{u}$, or sometimes absent from the Kalman filter equations.

# Initialisation of the Kalman filter

The initialisation of the Kalman filter is going to be an important step but also very application specific.

The variables consituing the Kalman filter will have to be found, or one could say intitialised, via theoretical and empirical means, the state vector $\vec{x}$ might even have to be chosen arbitrarily, and is somewhat apart from the rest of the variables for that.

Indeed, for example, let's say you have a robot needing to travel through a maze, just like a [micro mouse](https://en.wikipedia.org/wiki/Micromouse) robot. You choose to use a Kalman filter to estimate the position and velocity of the robot: your state vector $\vec{x}$. You could decide that the state vector $\vec{x}$ is initiated at the values of *position* $(0,0,0)$ and velocity *0*. The velocity make sense as your robot is, allegedly starting from a stopped position, but the position initiale state of $(0,0,0)$ is arbitrary: it just fit your needs, your application better.

The initial value of the state vector $\vec{x}$ really has this very obvious arbitrary aspect to it. But all the other *application specific* variables also have this arbritrary aspect: you build them, the models, the covariance matrices, etc... from theoretical and practical, empirical knowledge about your application and needs. This is where the line will be drawn between a good and a bad filter design: knowledge about the system and its domain of application and your ability to apply that knowledge adequatly. This is where the difficulty of the Kalman filter really lies.

Learn your system, study it. And just try. Build a safe and adequat experimental setup to be able to try out hypothesis about your system, your application and their practical application. Fail there rather than when guiding that multi millions rocket. Taxpayer around your country will thank you.

# Diagram summary and meta-example

Below is a little diagram followed by the variables associated to each components.

![Kalman filter diagram](./img/kalman_diagrams.png)

## Prediction phase

### Equation

1. 
$$
\begin{equation}
x_p = \mathbf{F(t)} \cdot \vec{x}(t) + \mathbf{B(t)} \cdot \vec{u}(t) + \vec{\omega}
\end{equation}
$$


2.
$$
\begin{equation}
\mathbf{P_p} = \mathbf{F(t)} \cdot \mathbf{P(t)} \cdot \mathbf{F(t)}^{T} + \mathbf{Q(t)}
\end{equation}
$$

### Variables 

* **State vector**: $\vec{x}(t)$, $\mathbf{P}(t)$

* **Control**: $\vec{u}(t)$, $\mathbf{B}$

* **State transition model**: $\mathbf{F}(t)$, $\mathbf{Q}$

* **External source of of uncertainty**: $\vec{\omega}$

## Correction phase

### Equation

3.
$$
\begin{equation}
\mathbf{K} = \mathbf{P_p} \cdot \mathbf{H}^T \cdot (\mathbf{H} \cdot \mathbf{P_p} \cdot \mathbf{H}^T + \mathbf{R})^{-1}
\end{equation}
$$

4.
$$
\begin{equation}
\vec{x_c} = \vec{x_p} + \mathbf{K} \cdot (\vec{z} - \mathbf{H} \cdot \vec{x_p})
\end{equation}
$$

5.
$$
\begin{equation}
\mathbf{P_c} = \mathbf{P_p} - \mathbf{K} \cdot \mathbf{H} \cdot \mathbf{P_p}
\end{equation}
$$

### Variables

* **State vector predicted**: $\vec{x_p}$, $\mathbf{P_p}$

* **Measurements**: $\vec{z}$, $\mathbf{R}$, $\mathbf{H}$, $\vec{v}$

* **Kalman gain**: $\mathbf{K}$

## Example

The following is a metaphorical example which, hoepfully, should help the reader grasp the meaning of each parts of the Kalman filter.

Let say that your brain is the hardware, and the software the Kalman filter. Your goal is to estimate the state of the world. For that you have at your disposal:

* The official informations about the world
* Your model of the functionning world
* The actions you do to change the world
* Your observations about the world

Which gave you the following system:

!["Kalman filter, world example"](img/kalman_example.png)

To estimate the state of the world ($\vec{x}(t)$) in 10 years ($\Delta t$), you start from the current state of the world. Of course, as you are not omniscien, there is things you are unsure about that present state ($\mathbf{P}$).

You listen to the TV, read some articles and build your model about how the world seems to be transforming ($\mathbf{F}$), and because you are not an augur you have a main line with a few variations depending on what your model can't account for ($\mathbf{Q}$).

In addition, because you are one of the wealthiest man on Earth, you account for the actions you are taking to change the world ($\vec{u}$) and apply that knowledge into your prediction ($\mathbf{B}$).

But sadly, you can't be 100% sure of the success of your endeavours or their consequences, and as we said earlier, you are rich but not omniscient: there is a lot you don't know, misunderstood or even been lied about ($\vec{\omega}$).

Once your prediction made about the futur is done, you go enquire about a how the world is doing ($\vec{z}$) directly, indirectly in some ways you can relate to your present predictions ($\mathbf{H}$). But once again, your informations are not always trusty, complete or intelligible ($\mathbf{R}$ and $\vec{v}$).

Once you got your predictions and you measurements made, you balance each others regarding the quality of the sources of information that allowed you to make those up, and give more importance to the trustier one ($\mathbf{K}$).

You end up with the best estimate you can come up to, of what will be the world in 10 years: that was one iteration of your mental Kalman estimator, also called Kalman filter. 

# How to cite

Robinault Lucien (2023). Demystifying the Kalman filter. V1.0.0 (https://github.com/TSS-22/demystifying-kalman-filter), GitHub. DOI: 10.5281/zenodo.10396368 

# Sources and recommended reads

The rumors say that **Anna** got them in her **archive**.

* BARRETO, Armando, ADJOUADI, Malek, ORTEGA, Francisco, et al. [Intuitive Understanding of Kalman Filtering with MATLAB®](https://www.taylorfrancis.com/books/mono/10.1201/9780429200656/intuitive-understanding-kalman-filtering-matlab%C2%AE-armando-barreto-malek-adjouadi-francisco-ortega-nonnarit-larnnithipong). ISBN-13: 978-0367191337. CRC Press, 2020.<br>A good book to grasp the foundation of the Kalman filter, as well as the filter in its basic mathematic form. The associated code examples are very valuable as they present the algorithm in a working manner (code is exempt of error). The drawback are that the book is relatively confuse and sometimes confusing, as denomination can seem inconsistent and information is scattered and disorganized.

* HASLWANTER, Thomas. [3D Kinematics](https://link.springer.com/book/10.1007/978-3-319-75277-8). ISBN-13: 978-3319752761. Cham, Switzerland : Springer, 2018.<br>Good ressource for 3D kinematics, as the name implies, but can be frustrating as it overlook some "trivial" steps sometimes. Very useful if your application of the Kalman filter will reside in the use of Inertial Motor Unit in the case of sensor fusion, or anything 3D related. It is short (good thing) and does its job. Sadly this is one of the most exhaustive source of information about the topic of 3D kinematics and it will ask for a lot of investment and personal research to dig valuable, valid, serious and proper information on the topic.

* STROUD, Kenneth Arthur et BOOTH, Dexter J. [Engineering mathematics](https://books.google.fr/books?hl=fr&lr=&id=ihlHEAAAQBAJ&oi=fnd&pg=PR4&dq=Engineering+Mathematics&ots=gtYs78qCCp&sig=ZEYlavfulfbGAY6DikNuDGrBQBs&redir_esc=y#v=onepage&q=Engineering%20Mathematics&f=false). ISBN-13: 978-1352010275. Bloomsbury Publishing, 2020.<br>Extremely valuable ressource to learn the basic mathematics to enter confidently the realm of science and engineering level mathematics. This book could teach mathematics to a monkey.

* STROUD, Kenneth Arthur et BOOTH, Dexter J. [Advanced engineering mathematics](https://www.tandfonline.com/doi/full/10.1080/00401706.2021.1982287). ISBN-13: 978-1352010251. Bloomsbury Publishing, 2020.<br>For the nerdy monkey.

* [How to understand Kalman gain intuitively?](https://dsp.stackexchange.com/questions/2347/how-to-understand-kalman-gain-intuitively). Answers: Jav_Rock, Zichao Zhang, ssk08. June 2012.<br> A nice view at the Kalman gain behavior to understand its mathematical purpose.

# Legal disclaimer

The information and tools contained in this repository are provided in good faith and no warranty, representation, statement or undertaking is given regarding any information or tool connected with this repository and any warranty, representation, statement or undertaking whatsoever that may be expressed or implied by statute, custom or otherwise is hereby expressly excluded. The use of the tools in this repository and any information in this repository is entirely at the risk of the user. Under no other circumstances the author should be liable for any costs, losses, expenses or damages (whether direct or indirect, consequential, special, economic or financial including any loss of profits) whatsoever that may be incurred through the use of any information or tools contained in this repository. This repository may contain inaccurate information. Which the author is under no responsibility to update or correct any such information or to even maintain this repository. Which the author reserves its right to change any information or any part of this repository without notice.
