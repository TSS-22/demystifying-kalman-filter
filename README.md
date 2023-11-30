clean the main file from the example

clean the name of the data file
Add header to the data file
What is stillness ? Should I discard ti

put in disclaimer that matrix can be scalar under certain circumstances, define which and when.

put in disclaimer the different names of the variables so people can read more easily other texts.

explain the distribution link aspect of the variables (That Kalman filter work on Bayesian estimation)

Add diagram of how the Kalman filter work, and put the variables where they are used, or simply when they appear on the diagrams.

look for ADD EXAMPLE

Add table of content

make it clear what the state is  (the estimated state of the model. it is a mix between the estimation and the results from kalman filter: the filter provide a state estimation that is pondered by its level of trust in its underlied model F, and its measurement and associated trust, as well as control and associated trust if any controls are present)

# The myth, the Kalman, the legend: The Kalman filter

An attempt at demystifying the Kalman filter, in order to stop making it look more complex than it is.

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

In this work we will refer to the discrete version of the Klaman filter algorithm.

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

$$
\begin{equation}
x_p = \mathbf{F(t)} \cdot \vec{x}(t) + \mathbf{B(t)} \cdot \vec{u}(t) + \vec{\omega}
\end{equation}
$$

$$
\begin{equation}
\mathbf{P_p} = \mathbf{F(t)} \cdot \mathbf{P(t)} \cdot \mathbf{F(t)}^{T} + \mathbf{Q(t)}
\end{equation}
$$


### Correction/Update

$$
\begin{equation}
\mathbf{K} = \mathbf{P_p} \cdot \mathbf{H}^T \cdot (\mathbf{H} \cdot \mathbf{P_p} \cdot \mathbf{H}^T + \mathbf{R})^{-1}
\end{equation}
$$

$$
\begin{equation}
\vec{x_c} = \vec{x_p} + \mathbf{K} \cdot (\vec{z} - \mathbf{H} \cdot \vec{x_p})
\end{equation}
$$

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

The state vector represent the assumption we are making about the state of the model. This is not the raw measurements. The state, and therefore the state vector, depends on the pre-established model $\mathbf{F}$ that will estimate the new state of the system from $t$ to $t+1$ : from $\vec{x(t)}$ to $\vec{x(t+1)}$. For example, *ADD EXAMPLE*

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

It works a bit in the same way as the measurement matrix $\mathbf{H}$ in that it maps the control vector $\vec{u}$ values to the state space of the state vector $\vec{x}$ as well as the magnitude of the influence of the control vector $\vec{u}$.

### $\mathbf{B}$ : Control matrix

The control matrix $\mathbf{B}$ maps the control vector $\vec{u}$ to the state space of the state vector $\vec{x}$ and define, scale, the magnitude of the impact of the control vector $\vec{u}$ inputs, values.

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

$$
\begin{equation}
vel(t+1) \approx vel(t) + acc(t) * \Delta t
\end{equation}
$$
$$
\begin{equation}
x(t+1) \approx x(t) + vel(t) * \Delta t + \frac{acc(t)}{2} * \Delta t^2
\end{equation}
$$
The predicted transition from $\vec{x(t)}$ to $\vec{x(t+1)}$ will be made through the equation :

$$
\begin{equation}
x_p = \mathbf{F(t)} \cdot \vec{x}(t) + \mathbf{B(t)} \cdot \vec{u}(t) + \vec{\omega}
\end{equation}
$$

With

* $\mathbf{F}$, the state transition model, equal to $\begin{pmatrix}
1 & \Delta t\\
\\
0 & 1\\
\end{pmatrix}$, as it add the distance travelled during $\Delta t$ at the velocity $vel(t)$ from the state vector $\vec{x}(t)$, and keep the velocity the same. This is the simply the expression of the physic formula described above.

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

### $\vec{\omega}$ : External sources of uncertainity

It represent the sources of uncertainity that may influence the dynamic of the state vector from one $t$ to the other.

This parameter is sometimes included, or even entierely represented by the control vector $\vec{u}$, or sometimes absent from the Kalman filter equations.

# Initialisation of the Kalman filter

go through a little rundown text and then variable by variable?

# Example

example with the governement

# Sources and recommended reads

The rumors say that **Anna** got them in her **archive**.

* BARRETO, Armando, ADJOUADI, Malek, ORTEGA, Francisco, et al. [Intuitive Understanding of Kalman Filtering with MATLAB®](https://www.taylorfrancis.com/books/mono/10.1201/9780429200656/intuitive-understanding-kalman-filtering-matlab%C2%AE-armando-barreto-malek-adjouadi-francisco-ortega-nonnarit-larnnithipong). ISBN-13: 978-0367191337. CRC Press, 2020.<br>A good book to grasp the foundation of the Kalman filter, as well as the filter in its basic mathematic form. The associated code examples are very valuable as they present the algorithm in a working manner (code is exempt of error). The drawback are that the book is relatively confuse and sometimes confusing, as denomination can seem inconsistent and information is scattered and disorganized.

* HASLWANTER, Thomas. [3D Kinematics](https://link.springer.com/book/10.1007/978-3-319-75277-8). ISBN-13: 978-3319752761. Cham, Switzerland : Springer, 2018.<br>Good ressource for 3D kinematics, as the name implies, but can be frustrating as it overlook some "trivial" steps sometimes. Very useful if your application of the Kalman filter will reside in the use of Inertial Motor Unit in the case of sensor fusion, or anything 3D related. It is short (good thing) and does its job. Sadly this is one of the most exhaustive source of information about the topic of 3D kinematics and it will ask for a lot of investment and personal research to dig valuable, valid, serious and proper information on the topic.

* STROUD, Kenneth Arthur et BOOTH, Dexter J. [Engineering mathematics](https://books.google.fr/books?hl=fr&lr=&id=ihlHEAAAQBAJ&oi=fnd&pg=PR4&dq=Engineering+Mathematics&ots=gtYs78qCCp&sig=ZEYlavfulfbGAY6DikNuDGrBQBs&redir_esc=y#v=onepage&q=Engineering%20Mathematics&f=false). ISBN-13: 978-1352010275. Bloomsbury Publishing, 2020.<br>Extremely valuable ressource to learn the basic mathematics to enter confidently the realm of science and engineering level mathematics. This book could teach mathematics to a monkey.

* STROUD, Kenneth Arthur et BOOTH, Dexter J. [Advanced engineering mathematics](https://www.tandfonline.com/doi/full/10.1080/00401706.2021.1982287). ISBN-13: 978-1352010251. Bloomsbury Publishing, 2020.<br>For the nerdy monkey.
