This document aims to summarize the developments and the theoretical
achievements covered throughout the implementation of a practical ICP
algorithm in order to be used with 3D Matching Systems.

Pose Registration via ICP
=========================

The matching process terminates with the attainment of the pose.
However, due to the multiple matching points, erroneous hypothesis, pose
averaging and etc. such pose is very open to noise and many times is far
from being perfect. Although the visual results obtained in that stage
are pleasing, the quantitative evaluation shows :math:`~10` degrees
variation, which is an acceptable threshold. Many times, the requirement
might be set well beyond this margin and it is desired to refine the
computed pose.

Furthermore, in typical RGBD scenes, the depth maps can capture only
less than half of the model due to the visibility in the scene.
Therefore, a robust pose refinement algorithm, which can register
occluded and partially visible shapes quickly and correctly is not an
unrealistic wish.

At this point, a trivial option would be to use the well known iterative
closest point algorithm . However, utilization of the basic ICP leads to
slow convergence, bad registration, outlier sensitivity and failure to
register partial shapes. Thus, it is definitely not suited to the
problem. For this reason, many variants have been proposed . Different
variants contribute to different stages of the pose estimation process.

ICP is composed of :math:`6` stages and the improvements I propose for
each stage is summarized below.

Sampling
--------

To improve convergence speed and computation time, it is common to use
less points than the model actually has. However, sampling the correct
points to register is an issue in itself. The naive way would be to
sample uniformly and hope to get a reasonable subset. More smarter ways
try to identify the critical points, which are found to highly
contribute to the registration process. Gelfand et. al. exploit the
covariance matrix in order to constrain the eigenspace, so that a set of
points which affect both translation and rotation are used. This is a
clever way of subsampling, which I will optionally be using in the
implementation.

A result of the smart covariance based sampling is shown in Figure
[samplecov].

Correspondence Search
---------------------

As the name implies, this step is actually the assignment of the points
in the data and the model in a closest point fashion. Correct
assignments will lead to a correct pose, where wrong assignments
strongly degrade the result. In general, KD-trees are used in the search
of nearest neighbors, to increase the speed. However this is not an
optimality guarantee and many times causes wrong points to be matched.
Luckily the assignments are corrected over iterations.

To overcome some of the limitations, Picky ICP and BC-ICP (ICP using
bi-unique correspondences) are two well-known methods. Picky ICP first
finds the correspondences in the old-fashioned way and then among the
resulting corresponding pairs, if more than one scene point :math:`p_i`
is assigned to the same model point :math:`m_j`, it selects :math:`p_i`
that corresponds to the minimum distance. BC-ICP on the other hand,
allows multiple correspondences first and then resolves the assignments
by establishing bi-unique correspondences. It also defines a novel
no-correspondence outlier, which intrinsically eases the process of
identifying outliers.

For reference, both methods are used. Because P-ICP is a bit faster,
with not-so-significant performance drawback, it will be the method of
choice in refinment of correspondences.

Weighting of Pairs
------------------

In my implementation, I currently do not use a weighting scheme. But the
common approaches involve *normal compatibility*
(:math:`w_i=n^1_i\cdot n^2_j`) or assigning lower weights to point pairs
with greater distances
(:math:`w=1-\frac{||dist(m_i,s_i)||_2}{dist_{max}}`) .

Rejection of Pairs
------------------

The rejections are done using a dynamic thresholding based on a robust
estimate of the standard deviation. In other words, in each iteration, I
find the MAD estimate of the Std. Dev. I denote this as :math:`mad_i`. I
reject the pairs with distances :math:`d_i>\tau mad_i`. Here
:math:`\tau` is the threshold of rejection and by default set to
:math:`3`. The weighting is applied prior to Picky refinement, explained
in the previous stage.

Error Metric
------------

As described in , a linearization of point to plane error metric is
used. This both speeds up the registration process and improves
convergence.

Minimization
------------

Even though many non-linear optimizers (such as Levenberg Mardquardt)
are proposed, due to the linearization in the previous step, pose
estimation reduces to solving a linear system of equations. This is what
I do exactly.

Efficient ICP Through Point Cloud Pyramids
------------------------------------------

The search is done through multiple levels, in a hierarchical fashion.
The registration starts with a very coarse set of samples of the model.
Iteratively, the points are densified and sought. After each iteration
the previously estimated pose is used as an initial pose and refined
with the ICP.

Visual Results
--------------

Results on Synthetic Data
~~~~~~~~~~~~~~~~~~~~~~~~~

Figure [icpsynth] shows the result of the registration when the scene is
partially observed and subject to random uniform noise. More over the
initial pose set out to be
:math:`[\theta_x, \theta_y, \theta_z, t_x, t_y, t_z]=[]`
