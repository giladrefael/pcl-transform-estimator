# pcl-transform-estimator

This code solves the problem of finding a rigid transform between two sets of points s.t. the
least-squares error between the destination points and the transformed points is minimized as described by [Umeyama](http://web.stanford.edu/class/cs273/refs/umeyama.pdf)


Scaling Support
When including a scaling factor, we now consider a new transformation matrix cR that includes the
scaling factor c as well as the rotation matrix R. The problem now becomes minimizing
∑ ‖(cRp i + t )−q i ‖
.
We can estimate c by computing
c=
1
tr(Σ M )
Var (P)
where Σ is the diagonal matrix from the SVD calculation and M and is either the identity matrix, for
non-reflective transformations, or diag(1,1,1,1...,-1) in case the algorithm happened to choose the
reflective transformation as the optimal one (determined by det (UV ) ).
We then need to adjust the translational part as follows:
t=q−cR p
Gaussian Noise
In the situation in which the destination points are corrupted by noise, the destination points become
q i =cRp i +t+ η
where η is random, gaussian noise.
In this case, the same optimization method will also minimize the least squares error for the noisy
data since it is also the maximum-likelihood estimation (for gaussian IID noise).
Outliers Handling
Our mean-based method is sensitive to outliers, therefore this solution should not provide good
results in the presence of outliers in the data. In this case it is advantageous to use robust methods
such as RANSAC or LMEDS (Least Median Squares) to acquire the rigid transform.
Sources:
http://web.stanford.edu/class/cs273/refs/umeyama.pdf
http://cs229.stanford.edu/notes/cs229-notes1.pdf