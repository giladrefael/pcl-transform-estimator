# pcl-transform-estimator

This code solves the problem of finding a rigid transformation between two sets of points such that the least-squares error between the destination points and the transformed source points is minimized as described by [Umeyama](http://web.stanford.edu/class/cs273/refs/umeyama.pdf).

Generally, we want to minimize ∑ ‖(cRp_i + t )−q_i ‖ where `p` is a source point and `q` is a destination point.

Input: `src.txt` and `dst.txt` files including the source and destination point clouds.
Output: The rotation, translation and scale factor that minimize the least-squares error.

The same optimization method will also work in the situation in which the destination points are corrupted by gaussian noise since it is also the maximum-likelihood estimation for gaussian IID noise.
