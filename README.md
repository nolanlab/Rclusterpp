# Rclusterpp -- Large-scale hierarchical clustering in R

Rclusterpp provides flexible native hierarchical clustering routings optimized
for performance and minimal memory requirements. In particular Rclusterpp
includes "stored data" clustering implementations with *O(n)* memory
footprints. Rclusterpp has been successfully used to cluster 100,000s of observations.

Rclusterpp makes extensive use of
[Rcpp](http://dirk.eddelbuettel.com/code/rcpp.html) for integration with R, and
the [Eigen](http://eigen.tuxfamily.org) matrix library (via
[RcppEigen](http://cran.r-project.org/web/packages/RcppEigen/index.html)).
Rclusterpp provides a R interface to its internal libraries that can be used in
place of `stats::hclust` and provides linkable libraries for use by downstream packages.

Explore the unit tests `inst/unit_tests` and examples directory `inst/examples`
for examples on how to use Rclusterpp directly within R, or as a linkable	library 
for use with other native code. Note that some of the examples
require the [inline](http://cran.r-project.org/web/packages/inline/index.html) package.

Rclusterpp uses OpenMP internally for concurrent execution. By default, as many
threads as processors are created. To control the number of threads set the
`OMP_NUM_THREADS` environment variable.

## Installation
Rclusterpp installation instructions can be found on the [project wiki](https://github.com/nolanlab/Rclusterpp/wiki/Getting-Started).
