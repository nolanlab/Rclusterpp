\name{Rclusterpp-package}
\alias{Rclusterpp-package}
\alias{Rclusterpp}
\docType{package}
\title{
Linkable C++ clustering
}
\description{
Rclusterpp provides flexible native clustering routines optimized for
performance and minimal memory requirements. Like Rcpp, Rclusterpp is designed
to be linked against by downstream packages.
}
\details{
Rclusterpp provides replacements for hierarchical clustering functions, e.g.
hclust, as a linkable library for building native implementations of custom
clustering algorithms.  
}
\author{
Michael D. Linderman, Robert Bruggner

Maintainer: Michael D. Linderman <michael.d.linderman@gmail.com>
}
\references{
Murtagh, F. (1983), "A survey of recent advances in hierarchical clustering algorithms", Computer Journal, 26, 354-359.
Sibson, R. (1973), "SLINK: An optimally efficient algorithm for the single-link cluster method", Computer Journal, 16, 30-34.
}
\keyword{ package }
\seealso{
\code{\link[Rcpp:Rcpp-package]{Rcpp}}
\code{\link[RcppEigen:RcppEigen-package]{RcppEigen}}
}
