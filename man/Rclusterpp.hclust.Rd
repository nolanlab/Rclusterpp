\name{Rclusterpp.hclust}
\alias{Rclusterpp.hclust}
\title{
Hierarchical Clustering
}
\description{
Hierarchical clustering on both disimilarities and data
}
\usage{
Rclusterpp.hclust(x, method = "ward", members = NULL, distance = "euclidean", p = 2)
}
\arguments{
  \item{x}{
A numeric data matrix, data frame or a dissimilarity structure as produced by \code{dist}.
}
  \item{method}{
The agglomeration method to be used. This must be one of "ward", "single", "complete" or "average".
}
  \item{members}{
\code{NULL} or a vector with length size of \code{x}. See \code{\link{hclust}}.
}
  \item{distance}{
The distance measure to be used. This must be one of "euclidiean", "manhattan", "maximum", or "minkowski".
}
  \item{p}{
The power of the Minkowski distance.
}
}
\details{
If \code{x} is a disimilarity matrix, execution defaults to standard hclust. If
\code{x} is a set of observations, specialized native clustering routines are
invoked. These routines are optimized for O(n) memory footprint and multicore
execution to permit clustering of large datasets.  
}
\value{
An object of class *hclust* which describes the tree produced by the clustering process. See \code{\link{hclust}}.
}
\references{
Murtagh, F. (1983), "A survey of recent advances in hierarchical clustering algorithms", Computer Journal, 26, 354-359.
Sibson, R. (1973), "SLINK: An optimally efficient algorithm for the single-link cluster method", Computer Journal, 16, 30-34.
}
\author{
Michael Linderman
}
\note{
Support for different agglomeration methods and distance metrics is evolving.
}
\seealso{
\code{\link{hclust}}
}
\examples{
h <- Rclusterpp.hclust(USArrests, method="ward", distance="euclidean")
}
