\name{Rclusterpp.linkageKinds}
\alias{Rclusterpp.linkageKinds}
\alias{Rclusterpp.distanceKinds}
\title{
	Linkage and distance kinds available in Rclusterpp
}
\description{
	Functions that return character vectors with the available linkage and distance kinds
	that correspond with internal enums. Intended for use with \samp{pmatch}. 
}
\usage{
Rclusterpp.linkageKinds()
Rclusterpp.distanceKinds()
}
\details{
	Returns character vectors whose ordering enables integer to enum conversion
	for \samp{LinkageKinds} and \samp{DistanceKinds} enums within the Rclusterpp library.
}
\value{
Character vectors.

Linkage Kinds: "ward", "average", "single", "complete"

Distance Kinds: "euclidean", "manhattan", "maximum", "minkowski"
}
\author{
Michael Linderman
}
\seealso{
\link[Rclusterpp]{Rclusterpp.hclust}
}
\examples{
	Rclusterpp.linkageKinds()
	Rclusterpp.distanceKinds()
}
