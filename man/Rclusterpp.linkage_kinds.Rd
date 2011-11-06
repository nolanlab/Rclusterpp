\name{Rclusterpp.linkage_kinds}
\alias{Rclusterpp.linkage_kinds}
\alias{Rclusterpp.distance_kinds}
\title{
	Linkage and distance kinds available in Rclusterpp
}
\description{
	Functions that return character vectors with the available linkage and distance kinds
	that correspond with internal enums. Intended for use with \samp{pmatch}. 
}
\usage{
Rclusterpp.linkage_kinds()
Rclusterpp.distance_kinds()
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
	Rclusterpp.linkage_kinds()
	Rclusterpp.distance_kinds()
}
