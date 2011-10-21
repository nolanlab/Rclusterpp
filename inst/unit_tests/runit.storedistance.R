compare.hclust <- function(h1, h2) {
	checkTrue(all(h1$merge == h2$merge | h1$merge == h2$merge[,c(2,1)]), msg="Agglomerations don't match")
	checkEquals(h1$height, h2$height, msg="Agglomeration heights are not equal")
	checkEquals(h1$labels, h2$labels, msg="Cluster labels do not match")
}

test.storedistance.average.euclidean <- function() {
	h <- hclust(dist(USArrests, method="euclidean"), method="average")
	r <- Rclusterpp.hclust(dist(USArrests, method="euclidean"), method="average")
	compare.hclust(h, r)
}

test.storedistance.average.manhattan <- function() {
	h <- hclust(dist(USArrests, method="manhattan"), method="average")
	r <- Rclusterpp.hclust(dist(USArrests, method="manhattan"), method="average")
	compare.hclust(h, r)
}

test.storedistance.single.euclidean <- function() {
	h <- hclust(dist(USArrests, method="euclidean"), method="single")
	r <- Rclusterpp.hclust(dist(USArrests, method="euclidean"), method="single")
	compare.hclust(h, r)
}

test.storedistance.complete.euclidean <- function() {
	h <- hclust(dist(USArrests, method="euclidean"), method="complete")
	r <- Rclusterpp.hclust(dist(USArrests, method="euclidean"), method="complete")
	compare.hclust(h, r)
}

