# Test Setup
 
if(FALSE) {
  # Not really needed, but can be handy when writing tests
  library("RUnit")
  library("Rclusterpp")
}

compare.hclust <- function(h1, h2) {
	checkTrue(all(h1$merge == h2$merge | h1$merge == h2$merge[,c(2,1)]), msg="Agglomerations don't match")
	checkEquals(h1$height, h2$height, msg="Agglomeration heights are not equal")
	checkEquals(h1$labels, h2$labels, msg="Cluster labels do not match")
}

# --- Test functions ---
 
test.hclust.ward <- function()
{
  h <- hclust((dist(USArrests, method="euclidean")^2)/2.0, method="ward")
	r <- Rclusterpp.hclust(USArrests, method="ward")
	compare.hclust(h, r)
}

test.hclust.average <- function()
{
	h <- hclust(dist(USArrests, method="euclidean"), method="average")
	r <- Rclusterpp.hclust(USArrests, method="average", distance="euclidean")
	compare.hclust(h, r)
}
