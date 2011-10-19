compare.hclust <- function(h1, h2) {
	checkTrue(all(h1$merge == h2$merge | h1$merge == h2$merge[,c(2,1)]), msg="Agglomerations don't match")
	checkEquals(h1$height, h2$height, msg="Agglomeration heights are not equal")
	checkEquals(h1$labels, h2$labels, msg="Cluster labels do not match")
}

hclust.distance <- function(x, method="average") {
	if (class(x) != "dist") {
		stop("Method only supported for distance matrices")
	}
	
	METHODS <- c("ward", "average", "single", "complete")
	method  <- pmatch(method, METHODS)
	if (is.na(method))
		stop("Invalid clustering method")
  if (method == -1) 
    stop("Ambiguous clustering method")

	dist.method = attributes(x)$method
	labels      = attributes(x)$Labels

	N <- nrow(x <- as.matrix(x))
	hcl <- .Call("hclust_from_distance", 
							 data = x,
							 link = as.integer(method), 
							 DUP = FALSE, NAOK = FALSE, PACKAGE = "Rclusterpp" )
	
	hcl$labels      = labels 
	hcl$method      = METHODS[method]
	hcl$call        = match.call()
	hcl$dist.method = dist.method 
	class(hcl) <- "hclust"
	
	return(hcl)
}


#test.storedistance.average.euclidean <- function() {
#	h <- hclust(dist(USArrests, method="euclidean"), method="average")
#	r <- hclust.distance(dist(USArrests, method="euclidean"), method="average")
#	compare.hclust(h, r)
#}
	
