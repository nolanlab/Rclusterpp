Rclusterpp.hclust <- function(x, method="ward", members=NULL, distance="euclidean", p=2) {
	METHODS <- c("ward")
	method  <- pmatch(method, METHODS)
	if (is.na(method))
		stop("Invalid clustering method")
  if (method == -1) 
    stop("Ambiguous clustering method")

	if (class(x) == "dist") {
		return(hclust(x, METHODS[method], members))
	} else {
		DISTANCES <- c("euclidean")
		distance  <- pmatch(distance, DISTANCES)
		if (is.na(distance))
			stop("Invalid distance metric")
		if (method == -1)
			stop("Ambiguous distance metric")

		d <- dist(x, method=DISTANCES[distance], p=p)
	
		N <- nrow(x <- as.matrix(x))
		hcl <- .Call("hclust_from_data", 
		             data = as.double(x), nr = as.integer(N), nc = as.integer(ncol(x)),
								 link = as.integer(method), 
								 dist = as.integer(distance),
								 DUP = FALSE, NAOK = TRUE, PACKAGE = "Rclusterpp" )

		return(hclust(d, method=METHODS[method], members))
	}
}

