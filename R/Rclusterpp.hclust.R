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

		if (METHODS[method] == "ward" && DISTANCES[distance] != "euclidean") {
			warning("Distance method is forced to (squared) 'euclidean' distance for Ward's method")
			distance <- which(DISTANCES == "euclidean")[1]
		}
	
		N <- nrow(x <- as.matrix(x))
		hcl <- .Call("hclust_from_data", 
		             data = x,
								 link = as.integer(method), 
								 dist = as.integer(distance),
								 DUP = FALSE, NAOK = TRUE, PACKAGE = "Rclusterpp" )
		
		hcl$labels = row.names(x)
		hcl$method = METHODS[method]
		hcl$call   = match.call()
		hcl$dist.method = DISTANCES[distance]
		class(hcl) <- "hclust"
		
		return(hcl)
	}
}

