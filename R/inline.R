inlineCxxPlugin <-
    Rcpp:::Rcpp.plugin.maker(
			include.before = "#include <Rclusterpp.h>", 
			package        = "Rclusterpp",
			LinkingTo      = c("Rclusterpp", "RcppEigen", "Rcpp"),
			Makevars       = NULL, 
			Makevars.win   = NULL
    )
