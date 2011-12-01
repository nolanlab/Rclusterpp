\name{Rclusterpp.setThreads}
\alias{Rclusterpp.setThreads}
\title{
Set number of threads used in clustering
}
\description{
Sets the number of threads used by the OpenMP based parallelism in hierarchical clustering.
}
\usage{
Rclusterpp.setThreads(threads = 1)
}
\arguments{
  \item{threads}{
	Desired number of threads. NULL will set number of threads to number of processors. 
}
}
\details{
	Simple wrapper for \code{omp_set_num_threads} when OpenMP is available,
	otherwise behaves as a NOOP. When \code{threads==NULL}, the number of
	processors will be determined via \code{omp_get_num_procs} and number of
	threads set to that value. Note on modern Intel processors with
	hyperthreading, this number is typically the number of hyperthread cores, and
	thus typically two times the number of physcal cores. Setting the threading
	that high is not always advantageous. Note that number of threads can also be
	set via the \code{OMP_NUM_THREADS} environment variable.
}
\value{
	Integer number of threads
}
\references{
\url{http://openmp.org/}
}
\author{
Michael Linderman
}
\examples{
# Set to one thread
Rclusterpp.setThreads(1)

# Set to number of threads equal to number of processors
Rclusterpp.setThreads()
}
