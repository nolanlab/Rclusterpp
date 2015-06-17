RclusterppLdFlags <- function(static=FALSE) { "" }

LdFlags <- function(static=FALSE) {
    cat(RclusterppLdFlags(static=static))
}
