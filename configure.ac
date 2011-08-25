AC_PREREQ(2.62)

AC_INIT([Rclusterpp], 0.0.1)

dnl Now find the compiler and compiler flags to use
: ${R_HOME=`R RHOME`}
if test -z "${R_HOME}"; then
  echo "could not determine R_HOME"
  exit 1
fi
CPP=`"${R_HOME}/bin/R" CMD config CPP`
CPPFLAGS=`"${R_HOME}/bin/R" CMD config CPPFLAGS`

AC_LANG_PUSH(C++)
AC_OPENMP
AC_SUBST(OPENMP_CXXFLAGS,${OPENMP_CXXFLAGS})
AC_LANG_POP(C++)

AC_CONFIG_FILES([src/Makevars])
AC_OUTPUT
