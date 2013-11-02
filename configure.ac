#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT(ser2sock, 1.4.4, general@support.nutech.com)
AC_CONFIG_SRCDIR([ser2sock.c])
AM_INIT_AUTOMAKE
AC_CONFIG_HEADERS([config.h])

# Checks for programs.
AC_PROG_CC

# Checks for libraries.
AC_ARG_WITH([ssl], [AS_HELP_STRING([--without-ssl], [disable support for OpenSSL])], [], [with_ssl=yes])

AS_IF([test "x$with_ssl" != xno],
		[AC_CHECK_LIB([ssl], [SSL_library_init], [], [AC_MSG_FAILURE([ssl test failed (--without-ssl to disable)])])])

AS_IF([test "x$with_ssl" != xno],
		[AC_CHECK_LIB([crypto], [BIO_pop], [], [AC_MSG_FAILURE([ssl test failed (--without-ssl to disable)])])])

AC_ARG_ENABLE([vswtc], [AS_HELP_STRING([--enable-vswtc], [enable VSWTC for termios])], [enable_vswtc=yes], [])
AS_IF([test "x$enable_vswtc" == xyes], [AC_DEFINE(VSWTC, [], [Define if you need VSWTC])])

AC_ARG_ENABLE([periodic-checking], [AS_HELP_STRING([--enable-periodic-checking], [enable periodic serial port checking])], [enable_periodic=yes], [])
AS_IF([test "x$enable_periodic" == xyes], [AC_DEFINE(USE_TIOCMGET, [], [Define if you need periodic serial port checking])])

# Checks for header files.
AC_CHECK_HEADERS([arpa/inet.h fcntl.h netinet/in.h stdint.h stdlib.h string.h sys/ioctl.h sys/socket.h sys/time.h syslog.h termios.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_TYPE_PID_T

# Checks for library functions.
AC_FUNC_ERROR_AT_LINE
AC_FUNC_FORK
AC_FUNC_MALLOC
AC_CHECK_FUNCS([bzero gettimeofday memset select socket strchr strdup strerror])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
