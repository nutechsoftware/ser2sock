/******************************************************************************\
 *
 *  MODULE: ser2sock.c
 *          Copyright (C) 2013 Nu Tech Software Solutions, Inc.
 *          All rights reserved
 *          Reproduction without permission is prohibited
 *
 *  This file may be used under the terms of the GNU General Public
 *  License versions 3.0 as published by the Free Software Foundation
 *  and appearing in the file COPYING included in the packaging of this project.
 *
 *  This file is provided "AS IS" with NO WARRANTY OF ANY KIND,
 *  INCLUDING THE WARRANTIES OF DESIGN, MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE. Nu Tech reserves all rights not expressly
 *  granted herein.
 *
 *
 *  PURPOSE: Connect to a given device file such as a serial device and multiplex
 *           all messages from this file to every socket that is connected.
 *
 *  DEVELOPED BY: Sean Mathews
 *                http://www.nutech.com/
 *
 *     Thanks to Richard Perlman [ad2usb at perlman.com] for his help testing on
 *     bsd and excellent feedback on features. Also a big thanks to everyone
 *     that helped support the AD2USB project get off the ground.
 *
 \******************************************************************************/
#define _GNU_SOURCE

#include "config.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>
#include <signal.h>
#include <stdarg.h>
#include <time.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <ctype.h>
#ifdef _POSIX_SOURCE
#include <sched.h>
#endif

#ifdef HAVE_LIBSSL
#include <openssl/bio.h>
#include <openssl/ssl.h>
#include <openssl/err.h>
#endif

#define SER2SOCK_VERSION "V1.4.4"
#define TRUE 1
#define FALSE 0

#define MAXCLIENTCONNECTIONS 10
#define MAXCONNECTIONS MAXCLIENTCONNECTIONS+2
#define MAX_FIFO_BUFFERS 30

#define SERIAL_CONNECTED_MSG	"!SER2SOCK SERIAL_CONNECTED\r\n"
#define SERIAL_DISCONNECTED_MSG	"!SER2SOCK SERIAL_DISCONNECTED\r\n"

#define CONFIG_PATH "/etc/ser2sock/ser2sock.conf"

#define DAEMON_NAME "ser2sock"
#define PID_FILE "/var/run/ser2sock.pid"

/* <Types and Constants> */

typedef int BOOL;
const char terminal_init_string[] = "\377\375\042";
const char * fd_type_strings[] =
{ "NA", "LISTEN", "CLIENT", "SERIAL" };
enum FD_TYPES
{
	NA, LISTEN_SOCKET = 1, CLIENT_SOCKET, SERIAL
} fd_types;

#define STREAM_MAIN 0
#define STREAM_SERIAL 1

char * syslog_format_type_strings[] =
{ "", "[✔] ", "[‼] ", "[✘] " };
int syslog_format_type_priority[] =
{ LOG_INFO, LOG_INFO, LOG_WARNING, LOG_ERR };
char * stderr_format_type_strings[] =
{ "", "[\033[1;32m✔\033[0m] ", "[\033[1;33m‼\033[0m] ", "[\033[1;31m✘\033[0m] " };
enum MESSAGE_TYPES
{
	MSG_NONE = 0, MSG_GOOD, MSG_WARN, MSG_BAD
} msg_types;


typedef struct
{
	char *name;
	int flag;
} speed_spec;
speed_spec speeds[] =
{
{ "1200", B1200 },
{ "2400", B2400 },
{ "4800", B4800 },
{ "9600", B9600 },
{ "19200", B19200 },
{ "38400", B38400 },
{ "57600", B57600 },
{ "115200", B115200 },
{ NULL, 0 } };


/* </Types and Constants> */

/* <Structures> */

typedef struct
{
  int last;
  char message[2048];
} logstream;

typedef struct
{
	int size, in, out, avail;
	void **table;
} fifo;

typedef struct
{
	/* flags */
	int new;
	int inuse;
	int fd_type;

	/* the fd */
	int fd;
#ifdef HAVE_LIBSSL
	/* SSL descriptor */
	BIO* ssl;
	BOOL handshake_done;
#endif
	/* persistent settings */
	struct termios oldtio;

	/* the buffer */
	fifo send_buffer;
} FDs;


#if defined __FreeBSD__
typedef unsigned char byte_t;
#else
typedef char byte_t;
#endif

/* </Structures> */

/* <Prototypes> */
int init_system();
int init_listen_socket_fd();
int init_serial_fd(char *path);
void add_to_all_socket_fds(char * message);
void add_to_serial_fd(char * message);
int cleanup_fd(int n);
void set_non_blocking(int fd);
void print_serial_fd_status(int fd);
int get_baud(char *szbaud);
void listen_loop();
void log_message(int stream,int type, char *msg, ...);
void vlog_message(int stream,int type, char *msg, va_list arg);
void error(char *msg, ...);
int kbhit();
int add_fd(int fd, int fd_type);
int __nsleep(const struct timespec *req, struct timespec *rem);
int msleep(unsigned long milisec);
void show_help();
void signal_handler(int sig);
BOOL read_config(char* filename);

// fifo buffer stuff
void fifo_init(fifo *f, int size);
void fifo_destroy(fifo *f);
int fifo_empty(fifo *f);
int fifo_add(fifo *f, void *next);
void* fifo_get(fifo *f);
void fifo_clear(fifo *f);
static void writepid(void);

#ifdef HAVE_LIBSSL
BOOL init_ssl();
void shutdown_ssl();
void shutdown_ssl_conn(BIO* sslbio);
#ifdef SSL_DEBUGGING
long    tls_bio_dump_cb(BIO *bio, int cmd, const char *argp, int argi,
			        long unused_argl, long ret);
void apps_ssl_info_callback(const SSL *s, int where, int ret);
void ssl_msg_callback(int write_p, int version, int content_type,
		 const void *buf, size_t len, SSL * ssl, void *arg);
#endif
#endif
/* </Prototypes> */

/* <Globals> */

/* Our process ID and Session ID */
pid_t pid=0, sid=0;

volatile sig_atomic_t got_hup = 0;
char * serial_device_name = 0;
int listen_port = 10000;
int socket_timeout = 10;
int listen_backlog = 10;
FDs my_fds[MAXCONNECTIONS];
/* our listen socket */
int listen_sock_fd = -1;
struct sockaddr_in serv_addr;
struct sockaddr_in peer_addr;
// fifo buffer
fifo data_buffer;
char * option_bind_ip = 0;
char * option_baud_rate = 0;
BOOL option_daemonize = FALSE;
BOOL option_send_terminal_init = FALSE;
int option_debug_level = 0;
BOOL option_keep_connection = FALSE;
int option_open_serial_delay = 5000;
int line_ended = 0;
int serial_connected = 0;
struct timeval tv_serial_start, tv_last_serial_check;

#ifdef HAVE_LIBSSL
BOOL option_ssl = FALSE;
SSL_CTX* sslctx = 0;
BIO* bio = 0, *abio = 0;
char* option_ca_certificate = NULL;
char* option_ssl_certificate = NULL;
char* option_ssl_key = NULL;
#endif
/* </Globals> */

/* <Code> */

/*
 show our error message and die
 todo: add params.
 */
void error(char *msg, ...)
{
	char * szError = strerror(errno);

	va_list arg;
	va_start(arg, msg);
	vlog_message(STREAM_MAIN,MSG_BAD, msg, arg);
	va_end(arg);

	log_message(STREAM_MAIN,MSG_BAD, " :");
	log_message(STREAM_MAIN,MSG_BAD, szError);
	log_message(STREAM_MAIN,MSG_BAD, "\n");
	log_message(STREAM_MAIN,MSG_BAD, "exiting\n");
	exit(EXIT_FAILURE);
}

/*
 log a message to console or syslog
 */
void log_message(int stream, int type,char *msg, ...)
{
	va_list arg;

	if (msg)
	{
		va_start(arg, msg);
		vlog_message(stream,type, msg, arg);
		va_end(arg);
	}
}

void vlog_message(int s,int type, char *msg, va_list arg)
{
	/* 2 queue's so we can watch 2 log streams for \n's static so auto init to 0's */
	static logstream ls[2];

	static BOOL syslog_open=FALSE;

	int x,y,z=0;
	BOOL done=FALSE;

	if (option_daemonize && !syslog_open) {
		openlog(DAEMON_NAME, LOG_CONS | LOG_NDELAY | LOG_PERROR | LOG_PID,
				LOG_USER);
		syslog_open=TRUE;
	}

	ls[s].last += vsnprintf(&ls[s].message[ls[s].last], sizeof(ls[0].message) - ls[s].last, msg, arg);

	/* check for overflow error */
	if (ls[s].last >= sizeof(ls[0].message))
		ls[s].last = 0;

	if (ls[s].last) {
		/* keep trying till we exause all \n's */
		while(!done)
		{
			/* look for an eol char */
			for (x = 0; x < ls[s].last ; x++) {

				if(ls[s].message[x] == '\n' || ls[s].message[x] == '\r') {

					ls[s].message[x]=0;
					if(x)
					{
						if (option_daemonize)
						{
							if (type) {

								syslog(syslog_format_type_priority[type], "%s%s",
								       syslog_format_type_strings[type], ls[s].message);
							}
							else
								syslog(LOG_INFO, "%s", ls[s].message);
						}
						else
						{
							if (type)
								fprintf(stderr, "%s%s\n", stderr_format_type_strings[type], ls[s].message);
							else {
								fprintf(stderr, "%s\n", ls[s].message);
								fflush(stderr);
							}
						}
					}
					/* move the rest to the start and clean out any non printable chars */
					z = 0;
					for(y = x+1; y < ls[s].last ; y++) {
						if((ls[s].message[y]>0x1f && ls[s].message[y]<0x7f) || ls[s].message[y]=='\n') {
							ls[s].message[z++]=ls[s].message[y];
						}
					}

					/* set our next fill position */
					ls[s].last = z;

					/* again */
					break;
				}
			}
			/* ok we reached the end of our buffer and found no more \n's */
			done = TRUE;
		}
	}
}

/*
 nanosecond sleep
 */
int __nsleep(const struct timespec *req, struct timespec *rem)
{
	struct timespec temp_rem;
	if (nanosleep(req, rem) == -1)
		__nsleep(rem, &temp_rem);
	return TRUE;
}

/*
 sleep for N milliseconds
 */
int msleep(unsigned long milisec)
{
	struct timespec req =
	{ 0 }, rem =
	{ 0 };
	time_t sec = (int) (milisec / 1000);
	milisec = milisec - (sec * 1000);
	req.tv_sec = sec;
	req.tv_nsec = milisec * 1000000L;
	__nsleep(&req, &rem);
	return 1;
}

/*
 show help info
 */
void show_help(const char *appName)
{
	fprintf(
			stderr,
			"Usage: %s -p <socket listen port> -s <serial port dev>\n\n"
				"  -h, -help                 display this help and exit\n"
				"  -p port                   socket port to listen on\n"
				"  -s <serial device>        serial device; ex /dev/ttyUSB0\n"
				"options\n"
				"  -i IP                     bind to a specific ip address; default is ALL\n"
				"  -b baudrate               set baud rate; defaults to 9600\n"
				"  -d                        daemonize\n"
				"  -t                        send terminal init string\n"
				"  -g                        debug level 0-3\n"
				"  -c                        keep incoming connections when a serial device is disconnected\n"
				"  -w milliseconds           delay between attempts to open a serial device (5000)\n"
#ifdef HAVE_LIBSSL
				"  -e                        use SSL to encrypt the connection\n"
#endif
				"\n", appName);
}

/*
 Initialize any structures etc.
 */
int init_system()
{
	int x;
	for (x = 0; x < MAXCONNECTIONS; x++)
	{
		my_fds[x].inuse = FALSE;
		my_fds[x].new = TRUE;
		my_fds[x].fd = -1;
		my_fds[x].fd_type = NA;
#ifdef HAVE_LIBSSL
		my_fds[x].ssl = 0;
		my_fds[x].handshake_done = FALSE;
#endif
		fifo_init(&my_fds[x].send_buffer, MAX_FIFO_BUFFERS);
	}

	/* Setup signal handling if we are to daemonize */
	if (option_daemonize)
	{
		signal(SIGTERM, signal_handler);
		signal(SIGINT, signal_handler);
		signal(SIGQUIT, signal_handler);
		setlogmask(LOG_UPTO(LOG_DEBUG));
	}

	// HUP is always bound.
	signal(SIGHUP, signal_handler);

	return TRUE;
}

/*
 clear all memory used before we exit
 */
int free_system()
{
	int x;
	for (x = 0; x < MAXCONNECTIONS; x++)
	{
		cleanup_fd(x);
		fifo_destroy(&my_fds[x].send_buffer);
	}
#ifdef HAVE_LIBSSL
	if (option_ssl)
		shutdown_ssl();
#endif
	return TRUE;
}

/*
 Initialize our listening socket and related api's
 */
int init_listen_socket_fd()
{
	BOOL bOptionTrue = TRUE;
	int results;
	struct linger solinger;
#ifdef HAVE_LIBSSL
	if (option_ssl)
	{
		if (!init_ssl())
			return FALSE;
	}
	else
#endif
	{
		/* create a listening socket fd */
		listen_sock_fd = socket(AF_INET, SOCK_STREAM, 0);
		if (listen_sock_fd < 0)
		{
			log_message(STREAM_MAIN, MSG_BAD, "Fatal error creating our listening socket errno: %i\n",errno);
			return FALSE;
		}

		/* clear our socket address structure */
		bzero((char *) &serv_addr, sizeof(serv_addr));

		if (option_bind_ip != NULL)
		{
			results = inet_pton(AF_INET, option_bind_ip, &serv_addr.sin_addr);
			if (results != 1)
			{
				log_message(STREAM_MAIN, MSG_BAD, "Fatal error unable to bind to provided IP %s errno: %i\n",
						option_bind_ip,errno);
				return FALSE;
			}
		}
		else
		{
			serv_addr.sin_addr.s_addr = INADDR_ANY;
		}

		serv_addr.sin_family = AF_INET;

		serv_addr.sin_port = htons(listen_port);

		setsockopt(listen_sock_fd, SOL_SOCKET, SO_SNDTIMEO,
				(char *) &socket_timeout, sizeof(socket_timeout));
		setsockopt(listen_sock_fd, SOL_SOCKET, SO_RCVTIMEO,
				(char *) &socket_timeout, sizeof(socket_timeout));
		setsockopt(listen_sock_fd, SOL_SOCKET, SO_REUSEADDR, (char *) &bOptionTrue,
				sizeof(bOptionTrue));

		solinger.l_onoff = TRUE;
		solinger.l_linger = 0;
		setsockopt(listen_sock_fd, SOL_SOCKET, SO_LINGER, &solinger, sizeof(solinger));

		if (bind(listen_sock_fd, (struct sockaddr *) &serv_addr, sizeof(serv_addr))< 0)
		{
			log_message(STREAM_MAIN, MSG_BAD, "Fatal error binding to server port %i errno: %i\n",listen_port,errno);
			return FALSE;
		}

		listen(listen_sock_fd, listen_backlog);

		set_non_blocking(listen_sock_fd);
	}

	add_fd(listen_sock_fd, LISTEN_SOCKET);

	log_message(STREAM_MAIN, MSG_GOOD, "Listening socket created on port %i\n", listen_port);

	return TRUE;
}

/*
 Init serial port and add fd to our list of sockets
 */
int init_serial_fd(char * szPortPath)
{
	struct termios newtio;
	int id, x;
	long BAUD;

	int fd = open(szPortPath, O_RDWR | O_NOCTTY | O_NDELAY);

	if (fd < 0)
	{
		log_message(STREAM_MAIN, MSG_BAD, "Error can not open com port at %s errno: %i '%s'\n", szPortPath, errno, strerror(errno));
		return fd;
	}

	log_message(STREAM_MAIN, MSG_GOOD, "Opened com port at %s\n", szPortPath);

	/* add it and get our structure */
	id = add_fd(fd, SERIAL);

	if (id < 0)
	{
		log_message(STREAM_MAIN, MSG_BAD, "Error can not add the serial fd\n");
		close(fd);
		return 0;
	}

	// derived baud rate from command line default to B9600
	BAUD = get_baud(option_baud_rate);

	/* alredy done above in the open but Richard had this in his tests and it worked so
	 I am adding it back in for now maybe some bug in the os */
	fcntl(fd, F_SETFL, FNDELAY);

	/* backup the original terminal io settings to restore back to later
	 removed for now Richards mod didnt save the old value */
	tcgetattr(fd, &my_fds[id].oldtio);

	/* get the current terminal settings */
	tcgetattr(fd, &newtio);

	/* Set the baud rates */
	cfsetispeed(&newtio, BAUD);
	cfsetospeed(&newtio, BAUD);

	/* change our c_cflag settings a little for the port */
	newtio.c_cflag |= (CLOCAL | CREAD); /* Enable the receiver and set local mode */
	newtio.c_cflag &= ~PARENB; /* Mask the character size to 8 bits, no parity */
	newtio.c_cflag &= ~CSTOPB;
	newtio.c_cflag &= ~CSIZE;
	newtio.c_cflag |= CS8; /* Select 8 data bits */
	newtio.c_cflag &= ~CRTSCTS; /* Disable hardware flow control */
	if (option_debug_level > 2)
		log_message(STREAM_MAIN, MSG_WARN, "c_cflags old:%08x  new:%08x\n", my_fds[id].oldtio.c_cflag,
				newtio.c_cflag);

	/* change our c_lflag settings enable raw input mode */
	newtio.c_lflag &= ~(ICANON | ECHO | ISIG);
	if (option_debug_level > 2)
		log_message(STREAM_MAIN, MSG_WARN, "c_lflags old:%08x  new:%08x\n", my_fds[id].oldtio.c_lflag,
				newtio.c_lflag);

	/* change our c_iflag settings (turn off all flags) */
	newtio.c_iflag = 0;
	if (option_debug_level > 2)
		log_message(STREAM_MAIN, MSG_WARN, "c_iflags old:%08x  new:%08x\n", my_fds[id].oldtio.c_iflag,
				newtio.c_iflag);

	/* change our c_oflag settings (turn off all flags) */
	newtio.c_oflag = 0;
	if (option_debug_level > 2)
		log_message(STREAM_MAIN, MSG_WARN, "c_oflags old:%08x  new:%08x\n", my_fds[id].oldtio.c_oflag,
				newtio.c_oflag);

	/* dump bytes out of old c_cc */
	if (option_debug_level > 2)
	{
		log_message(STREAM_MAIN, MSG_WARN, "c_cc old: ");
		for (x = 0; x < sizeof(my_fds[id].oldtio.c_cc); x++)
		{
			log_message(STREAM_MAIN, MSG_WARN, "%02x:", my_fds[id].oldtio.c_cc[x]);
		}
		log_message(STREAM_MAIN, MSG_WARN, "\n");
	}

	newtio.c_cc[VINTR] = 0; /* Ctrl-c */
	newtio.c_cc[VQUIT] = 0; /* Ctrl-\ */
	newtio.c_cc[VERASE] = 0; /* del */
	newtio.c_cc[VKILL] = 0; /* @ */
	newtio.c_cc[VEOF] = 4; /* Ctrl-d */
	newtio.c_cc[VTIME] = 0; /* inter-character timer unused */
	newtio.c_cc[VMIN] = 1; /* blocking read until 1 character arrives */
# ifdef VSWTC
	newtio.c_cc[VSWTC] = 0;
# endif
	newtio.c_cc[VSTART] = 0; /* Ctrl-q */
	newtio.c_cc[VSTOP] = 0; /* Ctrl-s */
	newtio.c_cc[VSUSP] = 0; /* Ctrl-z */
	newtio.c_cc[VEOL] = 0; /* '\0' */
	newtio.c_cc[VREPRINT] = 0; /* Ctrl-r */
	newtio.c_cc[VDISCARD] = 0; /* Ctrl-u */
	newtio.c_cc[VWERASE] = 0; /* Ctrl-w */
	newtio.c_cc[VLNEXT] = 0; /* Ctrl-v */
	newtio.c_cc[VEOL2] = 0; /* '\0' */

	/* dump bytes out of new c_cc */
	if (option_debug_level > 2)
	{
		log_message(STREAM_MAIN, MSG_WARN, "c_cc new: ");
		for (x = 0; x < sizeof(newtio.c_cc); x++)
		{
			log_message(STREAM_MAIN, MSG_WARN, "%02x:", newtio.c_cc[x]);
		}
		log_message(STREAM_MAIN, MSG_WARN, "\n");
	}

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);
	if (option_debug_level > 2)
		print_serial_fd_status(fd);

	return 1;
}

/*
 prints out the specific serial fd terminal flags
 */
void print_serial_fd_status(int fd)
{
	int status;
	unsigned int arg;
	status = ioctl(fd, TIOCMGET, &arg);
	log_message(STREAM_MAIN, MSG_GOOD, "Serial status (%i) ",status);
	if (arg & TIOCM_RTS)
		log_message(STREAM_MAIN, MSG_GOOD, "RTS ");
	if (arg & TIOCM_CTS)
		log_message(STREAM_MAIN, MSG_GOOD, "CTS ");
	if (arg & TIOCM_DSR)
		log_message(STREAM_MAIN, MSG_GOOD, "DSR ");
	if (arg & TIOCM_CAR)
		log_message(STREAM_MAIN, MSG_GOOD, "DCD ");
	if (arg & TIOCM_DTR)
		log_message(STREAM_MAIN, MSG_GOOD, "DTR ");
	if (arg & TIOCM_RNG)
		log_message(STREAM_MAIN, MSG_GOOD, "RI ");
	log_message(STREAM_MAIN, MSG_GOOD, "\n");
}

/*
 gets the baud numeric from a string constant
 */
int get_baud(char * szbaud)
{
	speed_spec *s;
	int speed = 0;
	if (szbaud != 0)
	{
		for (s = speeds; s->name; s++)
		{
			if (strcmp(s->name, szbaud) == 0)
			{
				speed = s->flag;
				break;
			}
		}
	}
	/* default to 3 in our array or 9600 */
	if (speed == 0)
		s = &speeds[3];
	log_message(STREAM_MAIN, MSG_GOOD, "Setting speed %s\n", s->name);
	return s->flag;
}

/*
 Makes a fd non blocking
 */
void set_non_blocking(int fd)
{
	int nonb = 0;
	int res = 1;
	nonb |= O_NONBLOCK;
	if (ioctl(fd, FIONBIO, &res) < 0)
		error("Error setting FIONBIO");
}

/*
 Add a fd to our array so we can poll it in our state machien loop
 */
int add_fd(int fd, int fd_type)
{
	int x;
	int results = -1;
	struct linger solinger;

	for (x = 0; x < MAXCONNECTIONS; x++)
	{
		if (my_fds[x].inuse == FALSE)
		{
			if (option_debug_level > 2)
				log_message(STREAM_MAIN, MSG_WARN, "Adding %s fd at %i\n", fd_type_strings[fd_type], x);

			if (fd_type != SERIAL)
			{
				solinger.l_onoff = TRUE;
				solinger.l_linger = 0;
				setsockopt(fd, SOL_SOCKET, SO_LINGER, &solinger, sizeof(solinger));
			}

			my_fds[x].inuse = TRUE;
			my_fds[x].fd_type = fd_type;
			my_fds[x].fd = fd;
			my_fds[x].new = TRUE;
			results = x;
			break;
		}
	}

	return results;
}

/*
 Cleanup an entry in the fd array and do any fd_type specific cleanup
 */
int cleanup_fd(int n)
{

	/* don't do anything unless its in was active */
	if (my_fds[n].inuse)
	{

		/* if this is a terminal or serial fd then restore its settings */
		if (my_fds[n].fd_type == SERIAL)
		{
			tcsetattr(my_fds[n].fd, TCSANOW, &my_fds[n].oldtio);
			serial_connected = FALSE;
		}

#ifdef HAVE_LIBSSL
		if (my_fds[n].ssl != NULL)
			shutdown_ssl_conn(my_fds[n].ssl);
		my_fds[n].ssl = NULL;
		my_fds[n].handshake_done = FALSE;
#endif
		/* close the fd */
		close(my_fds[n].fd);
		my_fds[n].fd = -1;

		/* clear any data we have saved */
		fifo_clear(&my_fds[n].send_buffer);

		/* mark the element as free for reuse */
		my_fds[n].inuse = FALSE;

		/* set the type to null */
		my_fds[n].fd_type = NA;

	}
	return TRUE;
}

/*
 Diff two time values
 */
long get_time_difference(struct timeval *startTime)
{
	struct timeval endTime;
	long seconds, nseconds;

	gettimeofday(&endTime, NULL);

	seconds = endTime.tv_sec - startTime->tv_sec;
	nseconds = endTime.tv_usec - startTime->tv_usec;

	return seconds * 1000 + nseconds / 1000;
}

#define clear_serial(n) \
		tv_serial_start.tv_sec = 0; \
		tv_serial_start.tv_usec = 0; \
		serial_connected = 0; \
		cleanup_fd(n); \
		add_to_all_socket_fds("\r\n"); \
		add_to_all_socket_fds(SERIAL_DISCONNECTED_MSG); \
		msleep(100); \

/*
 check for a hup signal and hup work if needed
 */
BOOL hup_check()
{

	/* did we get a hup signal? */
	if (!got_hup)
	  return FALSE;

	/* clear it */
	got_hup = 0;

	free_system();
	read_config(CONFIG_PATH);
	init_system();
	init_listen_socket_fd();
	return TRUE;
}

/*
 poll the serial port reconnect if needed
 */
void poll_serial_port()
{
	/* if our port is not connected check if we should try to reconnect */
	if (!serial_connected)
	{
		if ((tv_serial_start.tv_sec == 0) || (get_time_difference(&tv_serial_start)
				>= option_open_serial_delay))
		{
			gettimeofday(&tv_serial_start, NULL);
			if (init_serial_fd(serial_device_name) > 0)
			{
				serial_connected = 1;
				tv_last_serial_check.tv_sec = 0;
				tv_last_serial_check.tv_usec = 0;
				line_ended=0;
				add_to_all_socket_fds(SERIAL_CONNECTED_MSG);
			}
			else
				msleep(10);
		}
		else
			msleep(10);
		return;
	}

#ifdef USE_TIOCMGET
	int n,tmp;
	/* periodic serial device checking */
	if ((tv_last_serial_check.tv_sec == 0) || (get_time_difference(
			&tv_last_serial_check) >= 100))
	{
		errno = 0;
		gettimeofday(&tv_last_serial_check, NULL);
		for (n = 0; n < MAXCONNECTIONS; n++)
		{
			if (my_fds[n].fd_type == SERIAL && my_fds[n].inuse == TRUE)
			{
				if (ioctl(my_fds[n].fd, TIOCMGET, &tmp) < 0)
				{
					log_message(STREAM_MAIN, MSG_WARN, "Serial disconnected on check. errno: %i '%s'\n", errno, strerror(errno));
					clear_serial(n);
				}
				/* currently only 1 serial port so we are done */
				break;
			}
		}
	}
#endif
}

/*
 add all of our fd to our r,w and e fd sets
*/
void build_fdsets(fd_set *read_fdset, fd_set *write_fdset, fd_set *except_fdset)
{
	int n;

	/* add all sockets to our fdset */
	FD_ZERO(read_fdset);
	FD_ZERO(write_fdset);
	FD_ZERO(except_fdset);
	for (n = 0; n < MAXCONNECTIONS; n++)
	{
		if (my_fds[n].inuse == TRUE)
		{
			FD_SET(my_fds[n].fd,read_fdset);
			FD_SET(my_fds[n].fd,write_fdset);
			FD_SET(my_fds[n].fd,except_fdset);
		}
	}
}

/*
 poll any exception fd's return TRUE if we did some work
 */
BOOL poll_exception_fdset(fd_set *except_fdset)
{
	int n;
	BOOL did_work = FALSE;

	for (n = 0; n < MAXCONNECTIONS; n++)
	{
		if (my_fds[n].inuse == TRUE)
		{
			if (FD_ISSET(my_fds[n].fd,except_fdset))
			{
				if (my_fds[n].fd_type == CLIENT_SOCKET)
				{
					did_work = TRUE;
					log_message(STREAM_MAIN, MSG_WARN, "Exception occured on socket fd slot %i closing the socket.\n",n);
					cleanup_fd(n);
				}
			}
		}
	}
	return did_work;
}

/*
  poll any read fd's return TRUE if we did do some work
 */
BOOL poll_read_fdset(fd_set *read_fdset)
{
	int x, n, received, newsockfd, added_slot;
	unsigned int clilen;
	byte_t *tempbuffer;
	BOOL did_work = FALSE;
	byte_t buffer[1024];

#ifdef HAVE_LIBSSL
	BIO* newbio = 0;
#endif

	clilen = sizeof(struct sockaddr_in);

	/* check every socket to find the one that needs read */
	for (n = 0; n < MAXCONNECTIONS; n++)
	{
		if (my_fds[n].inuse == TRUE)
		{

			/* check read fd */
			if (FD_ISSET(my_fds[n].fd,read_fdset))
			{
				/*  if this is a listening socket then we accept on it and
				 * get a new client socket
				 */
				if (my_fds[n].fd_type == LISTEN_SOCKET)
				{
					/* clear our state vars */
					newsockfd = -1;
#ifdef HAVE_LIBSSL
					newbio = NULL;
					if (option_ssl)
					{
						if (BIO_do_accept(abio) <= 0)
						{
							log_message(STREAM_MAIN, MSG_BAD, "SSL BIO_do_accept failed: %s\n",
								    ERR_error_string(ERR_get_error(), NULL));
						} else
						{
							// try and grab our actual working BIO.
							newbio = BIO_pop(abio);

							if (!newbio)
							{
								log_message(STREAM_MAIN, MSG_BAD, "SSL BIO_pop failed: %s\n",
								    ERR_error_string(ERR_get_error(), NULL));
							} else
							{
								/* get our fd from the BIO */
								BIO_get_fd(newbio, &newsockfd);
							}
						}
					}
					else
#endif
					{
						newsockfd = accept(listen_sock_fd, (struct sockaddr *) &peer_addr, &clilen);
					}
					if (newsockfd != -1)
					{
						if (serial_connected || option_keep_connection)
						{
							/* reset our added id to a bad state */
							added_slot = -2;
#ifdef HAVE_LIBSSL
							if (option_ssl)
							{
								// tell the SSL state machine to start to handshake
								if (BIO_do_handshake(newbio) <= 0)
								{
									if (!BIO_should_retry(newbio))
									{
										log_message(STREAM_MAIN, MSG_BAD, "SSL handshake failed with no retry: %s\n",
											    ERR_error_string(ERR_get_error(), NULL));
										shutdown_ssl_conn(newbio);
										close(newsockfd);
									} else
									{
										added_slot = add_fd(newsockfd, CLIENT_SOCKET);;
									}
								}
							} else
#endif
							{
								added_slot = add_fd(newsockfd, CLIENT_SOCKET);
							}
							if (added_slot >= 0)
							{
#ifdef HAVE_LIBSSL
								if (option_ssl && newbio != NULL)
									my_fds[added_slot].ssl = newbio;
#endif
								log_message(STREAM_MAIN, MSG_GOOD, "Socket connected slot %i\n",added_slot);
								/* adding anything to the fifo must be pre allocated */
								if (option_send_terminal_init)
								{
									tempbuffer = strdup("!");
									fifo_add(
											&my_fds[added_slot].send_buffer,
											tempbuffer);
									tempbuffer = strdup(
											terminal_init_string);
									fifo_add(
											&my_fds[added_slot].send_buffer,
											tempbuffer);
									tempbuffer = strdup("\r\n");
									fifo_add(
											&my_fds[added_slot].send_buffer,
											tempbuffer);
								}

								tempbuffer = strdup(
										"!SER2SOCK Connected\r\n");
								fifo_add(&my_fds[added_slot].send_buffer,
										tempbuffer);
								if (serial_connected)
									tempbuffer = strdup(
											SERIAL_CONNECTED_MSG);
								else
									tempbuffer = strdup(
											SERIAL_DISCONNECTED_MSG);
								fifo_add(&my_fds[added_slot].send_buffer,
										tempbuffer);
								did_work = TRUE;
							}
							else
							{
#ifdef HAVE_LIBSSL
								if (newbio != NULL)
									shutdown_ssl_conn(newbio);
#endif
								close(newsockfd);
								if(added_slot == -1)
								log_message(STREAM_MAIN, MSG_WARN, "Socket refused because no more space\n");
							}
						}
						else
						{
#ifdef HAVE_LIBSSL
							if (newbio != NULL)
							      shutdown_ssl_conn(newbio);
#endif
							close(newsockfd);
							log_message(STREAM_MAIN, MSG_WARN, "Socket refused because serial is not connected\n");
						}
					}

				}
				else
				{
					if (my_fds[n].fd_type == SERIAL)
					{
						errno = 0;
						while ((received = read(my_fds[n].fd, buffer,
								sizeof(buffer))) > 0)
						{
							if (received > 0)
							{
								did_work = TRUE;
								buffer[received] = 0;
								add_to_all_socket_fds(buffer);
								if (option_debug_level > 1)
								{
									if (option_debug_level > 2)
									{
										log_message(STREAM_MAIN, MSG_WARN, "SERIAL>");
										for (x = 0; x < received; x++)
										{
											log_message(STREAM_MAIN, MSG_WARN, "[%02x]",
													buffer[x]);
										}
										log_message(STREAM_MAIN, MSG_WARN, "\n");
									}
									else
									{
										log_message(STREAM_SERIAL, MSG_WARN, "%s", buffer);
									}
								}
							}
						}

						if (received < 0)
						{
							if (errno != EAGAIN)
							{
								log_message(STREAM_MAIN, MSG_WARN,
										"Serial disconnected on read. errno: %i '%s'\n",
										errno, strerror(errno));
								clear_serial(n);
							}
						}
					}
					else
					{
						errno = 0;
#ifdef HAVE_LIBSSL
						if (option_ssl)
						{
							received = BIO_read(my_fds[n].ssl, buffer, sizeof(buffer));
							if (received <= 0 && BIO_should_retry(my_fds[n].ssl))
									continue;
						}
						else
#endif
						{
							received = recv(my_fds[n].fd, buffer, sizeof(buffer), 0);
						}
						if (received == 0)
						{
							log_message(STREAM_MAIN, MSG_WARN, "Closing socket fd slot %i errno: %i '%s'\n", n,
									errno, strerror(errno));
							cleanup_fd(n);
						}
						else
						{
							if (received < 0)
							{
								if (errno == EAGAIN || errno == EINTR)
									continue;
								log_message(STREAM_MAIN, MSG_WARN,
										"Closing socket errno: %i '%s'\n",
										errno, strerror(errno));
								cleanup_fd(n);
							}
							else
							{
								did_work = TRUE;
								buffer[received] = 0;
								add_to_serial_fd(buffer);
								if (option_debug_level > 2)
								{
									log_message(STREAM_MAIN, MSG_WARN, "SOCKET[%i]>", n);
									for (x = 0; x < strlen(buffer); x++)
									{
										log_message(STREAM_MAIN, MSG_WARN, "[%02x]", buffer[x]);
									}
									log_message(STREAM_MAIN, MSG_WARN, "\n");
								}
							}
						}
					}
				}
			} /* end FD_ISSET() */
		}
	}

	return did_work;
}

/*
  poll all write fd's return TRUE if we did do some work
 */
BOOL poll_write_fdset(fd_set *write_fdset)
{
	int x, n, written;
	byte_t *tempbuffer;
	BOOL did_work = FALSE;

	/* check every socket to find the one that needs write */
	for (n = 0; n < MAXCONNECTIONS; n++)
	{
		if (my_fds[n].inuse == TRUE && FD_ISSET(my_fds[n].fd,write_fdset))
		{
			/* see if we have data to write */
			if (!fifo_empty(&my_fds[n].send_buffer))
			{
				/* set our var to an invalid state */
				tempbuffer = NULL;

				/* handle writing to CLIENT_SOCKET */
				if (my_fds[n].fd_type == CLIENT_SOCKET)
				{
#ifdef HAVE_LIBSSL
					if (option_ssl)
					{
						/* dont try and send till we are ready */
						if(my_fds[n].handshake_done)
						{
							/* load our buffer with data to send */
							tempbuffer = (char *) fifo_get(
									&my_fds[n].send_buffer);
							written = BIO_write(my_fds[n].ssl, tempbuffer, strlen(tempbuffer));
							if (written <= 0 && BIO_should_retry(my_fds[n].ssl))
								continue;
						} else
						{
							if (BIO_do_handshake(my_fds[n].ssl) <= 0)
							{
								if (!BIO_should_retry(my_fds[n].ssl))
								{
									log_message(STREAM_MAIN, MSG_BAD, "SSL handshake failed with no retry: %s\n",
										    ERR_error_string(ERR_get_error(), NULL));
									cleanup_fd(n);
								}
							} else
							{
								my_fds[n].handshake_done = TRUE;
							}
						}
					}
					else
#endif
					/* load our buffer with data to send */
					{
						tempbuffer = (char *) fifo_get(
								&my_fds[n].send_buffer);
						send(my_fds[n].fd, tempbuffer, strlen(tempbuffer), 0);
					}

					/* did we do any work? */
					if ( tempbuffer )
					{
						did_work = TRUE;
						if (option_debug_level > 2)
							log_message(STREAM_MAIN, MSG_WARN, "SOCKET[%i]<", n);
					}
				}

				/* handle writes to SERIAL */
				if (my_fds[n].fd_type == SERIAL)
				{
					/* load our buffer with data to send */
					tempbuffer = (char *) fifo_get(
							&my_fds[n].send_buffer);
					errno = 0;
					did_work = TRUE;
					if (option_debug_level > 2)
						log_message(STREAM_MAIN, MSG_WARN, "SERIAL[%i]<", n);
					if (write(my_fds[n].fd, tempbuffer, strlen(
							tempbuffer)) < 0)
					{
						if (errno != EAGAIN)
						{
							log_message(STREAM_MAIN, MSG_BAD,
									"Serial disconnected on write. errno: %i '%s'\n",
									errno, strerror(errno));
							clear_serial(n);
						}
					}
				}

				/* logging if needed */
				if (option_debug_level > 2 && tempbuffer)
				{
					for (x = 0; x < strlen(tempbuffer); x++)
					{
						log_message(STREAM_MAIN, MSG_WARN, "[%02x]", tempbuffer[x]);
					}
					log_message(STREAM_MAIN, MSG_WARN, "\n");
				}

				/* free up memory */
				if(tempbuffer)
					free(tempbuffer);
			}
			else
			{
				/* if serial disconnected and not option_keep_connected
				   then disconnect the client
				 */
				if (!serial_connected && !option_keep_connection)
				{
					if (my_fds[n].fd_type == CLIENT_SOCKET)
					{
						cleanup_fd(n);
					}
				}
			}
		}
	}

	return did_work;
}

/*
 this loop polls all of our fd's
 */
void listen_loop()
{
	int n;
	BOOL did_work=FALSE,reset_state=TRUE;
	fd_set read_fdset, write_fdset, except_fdset;
	struct timeval wait;

#ifdef _POSIX_SOURCE
	// Set high thread priority
	struct sched_param param;

	log_message(STREAM_MAIN, MSG_GOOD, "Seting thread priority to HIGH\n");
	memset(&param,0,sizeof(param));
	param.__sched_priority = sched_get_priority_min(SCHED_RR);
	sched_setscheduler(0,SCHED_RR,&param);
#endif

	log_message(STREAM_MAIN, MSG_GOOD, "Start wait loop\n");

	/* continue polling until interrupted */
	while (!kbhit())
	{
		/* reset our loop state var(s) for this iteration */
		did_work = FALSE;

		/* check for hup signal */
		if(hup_check())
			reset_state = TRUE;

		/* reset state if asked */
		if (reset_state) {
			line_ended=0;
			tv_serial_start.tv_sec = 0;
			tv_serial_start.tv_usec = 0;
			tv_last_serial_check.tv_sec = 0;
			tv_last_serial_check.tv_usec = 0;
			reset_state=FALSE;
		}

		/* poll our serial port reconnect if needed */
		poll_serial_port();

		/* build our fd sets */
		build_fdsets(&read_fdset, &write_fdset, &except_fdset);

		/* lets not block our select and bail after 20us */
		wait.tv_sec = 0; wait.tv_usec = 20;

		/* see if any of the fd's need attention */
		n = select(FD_SETSIZE, &read_fdset, &write_fdset, &except_fdset, &wait);
		if (n == -1)
		{
			log_message(STREAM_MAIN, MSG_BAD, "An error occured during select() errno: %i '%s'\n", errno, strerror(errno));
			continue;
		}

		/* poll our exception fdset */
		poll_exception_fdset(&except_fdset);

		/* poll our read fdset */
		did_work = poll_read_fdset(&read_fdset);

		/* poll our write fdset */
		did_work = poll_write_fdset(&write_fdset);

		/* if we did not do anything then sleep a little predict
		   next go round will be idle too
		*/
		if (!did_work)
			msleep(20);
	}

	log_message(STREAM_MAIN, MSG_NONE, "\n");
	log_message(STREAM_MAIN, MSG_GOOD, "cleaning up\n");
	free_system();

	log_message(STREAM_MAIN, MSG_GOOD, "done.\n");
}

/*
 add_to_all_socket_fds
 adds a buffer to ever connected socket fd ie multiplexes
 */
void add_to_all_socket_fds(char * message)
{

	char * tempbuffer;
	char * location;
	int n;
	/*
	 Adding anything to the fifo must be allocated so it can be free'd later
	 Not very efficient but we have plenty of mem with as few connections as we
	 will use. If we needed many more I would need to re-factor this code
	 */
	for (n = 0; n < MAXCONNECTIONS; n++)
	{
		if (my_fds[n].inuse == TRUE)
		{
			if (my_fds[n].fd_type == CLIENT_SOCKET)
			{
				/* caller of fifo_get must free this */
				if (line_ended || !my_fds[n].new)
				{
					if (my_fds[n].new)
						my_fds[n].new = FALSE;
					tempbuffer = strdup(message);
					fifo_add(&my_fds[n].send_buffer, tempbuffer);
				}
				else
				{
					/*
					 wait for our first \n to start on a clean line. We are skipping
					 our first message so we dont send a partial message
					 */
					location = strchr(message, '\n');
					if (location != NULL)
					{
						tempbuffer = strdup(location);
						fifo_add(&my_fds[n].send_buffer, tempbuffer);
						my_fds[n].new = FALSE;
					}
				}
			}
		}
	}
	line_ended = 0;
	if (message)
	{
		location = message + strlen(message) - 1;
		if (location)
		{
			if ((*location) == '\n')
				line_ended = 1;
		}
	}
}

/*
 adds data to the serial fifo buffer should be at 0 ever time
 */
void add_to_serial_fd(char *message)
{
	int n;
	char * tempbuffer;
	for (n = 0; n < MAXCONNECTIONS; n++)
	{
		if (my_fds[n].inuse == TRUE)
		{
			if (my_fds[n].fd_type == SERIAL)
			{
				tempbuffer = strdup(message);
				fifo_add(&my_fds[n].send_buffer, tempbuffer);
				break;
			}
		}
	}
}

/*
 skip over ' ' if the add one to the option
 */
int skip_param(char *buf)
{
	int x = 0;
	/* did they do -afoobar or -a foobar */
	if (buf[2] != 0)
		return 2;

	/* skip any other whitespace */
	x = 3;
	while (buf[x] == ' ' && x < 40)
		x++;
	if (x < 40)
		return x;

	return 0;
}

/*
 parse_args
 */
int parse_args(int argc, char * argv[])
{

	char **loc_argv = argv;
	int loc_argc = argc;
	int skip;

	while (loc_argc > 1)
	{
		if (loc_argv[1][0] == '-')
			switch (loc_argv[1][1])
			{
				case 'h':
					show_help(argv[0]);
					exit(EXIT_SUCCESS);
					break;
				case 'p':
					skip = skip_param(&loc_argv[1][0]);
					listen_port = atoi(&loc_argv[1][skip]);
					break;
				case 's':
					skip = skip_param(&loc_argv[1][0]);
					serial_device_name = &loc_argv[1][skip];
					break;
				case 'i':
					skip = skip_param(&loc_argv[1][0]);
					option_bind_ip = &loc_argv[1][skip];
					break;
				case 'b':
					skip = skip_param(&loc_argv[1][0]);
					option_baud_rate = &loc_argv[1][skip];
					break;
				case 'd':
					option_daemonize = TRUE;
					break;
				case 't':
					option_send_terminal_init = TRUE;
					break;
				case 'g':
					skip = skip_param(&loc_argv[1][0]);
					option_debug_level = atoi(&loc_argv[1][skip]);
					break;
				case 'w':
					skip = skip_param(&loc_argv[1][0]);
					option_open_serial_delay = atoi(&loc_argv[1][skip]);
					break;
				case 'c':
					option_keep_connection = TRUE;
					break;
#ifdef HAVE_LIBSSL
				case 'e':
					option_ssl = TRUE;
					break;
#endif

				default:
					show_help(argv[0]);
					error("ERROR Wrong argument: %s", loc_argv[1]);
					exit(EXIT_FAILURE);
			}

		++loc_argv;
		--loc_argc;
	}

	if (serial_device_name == 0)
	{
		show_help(argv[0]);
		log_message(STREAM_MAIN, MSG_BAD, "Error missing serial device name exiting\n");
		exit(EXIT_FAILURE);
	}
	return TRUE;
}

/*
 writepid
 */
static void writepid(void)
{
	int fd;
	char buff[20];
	if ((fd = open(PID_FILE, O_CREAT | O_WRONLY, 0600)) >= 0)
	{
		snprintf(buff, 20, "%d\n", (int) getpid());
		if (write(fd, buff, strlen(buff)) == -1)
			log_message(STREAM_MAIN, MSG_WARN, "Error writing to pid file %s\n", PID_FILE);
		close(fd);
		return;
	}
	else
		log_message(STREAM_MAIN, MSG_WARN, "Error opening pid file %s\n", PID_FILE);
}

/*
 main()
 */
int main(int argc, char *argv[])
{
	BOOL config_read = read_config(CONFIG_PATH);

	/* parse args and set global vars as needed */
	parse_args(argc, argv);

	if(config_read)
		log_message(STREAM_MAIN, MSG_GOOD, "Using config file: %s\n", CONFIG_PATH);


	/* startup banner and args check */
	log_message(STREAM_MAIN, MSG_GOOD, "Serial 2 Socket Relay version %s starting\n", SER2SOCK_VERSION);
	if (!config_read && argc < 2)
	{
		show_help(argv[0]);
		log_message(STREAM_MAIN, MSG_BAD, "ERROR insufficient arguments\n");
		exit(EXIT_FAILURE);
	}

	/* initialize the system */
	init_system();

	/* initialize our listening socket */
	if (!init_listen_socket_fd())
	{
		error("ERROR initializing listen socket");
	}

	if (option_daemonize)
	{
		log_message(STREAM_MAIN, MSG_GOOD, "daemonizing the process\n");

		/* Fork off the parent process */
		pid = fork();
		if (pid < 0)
		{
			exit(EXIT_FAILURE);
		}
		/* If we got a good PID, then
		 we can exit the parent process. */
		if (pid > 0)
		{
			closelog();
			exit(EXIT_SUCCESS);
		}
		else
		{
			/* close and re open with different settings no LOG_PERROR */
			closelog();
			openlog(DAEMON_NAME, LOG_CONS | LOG_NDELAY | LOG_PID, LOG_USER);
		}

		/* Change the file mode mask */
		umask(0);

		/* Create a new SID for the child process */
		sid = setsid();
		if (sid < 0)
		{
			/* Log the failure */
			exit(EXIT_FAILURE);
		}

		/* Change the current working directory */
		if ((chdir("/")) < 0)
		{
			/* Log the failure */
			exit(EXIT_FAILURE);
		}

		/* Close out the standard file descriptors */
		close(STDIN_FILENO);
		close(STDOUT_FILENO);
		close(STDERR_FILENO);

		/* write our pid file */
		writepid();
	}

	/* begin our listen loop */
	listen_loop();

	if (option_daemonize)
	{
		closelog();
	}

	return 0;
}

/*
 signal handlers
 */
void signal_handler(int sig)
{
	switch (sig)
	{
		case SIGHUP:
			log_message(STREAM_MAIN, MSG_WARN, "Received SIGHUP signal.\n");
			got_hup = 1;
			break;
		case SIGTERM:
			log_message(STREAM_MAIN, MSG_WARN, "Received SIGTERM signal.\n");
			log_message(STREAM_MAIN, MSG_GOOD, "Cleaning up\n");
			free_system();
			log_message(STREAM_MAIN, MSG_GOOD, "done.\n");
			exit(EXIT_SUCCESS);
			break;
		default:
			log_message(STREAM_MAIN, MSG_WARN, "Unhandled signal (%d) %s\n", sig, strsignal(sig));
			break;
	}
}

/*
 Configuration parsing
 */

char* trim(char* string)
{
	char* end = 0;

	while(isspace(*string)) string++;

	if (*string == 0)
		return string;

	end = string + strlen(string) - 1;

	while(isspace(*end)) end--;

	*(end + 1) = 0;

	return string;
}

BOOL read_config(char* filename)
{
	FILE* fp = NULL;
	char buffer[256];

	char* bufp = NULL;
	char* opt = NULL;
	char* optdata = NULL;

	if ((fp = fopen(filename, "r")) != NULL)
	{

		while (!feof(fp))
		{
			if (fgets(buffer, 256, fp))
			{
				bufp = trim(buffer);

				// skip comments and section headers
				if(bufp[0] == '#' || bufp[0] == ';' || bufp[0] == '[')
					continue;

				// Split on =
				opt = strtok(bufp, "=");
				if (opt)
				{
					opt = trim(opt);
					optdata = strtok(NULL, "=");

					if (optdata)
					{
						optdata = trim(optdata);

						if (!strcmp(opt, "daemonize"))
						{
							option_daemonize = atoi(optdata);
						}
						else if (!strcmp(opt, "port"))
						{
							listen_port = atoi(optdata);
						}
						else if (!strcmp(opt, "device"))
						{
							serial_device_name = strdup(optdata);
						}
						else if (!strcmp(opt, "bind_ip"))
						{
							option_bind_ip = strdup(optdata);
						}
						else if (!strcmp(opt, "baudrate"))
						{
							option_baud_rate = strdup(optdata);
						}
						else if (!strcmp(opt, "send_terminal_init"))
						{
							option_send_terminal_init = atoi(optdata);
						}
						else if (!strcmp(opt, "preserve_connections"))
						{
							option_keep_connection = atoi(optdata);
						}
						else if (!strcmp(opt, "device_open_delay"))
						{
							option_open_serial_delay = atoi(optdata);
						}

#ifdef HAVE_LIBSSL
						else if (!strcmp(opt, "encrypted"))
						{
							option_ssl = atoi(optdata);
						}
						else if (!strcmp(opt, "ca_certificate"))
						{
							option_ca_certificate = strdup(optdata);
						}
						else if (!strcmp(opt, "ssl_certificate"))
						{
							option_ssl_certificate = strdup(optdata);
						}
						else if (!strcmp(opt, "ssl_key"))
						{
							option_ssl_key = strdup(optdata);
						}
#endif
					}
				}
			}
		}

		fclose(fp);

		return TRUE;
	}

	return FALSE;
}

/*
 good old kbhit function just like back in the day
 */
int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	if (option_daemonize)
		return 0;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO | ISIG);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}

//<Fifo Buffer>
/*
 init queue allocate memory
 */
void fifo_init(fifo *f, int size)
{
	f->avail = 0;
	f->in = 0;
	f->out = 0;
	f->size = size;
	f->table = (void**) malloc(f->size * sizeof(void*));
}

/*
 fifo empty if queue = 1 else 0
 */
int fifo_empty(fifo *f)
{
	return (f->avail == 0);
}

/*
 free up any memory we allocated
 */
void fifo_destroy(fifo *f)
{
	int i;
	if (!fifo_empty(f))
		free(f->table);
	else
	{
		for (i = f->out; i < f->in; i++)
		{
			/* free actual block of memory */
			free(f->table[i]);
		}
		free(f->table);
	}
}

/*
 remove all stored pending data
 */
void fifo_clear(fifo *f)
{
	void *p;
	if (option_debug_level > 2)
		log_message(STREAM_MAIN, MSG_GOOD, "Clearing fifo queue: ");
	while (!fifo_empty(f))
	{
		if (option_debug_level > 2)
			log_message(STREAM_MAIN, MSG_GOOD, "*");
		p = fifo_get(f);
		if (p)
			free(p);
	}
	if (option_debug_level > 2)
		log_message(STREAM_MAIN, MSG_GOOD, " done.\n");
}

/*
 insert an element
 this must be already allocated with malloc or strdup
 */
int fifo_add(fifo *f, void *next)
{
	if (f->avail == f->size)
	{
		return (0);
	}
	else
	{
		f->table[f->in] = next;
		f->avail++;
		f->in = (f->in + 1) % f->size;
		return (1);
	}
}

/*
 return next element
 */
void* fifo_get(fifo *f)
{
	void* get;
	if (f->avail > 0)
	{
		get = f->table[f->out];
		f->out = (f->out + 1) % f->size;
		f->avail--;
		return (get);
	}
	return 0;
}

//</Fifo Buffer>

#ifdef HAVE_LIBSSL
/*
 initialize the ssl library
 */
BOOL init_ssl()
{
	SSL* ssl;
	char port_string[256];
	int use_cert = 0;
	int use_prv = 0;

	// Initialize SSL
	SSL_load_error_strings();
	SSL_library_init();
	OpenSSL_add_all_algorithms();

	// Set up our SSL context.
	sslctx = SSL_CTX_new(TLSv1_server_method());

	SSL_CTX_set_options(sslctx, SSL_OP_SINGLE_DH_USE);

	// Load our certificates
	use_cert = SSL_CTX_use_certificate_file(sslctx, option_ssl_certificate , SSL_FILETYPE_PEM);
	if (use_cert <= 0)
	{
		log_message(STREAM_MAIN, MSG_BAD, "Loading certificate failed: %s\n", ERR_error_string(ERR_get_error(), NULL));
		return FALSE;
	}

	use_prv = SSL_CTX_use_PrivateKey_file(sslctx, option_ssl_key, SSL_FILETYPE_PEM);
	if (use_prv <= 0)
	{
		log_message(STREAM_MAIN, MSG_BAD, "Loading private key failed: %s\n", ERR_error_string(ERR_get_error(), NULL));
		return FALSE;
	}

	// Specify required CAs
	SSL_CTX_set_client_CA_list(sslctx, SSL_load_client_CA_file(option_ca_certificate));

	// Load trusted CA.
	if (!SSL_CTX_load_verify_locations(sslctx, option_ca_certificate, NULL))
	{
		log_message(STREAM_MAIN, MSG_BAD, "Loading CA cert failed: %s\n", ERR_error_string(ERR_get_error(), NULL));
		return FALSE;
	}

	// Set to require peer (client) certificate verification
	SSL_CTX_set_verify(sslctx, SSL_VERIFY_PEER|SSL_VERIFY_FAIL_IF_NO_PEER_CERT, NULL);

	// Set the verification depth to 1
	SSL_CTX_set_verify_depth(sslctx, 1);

	// Set everything up to serve.
	bio = BIO_new_ssl(sslctx, 0);
	BIO_get_ssl(bio, &ssl);
	SSL_set_mode(ssl, SSL_MODE_AUTO_RETRY);
	BIO_set_nbio(bio, 1);	// Non-blocking on.

#ifdef SSL_DEBUGGING
	/* debug ssl do not use in production just testing */
	BIO_set_callback(bio,tls_bio_dump_cb);
	SSL_set_info_callback(ssl, apps_ssl_info_callback);
	SSL_set_msg_callback(ssl, ssl_msg_callback);
#endif

	sprintf(port_string, "%d", listen_port);
	abio = BIO_new_accept(port_string);
	BIO_set_accept_bios(abio, bio);
	BIO_set_nbio(abio, 1);

	// NOTE: This first accept is required.
	if (BIO_do_accept(abio) <= 0)
	{
		log_message(STREAM_MAIN, MSG_BAD, "SSL accept-1 failed: %s\n", ERR_error_string(ERR_get_error(), NULL));
		return FALSE;
	}

	// Save our listening socket descriptor.
	BIO_get_fd(abio, &listen_sock_fd);

	return TRUE;
}

/*
 cleanup the ssl library handles
 */
void shutdown_ssl()
{
	BIO_free_all(abio);
	SSL_CTX_free(sslctx);
	ERR_free_strings();
	EVP_cleanup();
}

/*
 cleanup an ssl connection and its handles
 */
void shutdown_ssl_conn(BIO* sslbio)
{
	BIO *tmp=BIO_pop(sslbio);
	BIO_free_all(tmp);
}

#ifdef SSL_DEBUGGING
/*
 * Every sent & received message this callback function is invoked,
 * so we know when alert messages have arrived or are sent and
 * we can print debug information about TLS handshake. This is
 * debug code and should not be used in production it is not
 * well tested.
 */
void ssl_msg_callback(int write_p, int version, int content_type,
		 const void *buf, size_t len, SSL * ssl, void *arg)
{
	char string[2048];
	char sbuf[2048];
	unsigned char code;
	bzero(string, sizeof(string));
	strcat(string,"msg_callback");
	if(write_p)
		strcat(string," -> ");
	else
		strcat(string," <- ");


	switch(content_type) {

	case SSL3_RT_ALERT:
		strcat(string, "Alert: ");
		code = ((const unsigned char *)buf)[1];
		strcat(string, SSL_alert_desc_string_long(code));
		break;

	case SSL3_RT_CHANGE_CIPHER_SPEC:
		strcat(string, "ChangeCipherSpec");
		break;

	case SSL3_RT_HANDSHAKE:

		strcat(string, "Handshake: ");
		code = ((const unsigned char *)buf)[0];

		switch(code) {
			case SSL3_MT_HELLO_REQUEST:
				strcat(string,"Hello Request");
				break;
			case SSL3_MT_CLIENT_HELLO:
				strcat(string,"Client Hello");
				break;
			case SSL3_MT_SERVER_HELLO:
				strcat(string,"Server Hello");
				break;
			case SSL3_MT_CERTIFICATE:
				strcat(string,"Certificate");
				break;
			case SSL3_MT_SERVER_KEY_EXCHANGE:
				strcat(string,"Server Key Exchange");
				break;
			case SSL3_MT_CERTIFICATE_REQUEST:
				strcat(string,"Certificate Request");
				break;
			case SSL3_MT_SERVER_DONE:
				strcat(string,"Server Hello Done");
				break;
			case SSL3_MT_CERTIFICATE_VERIFY:
				strcat(string,"Certificate Verify");
				break;
			case SSL3_MT_CLIENT_KEY_EXCHANGE:
				strcat(string,"Client Key Exchange");
				break;
			case SSL3_MT_FINISHED:
				strcat(string,"Finished");
				break;

			default:
				sprintf( sbuf, "Handshake: Unknown SSL3 code received: %d", code );
				strcat( string, sbuf);
		}
		break;

	default:
		sprintf( sbuf, "SSL message contains unknown content type: %d", content_type );
		strcat( string, sbuf);
	}

	log_message(STREAM_MAIN, MSG_WARN, "%s\n", string);

}

long    tls_bio_dump_cb(BIO *bio, int cmd, const char *argp, int argi,
			long unused_argl, long ret)
{
	if (cmd == (BIO_CB_READ | BIO_CB_RETURN)) {
		log_message(STREAM_MAIN, MSG_WARN,
		    "read from %08lX [%08lX] (%d bytes => %ld (0x%lX))\n",
		 (unsigned long) bio, (unsigned long) argp, argi,
		 ret, (unsigned long) ret);
	} else if (cmd == (BIO_CB_WRITE | BIO_CB_RETURN)) {
	log_message(STREAM_MAIN, MSG_WARN,
		    "write to %08lX [%08lX] (%d bytes => %ld (0x%lX))\n",
		 (unsigned long) bio, (unsigned long) argp, argi,
		 ret, (unsigned long) ret);
	}
	return (ret);
}

void apps_ssl_info_callback(const SSL *s, int where, int ret)
{
	char string[2048];
	char sbuf[2048];
	const char *str;
	int w;

	bzero(string, sizeof(string));

	w=where& ~SSL_ST_MASK;
	if (w & SSL_ST_CONNECT) strcat(string,"SSL_connect ");
	else if (w & SSL_ST_ACCEPT) strcat(string,"SSL_accept ");
	else strcat(string,"undefined ");

	strcat(string,"statestr ");
	//strcat(string,SSL_state_string_long(s));

	if (where & SSL_CB_LOOP)
	{
		sprintf(sbuf, " %s", SSL_state_string_long(s));
		strcat( string, sbuf);
	}
	else if (where & SSL_CB_ALERT)
	{
		str=(where & SSL_CB_READ)?"read":"write";
		sprintf(sbuf," SSL3 alert %s:%s:%s",
			str,
			SSL_alert_type_string_long(ret),
			SSL_alert_desc_string_long(ret));
		strcat( string, sbuf);
	}
	else if (where & SSL_CB_EXIT)
	{
		if (ret == 0) {
			sprintf(sbuf," :failed in %s",
				SSL_state_string_long(s));
			strcat( string, sbuf);
		}
		else if (ret < 0)
		{
			sprintf(sbuf," error in %s",
				SSL_state_string_long(s));
			strcat( string, sbuf);
		}
	}
	log_message(STREAM_MAIN, MSG_WARN, "%s\n", string);

}
#endif

#endif
/* </Code> */
