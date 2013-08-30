/******************************************************************************\ 
 *
 *  MODULE: ser2sock.c
 *          Copyright (C) 2010 Nu Tech Software Solutions, Inc.
 *          All rights reserved
 *          Reproduction without permission is prohibited
 *
 *  This file may be used under the terms of the GNU General Public
 *  License versions 2.0 or 3.0 as published by the Free Software
 *  Foundation and appearing in the files LICENSE.GPL2 and LICENSE.GPL3
 *  included in the packaging of this file.
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
 *
 *  FUNCTIONS: Too many to list
 *
 *  COMMENTS: to build from a shell prompt use cc as follows
 *    user@host:~> cc -o ser2sock ser2sock.c
 *
 *
 *  DEVELOPED BY: Sean Mathews
 *                  http://www.nutech.com/
 *
 *	Thanks to Richard Perlman [ad2usb at perlman.com] for his help testing on
 *     bsd and excellent feedback on features. Also a big thanks to everyone
 *     that helped support the AD2USB project get off the ground.
 *
 *  REV INFO: ver 1.0   05/08/10 Initial release
 *		  1.1   05/18/10 Refining, looking for odd echo bug thanks Richard!
 *		  1.2   05/19/10 Adding daemon mode, bind to ip, debug output
 *                      syslog and more
 *		  1.2.4 01/17/11 Fixed a socket bug
 *        1.2.5 04/01/11 Working on issue on BSD systems
 *		  1.3.0 05/16/11 Reestablishing connection with the serial device
 *		  1.3.1 04/27/12 fixed a few compiler issues with some systems
 *		  1.3.2 08/28/13 improvments in syslog formats
 *
 \******************************************************************************/
#define _GNU_SOURCE
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
#ifdef _POSIX_SOURCE
#include <sched.h>
#endif

#define SER2SOCK_VERSION "V1.3.1"
#define TRUE 1
#define FALSE 0
typedef int BOOL;
#define MAXCLIENTCONNECTIONS 10
#define MAXCONNECTIONS MAXCLIENTCONNECTIONS+2
#define MAX_FIFO_BUFFERS 30

#define SERIAL_CONNECTED_MSG	"!SER2SOCK SERIAL_CONNECTED\r\n"
#define SERIAL_DISCONNECTED_MSG	"!SER2SOCK SERIAL_DISCONNECTED\r\n"

/* <Types and Constants> */
const char terminal_init_string[] = "\377\375\042";
const char * fd_type_strings[] =
{ "NA", "LISTEN", "CLIENT", "SERIAL" };
enum FD_TYPES
{
	NA, LISTEN_SOCKET = 1, CLIENT_SOCKET, SERIAL
} fd_types;

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

#define DAEMON_NAME "ser2sock"
#define PID_FILE "/var/run/ser2sock.pid"

/* </Types and Constants> */

/* <Structures> */

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

	/* persistent settings */
	struct termios oldtio;

	/* the buffer */
	fifo send_buffer;
} FDs;

struct add_to_all_socket_fds_state_t
{
	int line_ended;
};

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
void init_add_to_all_socket_fds_state(
		struct add_to_all_socket_fds_state_t *state);
void add_to_all_socket_fds(struct add_to_all_socket_fds_state_t *state,
		char * message);
void add_to_serial_fd(char * message);
int cleanup_fd(int n);
void set_non_blocking(int fd);
void print_serial_fd_status(int fd);
int get_baud(char *szbaud);
void listen_loop();
void log_message(char *msg, ...);
void vlog_message(char *msg, va_list arg);
void error(char *msg, ...);
int kbhit();
int add_fd(int fd, int fd_type);
int __nsleep(const struct timespec *req, struct timespec *rem);
int msleep(unsigned long milisec);
void show_help();
void signal_handler(int sig);
// fifo buffer stuff
void fifo_init(fifo *f, int size);
void fifo_destroy(fifo *f);
int fifo_empty(fifo *f);
int fifo_add(fifo *f, void *next);
void* fifo_get(fifo *f);
void fifo_clear(fifo *f);
static void writepid(void);
/* </Prototypes> */

/* <Globals> */
// soon to be params or config values
char * serial_device_name = 0;
int listen_port = 10000;
int socket_timeout = 10;
int listen_backlog = 10;
FDs my_fds[MAXCONNECTIONS];
// our listen socket */
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
	vlog_message(msg, arg);
	va_end(arg);

	log_message(" :");
	log_message(szError);
	log_message("\n");
	log_message("exiting\n");
	exit(EXIT_FAILURE);
}

/*
 log a message to console or syslog
 */
void log_message(char *msg, ...)
{
	if (msg)
	{
		va_list arg;
		va_start(arg, msg);
		vlog_message(msg, arg);
		va_end(arg);
	}
}

void vlog_message(char *msg, va_list arg)
{
	static char message[2048];
	static int last = 0;
	int x,y,z=0;
	BOOL done=FALSE;
	last += vsnprintf(&message[last], sizeof(message) - last, msg, arg);

	/* check for overflow error */
	if (last >= sizeof(message))
		last = 0;

	if (last) {
		/* keep trying till we exause all \n's */
		while(!done)
		{
			/* look for an eol char */
			for (x = 0; x < last ; x++) {

				if(message[x] == '\n' || message[x] == '\r') {

					message[x]=0;
					if(x)
					if (option_daemonize)
					{
						syslog(LOG_INFO, "%s", message);
					}
					else
					{
						fprintf(stderr, "%s\n", message);
					}

					/* move the rest to the start and clean out any non printable chars */ 
					z = 0;
					for(y = x+1; y < last ; y++) {
						if((message[y]>0x1f && message[y]<0x7f) || message[y]=='\n') {
							message[z++]=message[y];
						}
					}

					/* set our next fill position */
					last = z;

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
				"  -s <serial device>        serial device ex /dev/ttyUSB0\n"
				"options\n"
				"  -i IP                     bind to a specific ip address default is ALL\n"
				"  -b baudrate               set buad rate default to 9600\n"
				"  -d                        daemonize\n"
				"  -t                        send terminal init string\n"
				"  -g                        debug level 0-3\n"
				"  -c                        keep incoming connections when a serial device is disconnected\n"
				"  -w milliseconds           delay between attempts to open a serial device (5000)\n"
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
		fifo_init(&my_fds[x].send_buffer, MAX_FIFO_BUFFERS);
	}

	/* Setup signal handling if we are to daemonize */
	if (option_daemonize)
	{
		signal(SIGHUP, signal_handler);
		signal(SIGTERM, signal_handler);
		signal(SIGINT, signal_handler);
		signal(SIGQUIT, signal_handler);
		setlogmask(LOG_UPTO(LOG_DEBUG));
	}
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
	return TRUE;
}

/*
 Initialize our listening socket and related api's
 */
int init_listen_socket_fd()
{
	BOOL bOptionTrue = TRUE;
	int results;

	/* create a listening socket fd */
	listen_sock_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (listen_sock_fd < 0)
	{
		log_message("ERROR creating our listening socket");
		return FALSE;
	}

	/* clear our socket address structure */
	bzero((char *) &serv_addr, sizeof(serv_addr));

	if (option_bind_ip != NULL)
	{
		results = inet_pton(AF_INET, option_bind_ip, &serv_addr.sin_addr);
		if (results != 1)
		{
			log_message("ERROR unable to bind to provided IP %s",
					option_bind_ip);
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

	if (bind(listen_sock_fd, (struct sockaddr *) &serv_addr, sizeof(serv_addr))
			< 0)
	{
		log_message("ERROR binding to server port");
		return FALSE;
	}

	listen(listen_sock_fd, listen_backlog);

	set_non_blocking(listen_sock_fd);

	add_fd(listen_sock_fd, LISTEN_SOCKET);

	log_message("Listening socket created on port %i\n", listen_port);

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
		log_message("ERROR. Can't open com port at %s\n", szPortPath);
		return fd;
	}

	log_message("opened com port at %s\n", szPortPath);

	/* add it and get our structure */
	id = add_fd(fd, SERIAL);

	if (id < 0)
	{
		log_message("ERROR. Can't serial add fd\n");
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
		log_message("c_cflags old:%08x  new:%08x\n", my_fds[id].oldtio.c_cflag,
				newtio.c_cflag);

	/* change our c_lflag settings enable raw input mode */
	newtio.c_lflag &= ~(ICANON | ECHO | ISIG);
	if (option_debug_level > 2)
		log_message("c_lflags old:%08x  new:%08x\n", my_fds[id].oldtio.c_lflag,
				newtio.c_lflag);

	/* change our c_iflag settings (turn off all flags) */
	newtio.c_iflag = 0;
	if (option_debug_level > 2)
		log_message("c_iflags old:%08x  new:%08x\n", my_fds[id].oldtio.c_iflag,
				newtio.c_iflag);

	/* change our c_oflag settings (turn off all flags) */
	newtio.c_oflag = 0;
	if (option_debug_level > 2)
		log_message("c_oflags old:%08x  new:%08x\n", my_fds[id].oldtio.c_oflag,
				newtio.c_oflag);

	/* dump bytes out of old c_cc */
	if (option_debug_level > 2)
	{
		log_message("c_cc old: ");
		for (x = 0; x < sizeof(my_fds[id].oldtio.c_cc); x++)
		{
			log_message("%02x:", my_fds[id].oldtio.c_cc[x]);
		}
		log_message("\n");
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
# ifdef VSWTCH
	//newio.c_cc[VSWTCH] = 0;
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
		log_message("c_cc new: ");
		for (x = 0; x < sizeof(newtio.c_cc); x++)
		{
			log_message("%02x:", newtio.c_cc[x]);
		}
		log_message("\n");
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
	log_message("Serial Status (%i) ",status);
	if (arg & TIOCM_RTS)
		log_message("RTS ");
	if (arg & TIOCM_CTS)
		log_message("CTS ");
	if (arg & TIOCM_DSR)
		log_message("DSR ");
	if (arg & TIOCM_CAR)
		log_message("DCD ");
	if (arg & TIOCM_DTR)
		log_message("DTR ");
	if (arg & TIOCM_RNG)
		log_message("RI ");
	log_message("\r\n");
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
	log_message("setting speed %s\n", s->name);
	return s->flag;
}

/* 
 Makes a fd non blocking after opening
 */
void set_non_blocking(int fd)
{
	int nonb = 0;
	int res = 1;
	nonb |= O_NONBLOCK;
	if (ioctl(fd, FIONBIO, &res) < 0)
		error("ERROR setting FIONBIO failed\n");
}

/*
 Add a fd to our array so we can process it in our loop
 */
int add_fd(int fd, int fd_type)
{
	int s = 0;
	int x;
	int results = -1;
	// Reserve two first sockets for serial & listen
	if (fd_type == CLIENT_SOCKET)
		s = 2;
	for (x = s; x < MAXCONNECTIONS; x++)
	{
		if (my_fds[x].inuse == FALSE)
		{
			if (option_debug_level > 2)
				log_message("adding %s fd at %i\n", fd_type_strings[fd_type], x);
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
		}

		/* close the fd */
		close(my_fds[n].fd);

		/* clear any data we have saved */
		fifo_clear(&my_fds[n].send_buffer);

		/* mark the element as free for reuse */
		my_fds[n].inuse = FALSE;

		/* set the type to null */
		my_fds[n].fd_type = NA;

	}
	return TRUE;
}

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
		tv_start.tv_sec = 0; \
		tv_start.tv_usec = 0; \
		serial_connected = 0; \
		cleanup_fd(n); \
		add_to_all_socket_fds(&aas_state, "\r\n"); \
		add_to_all_socket_fds(&aas_state, SERIAL_DISCONNECTED_MSG); \
		msleep(100); \

/*
 this loop processes all of our fds'
 */
void listen_loop()
{
	unsigned int clilen;
	int newsockfd;
	int added_id, tmp;
	byte_t *tempbuffer;
	byte_t buffer[1024];
	int n, x, fd_count, received;
	char is_serviced;
	fd_set read_fdset, write_fdset, except_fdset;
	clilen = sizeof(struct sockaddr_in);
	struct timeval wait;
	int serial_connected = 0;
	struct timeval tv_start, tv_last_serial_check;
	struct add_to_all_socket_fds_state_t aas_state;
#ifdef _POSIX_SOURCE
	// Set high thread priority
	struct sched_param param;

	log_message("Set high thread priority\n");
	memset(&param,0,sizeof(param));
	param.__sched_priority = sched_get_priority_min(SCHED_RR);
	sched_setscheduler(0,SCHED_RR,&param);
#endif


	init_add_to_all_socket_fds_state(&aas_state);
	tv_start.tv_sec = 0;
	tv_start.tv_usec = 0;
	tv_last_serial_check.tv_sec = 0;
	tv_last_serial_check.tv_usec = 0;

	log_message("Start wait loop\n");
	while (!kbhit())
	{
		if (!serial_connected)
		{
			if ((tv_start.tv_sec == 0) || (get_time_difference(&tv_start)
					>= option_open_serial_delay))
			{
				gettimeofday(&tv_start, NULL);
				if (init_serial_fd(serial_device_name) > 0)
				{
					serial_connected = 1;
					tv_last_serial_check.tv_sec = 0;
					tv_last_serial_check.tv_usec = 0;
					init_add_to_all_socket_fds_state(&aas_state);
					add_to_all_socket_fds(&aas_state, SERIAL_CONNECTED_MSG);
				}
				else
					msleep(10);
			}
			else
				msleep(10);

		} else 
		{
			if ((tv_last_serial_check.tv_sec == 0) || (get_time_difference(
					&tv_last_serial_check) >= 100))
			{
				errno = 0;
				gettimeofday(&tv_last_serial_check, NULL);
				for (n = 0; n < MAXCONNECTIONS; n++)
				{
					if (my_fds[n].inuse == TRUE)
					{
						if (my_fds[n].fd_type == SERIAL)
						{
#ifdef USE_TIOCMGET
							if (ioctl(my_fds[n].fd, TIOCMGET, &tmp) < 0)
							{
								log_message(
										"Serial disconnected on check. errno: %i\n",
										errno);
								clear_serial(n);
							}
							break;
#endif
						}
					}
				}
			}
		}

		fd_count = 0;
		/* add all sockets to our fdset */
		FD_ZERO(&read_fdset);
		FD_ZERO(&write_fdset);
		FD_ZERO(&except_fdset);
		for (n = 0; n < MAXCONNECTIONS; n++)
		{
			if (my_fds[n].inuse == TRUE)
			{
				FD_SET(my_fds[n].fd,&read_fdset);
				FD_SET(my_fds[n].fd,&write_fdset);
				FD_SET(my_fds[n].fd,&except_fdset);
				fd_count++;
			}
		}
		is_serviced = 0;

		/* lets not block our select and bail after 20us */
		wait.tv_sec = 0;
		wait.tv_usec = 20;

		/* see if any sockets need service */
		n = select(FD_SETSIZE, &read_fdset, &write_fdset, &except_fdset, &wait);
		if (n == -1)
		{
			log_message("socket error\n");
		}
		else
		{
			for (n = 0; n < MAXCONNECTIONS; n++)
			{
				if (my_fds[n].inuse == TRUE)
				{
					if (FD_ISSET(my_fds[n].fd,&except_fdset))
						if (my_fds[n].fd_type == CLIENT_SOCKET)
						{
							log_message("closing socket on exception\n");
							cleanup_fd(n);
						}
				}
			}

			/* check every socket to find the one that needs read */
			for (n = 0; n < MAXCONNECTIONS; n++)
			{
				if (my_fds[n].inuse == TRUE)
				{

					/* check read fd */
					if (FD_ISSET(my_fds[n].fd,&read_fdset))
					{
						/* if this is a listening socket then we accept on it and get a new client socket */
						if (my_fds[n].fd_type == LISTEN_SOCKET)
						{
							newsockfd = accept(listen_sock_fd,
									(struct sockaddr *) &peer_addr, &clilen);
							if (newsockfd != -1)
							{
								if (serial_connected || option_keep_connection)
								{
									added_id = add_fd(newsockfd, CLIENT_SOCKET);
									if (added_id >= 0)
									{
										log_message("socket connected\n");
										/* adding anything to the fifo must be pre allocated */
										if (option_send_terminal_init)
										{
											tempbuffer = strdup("!");
											fifo_add(
													&my_fds[added_id].send_buffer,
													tempbuffer);
											tempbuffer = strdup(
													terminal_init_string);
											fifo_add(
													&my_fds[added_id].send_buffer,
													tempbuffer);
											tempbuffer = strdup("\r\n");
											fifo_add(
													&my_fds[added_id].send_buffer,
													tempbuffer);
										}

										tempbuffer = strdup(
												"!SER2SOCK Connected\r\n");
										fifo_add(&my_fds[added_id].send_buffer,
												tempbuffer);
										if (serial_connected)
											tempbuffer = strdup(
													SERIAL_CONNECTED_MSG);
										else
											tempbuffer = strdup(
													SERIAL_DISCONNECTED_MSG);
										fifo_add(&my_fds[added_id].send_buffer,
												tempbuffer);
										is_serviced = 1;
									}
									else
									{
										close(newsockfd);
										log_message(
												"socket refused because no more space\n");
									}
								}
								else
								{
									close(newsockfd);
									log_message(
											"socket refused because serial is not connected\n");
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
										is_serviced = 1;
										buffer[received] = 0;
										add_to_all_socket_fds(&aas_state,
												buffer);
										if (option_debug_level > 1)
										{
											if (option_debug_level > 2)
											{
												log_message("SERIAL>'");
												for (x = 0; x < received; x++)
												{
													log_message("[%02x]",
															buffer[x]);
												}
												log_message("\n");
											}
											else
											{
												log_message("%s", buffer);
											}
										}
									}
								}

								if (received < 0)
								{
									if (errno != EAGAIN)
									{
										log_message(
												"Serial disconnected on read. errno: %i\n",
												errno);
										clear_serial(n);
									}
								}
							}
							else
							{
								errno = 0;
								received = recv(my_fds[n].fd, buffer,
										sizeof(buffer), 0);
								if (received == 0)
								{
									log_message("closing socket errno: %i\n",
											errno);
									cleanup_fd(n);
								}
								else
								{
									if (received < 0)
									{
										if (errno == EAGAIN || errno == EINTR)
											continue;
										log_message(
												"closing socket errno: %i\n",
												errno);
										cleanup_fd(n);
									}
									else
									{
										is_serviced = 1;
										buffer[received] = 0;
										add_to_serial_fd(buffer);
										if (option_debug_level > 2)
										{
											log_message("SOCKET[%i]>", n);
											for (x = 0; x < strlen(buffer); x++)
											{
												log_message("[%02x]", buffer[x]);
											}
											log_message("\n");
										}
									}
								}
							}
						}
					} /* end FD_ISSET() */
				}
			}
			/* check every socket to find the one that needs write */
			for (n = 0; n < MAXCONNECTIONS; n++)
			{
				if (my_fds[n].inuse == TRUE)
				{
					if (FD_ISSET(my_fds[n].fd,&write_fdset))
					{
						if (!fifo_empty(&my_fds[n].send_buffer))
						{
							tempbuffer = (char *) fifo_get(
									&my_fds[n].send_buffer);
							if (my_fds[n].fd_type == CLIENT_SOCKET)
							{
								is_serviced = 1;
								if (option_debug_level > 2)
									log_message("SOCKET[%i]<", n);
								send(my_fds[n].fd, tempbuffer, strlen(
										tempbuffer), 0);
							}
							else
								if (my_fds[n].fd_type == SERIAL)
								{
									errno = 0;
									is_serviced = 1;
									if (option_debug_level > 2)
										log_message("SERIAL[%i]<", n);
									if (write(my_fds[n].fd, tempbuffer, strlen(
											tempbuffer)) < 0)
									{
										if (errno != EAGAIN)
										{
											log_message(
													"Serial disconnected on write. errno: %i\n",
													errno);
											clear_serial(n);
										}

									}
								}

							if (option_debug_level > 2)
							{
								for (x = 0; x < strlen(tempbuffer); x++)
								{
									log_message("[%02x]", tempbuffer[x]);
								}
								log_message("\n");
							}
							free(tempbuffer);
						}
						else
						{
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
			}
		}

		if (!is_serviced)
			msleep(20);
	}

	log_message("\ncleaning up\n");
	free_system();

	log_message("done.\n");
}

void init_add_to_all_socket_fds_state(
		struct add_to_all_socket_fds_state_t *state)
{
	state->line_ended = 0;
}
/* 
 add_to_all_socket_fds
 adds a buffer to ever connected socket fd ie multiplexes
 */
void add_to_all_socket_fds(struct add_to_all_socket_fds_state_t *state,
		char * message)
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
				if (state->line_ended || !my_fds[n].new)
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
	state->line_ended = 0;
	if (message)
	{
		location = message + strlen(message) - 1;
		if (location)
		{
			if ((*location) == '\n')
				state->line_ended = 1;
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

				default:
					show_help(argv[0]);
					error("ERROR Wrong argument: %s\n", loc_argv[1]);
					exit(EXIT_FAILURE);
			}

		++loc_argv;
		--loc_argc;
	}

	if (serial_device_name == 0)
	{
		show_help(argv[0]);
		log_message("Error missing serial device name exiting\n");
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
			log_message("Error writing to pid file %s", PID_FILE);
		close(fd);
		return;
	}
	else
		log_message("Error opening pid file %s", PID_FILE);
}

/*
 main()
 */
int main(int argc, char *argv[])
{
	/* parse args and set global vars as needed */
	parse_args(argc, argv);

	if (option_daemonize)
	{
		openlog(DAEMON_NAME, LOG_CONS | LOG_NDELAY | LOG_PERROR | LOG_PID,
				LOG_USER);
	}


	/* startup banner and args check */
	log_message("Serial 2 Socket Relay version %s starting\n", SER2SOCK_VERSION);
	if (argc < 2)
	{
		show_help(argv[0]);
		log_message("ERROR insufficient arguments\n");
		exit(EXIT_FAILURE);
	}

	/* initialize the system */
	init_system();

	/* initialize our listening socket */
	if (!init_listen_socket_fd())
	{
		error("ERROR initializing listen socket\n");
	}

	/* Our process ID and Session ID */
	pid_t pid, sid;

	if (option_daemonize)
	{
		syslog(LOG_INFO, "daemonizing the process");

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
			openlog(DAEMON_NAME, LOG_NDELAY | LOG_PID, LOG_USER);
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
			syslog(LOG_WARNING, "Received SIGHUP signal.");
			log_message("cleaning up\n");
			free_system();
			log_message("done.\n");
			exit(EXIT_SUCCESS);
			break;
		case SIGTERM:
			syslog(LOG_WARNING, "Received SIGTERM signal.");
			log_message("cleaning up\n");
			free_system();
			log_message("done.\n");
			exit(EXIT_SUCCESS);
			break;
		default:
			syslog(LOG_WARNING, "Unhandled signal (%d) %s", sig, strsignal(sig));
			break;
	}
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
		log_message("clearning fifo: ");
	while (!fifo_empty(f))
	{
		if (option_debug_level > 2)
			log_message("*");
		p = fifo_get(f);
		if (p)
			free(p);
	}
	if (option_debug_level > 2)
		log_message(" done.\n");
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

/* </Code> */
