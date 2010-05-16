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
*  This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
*  WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
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
*  REV INFO: ver 1.0 05/08/10
*
\******************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>


#define SER2SOCK_VERSION "V1.0"
#define TRUE 1
#define FALSE 0
typedef int BOOL;
#define MAXCONNECTIONS 10
#define MAX_FIFO_BUFFERS 30

/* <Types and Constants> */
const char * fd_type_strings[] = {"","LISTEN","CLIENT","SERIAL"};

enum FD_TYPES {
  LISTEN_SOCKET = 1,
  CLIENT_SOCKET,
  SERIAL
} fd_types;
/* </Types and Constantes> */

/* <Structures> */
 
typedef struct
{
  int size,in,out,avail;
  void **table;
} fifo;

typedef struct  {
  /* flags */
  int inuse;
  int fd_type;
  
  /* the fd */
  int fd;

  /* persistent settings */
  struct termios oldtio;
  
  /* the buffer */
  fifo send_buffer;
} FDs;

/* </Structures> */

/* <Prototypes> */
int init_system();
int init_listen_socket_fd();
int init_serial_fd(char *path);
void set_non_blocking(int fd);
void listen_loop();
void error(char *msg,...);
int kbhit();
int add_fd(int fd,int fd_type);
int msleep(unsigned long milisec);
void show_help();
void print_serial_fd_status(int fd);
// fifo buffer stuff
void  fifo_init(fifo *f, int size);
void  fifo_destroy(fifo *f);
int   fifo_empty(fifo *f);
int   fifo_add(fifo *f,void *next);
void* fifo_get(fifo *f);
void fifo_clear(fifo *f);
void add_to_all_socket_fds(char * message);
void add_to_serial_fd(char * message);
/* </Prototypes> */


/* <Globals> */
// soon to be params or config values
char * serial_device_name=0;
int listen_port = 10000;
int socket_timeout = 10;
int listen_backlog = 10;
FDs my_fds[MAXCONNECTIONS];
// our fdsets
fd_set read_set, write_set;
// our listen socket */
int listen_sock_fd=-1;
struct sockaddr_in serv_addr;
struct sockaddr_in peer_addr;
// fifo buffer 
fifo data_buffer;
/* </Globals> */

/* <Code> */

/* 
 show our error message and die 
 todo: add params.
*/
void error(char *msg,...)
{
    perror(msg);
    exit(1);
}

/* 
 nanosecond seleep 
*/
int __nsleep(const struct timespec *req, struct timespec *rem)
{
    struct timespec temp_rem;
    if(nanosleep(req,rem)==-1)
        __nsleep(rem,&temp_rem);
    else
        return 1;
}

/* 
 sleep for N miliseconds 
*/
int msleep(unsigned long milisec)
{
    struct timespec req={0},rem={0};
    time_t sec=(int)(milisec/1000);
    milisec=milisec-(sec*1000);
    req.tv_sec=sec;
    req.tv_nsec=milisec*1000000L;
    __nsleep(&req,&rem);
    return 1;
}

/* 
 show help info 
*/
void show_help(const char *appName)
{
    fprintf(stderr,"Usage: %s -p <socket listen port> -s <serial port dev>\n\n"
            "  -h, -help                 display this help and exit\n"
            "  -v, -version              display version\n"
            "  -p port                   socket port to listen on\n"
            "  -s <serial device>        serial device ex /dev/ttyUSB0\n"
            "\n", appName);
}

/*
 Initialize any structures etc.
*/
int init_system() {
  int x;
  for(x=0;x<MAXCONNECTIONS;x++) {
    my_fds[x].inuse=FALSE;
    my_fds[x].fd=-1;
    my_fds[x].fd_type=LISTEN_SOCKET;
    fifo_init(&my_fds[x].send_buffer,MAX_FIFO_BUFFERS);
  }
    
  /* clear our read and write sets */
  FD_ZERO(&read_set);
  FD_ZERO(&write_set);
}

/*
 clear all memeor used before we exit
*/
int free_system() {
  int x;
  for(x=0;x<MAXCONNECTIONS;x++) {
    cleanup_fd(x);
    fifo_destroy(&my_fds[x].send_buffer);
  }  
}

/*
 Initialize our listening socket and related api's
*/
int init_listen_socket_fd() {
    BOOL bOptionTrue=TRUE;
    int res;
    
    
    /* create a listening socket fd */
    listen_sock_fd = socket(AF_INET, SOCK_STREAM,0);
    if (listen_sock_fd < 0) 
       error("ERROR creating our listening socket");

    /* clear our socket address structure */
    bzero((char *) &serv_addr, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    
    serv_addr.sin_port = htons(listen_port);
     
    setsockopt(listen_sock_fd,SOL_SOCKET ,SO_SNDTIMEO,(char *)&socket_timeout,sizeof(socket_timeout));
    setsockopt(listen_sock_fd,SOL_SOCKET ,SO_RCVTIMEO,(char *)&socket_timeout,sizeof(socket_timeout));
    setsockopt(listen_sock_fd, SOL_SOCKET, SO_REUSEADDR, (char *)&bOptionTrue, sizeof(bOptionTrue));

 
    if (bind(listen_sock_fd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0) 
              error("ERROR binding to server port");
    
     listen(listen_sock_fd,listen_backlog);
    
     set_non_blocking(listen_sock_fd);
  
     add_fd(listen_sock_fd,LISTEN_SOCKET);
   
     printf("Listening socket created on port %i\n",listen_port); 
     return TRUE;
}

/*
  Init serial port and add fd to our list of sockets
*/
int init_serial_fd(char * szPortPath) {
  struct termios newtio;
  int id;
  long BAUD;                      // derived baud rate from command line
  long DATABITS;
  long STOPBITS;
  long PARITYON;
  long PARITY;
  int Data_Bits = 8;              // Number of data bits
  int Stop_Bits = 1;              // Number of stop bits
  int Parity = 0;                 // Parity as follows:

  int fd = open(szPortPath, O_RDWR | O_NOCTTY | O_NONBLOCK);

  if(fd<0) 
     error("cant open com port at %s\n",szPortPath);

  /* add it and get our strucutre */
  id = add_fd(fd,SERIAL);
  
  BAUD=B9600;

  tcgetattr(fd,&my_fds[id].oldtio); // save current port settings

  newtio.c_cflag = BAUD | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VMIN]=1;
  newtio.c_cc[VTIME]=0;
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd,TCSANOW,&newtio);
  print_serial_fd_status(fd);

  return 1;
}

/*
 prints out the specific serial fd terminal flags  
*/
void print_serial_fd_status(int fd) {
        int status;
        unsigned int arg;
        status = ioctl(fd, TIOCMGET, &arg);
        fprintf(stderr,"Serial Status ");
        if(arg & TIOCM_RTS) fprintf(stderr,"RTS ");
        if(arg & TIOCM_CTS) fprintf(stderr,"CTS ");
        if(arg & TIOCM_DSR) fprintf(stderr,"DSR ");
        if(arg & TIOCM_CAR) fprintf(stderr,"DCD ");
        if(arg & TIOCM_DTR) fprintf(stderr,"DTR ");
        if(arg & TIOCM_RNG) fprintf(stderr,"RI ");
        fprintf(stderr,"\r\n");
}

/* 
 Makes a fd non blocking after opening 
*/
void set_non_blocking(int fd) {
  int nonb = 0;
  int res = 1;
  nonb |= O_NONBLOCK;
  if (ioctl(fd, FIONBIO, &res) < 0)
    error("ERROR setting FIONBIO failed\n");
}

/*
 Add a fd to our array so we can process it in our loop
*/
int add_fd(int fd,int fd_type) {
  int x;
  int results=-1;
  for(x=0;x<MAXCONNECTIONS;x++) {
      if(my_fds[x].inuse==FALSE) {
	fprintf(stderr,"adding %s fd at %i\n",fd_type_strings[fd_type],x);
	my_fds[x].inuse=TRUE;
	my_fds[x].fd_type=fd_type;
	my_fds[x].fd=fd;
	results=x;
	break;
      }
  }
  
  return results;
}

/*
 Cleanup an entry in the fd array and do any fd_type specific cleanup 
*/
int cleanup_fd(int n) {

  /* dont do anything unless its in was active */
  if(my_fds[n].inuse) {

    /* if this is a terminal or serial fd then restore its settings */
    if(my_fds[n].fd_type=SERIAL) {
 	tcsetattr(my_fds[n].fd,TCSANOW,&my_fds[n].oldtio);
    }

    /* close the fd */
    close(my_fds[n].fd);

    /* clear any data we have saved */
    fifo_clear(&my_fds[n].send_buffer);

    /* mark the element as free for reuse */
    my_fds[n].inuse=FALSE;

  }
}

/*
  this loop processes all of our fds'
*/
void listen_loop() {
     int clilen;
     int newsockfd;
     int added_id;
     char *tempbuffer;
     char buffer[1024];
     int n,fd_count,received;
     fd_set read_fdset,write_fdset;
     clilen = sizeof(struct sockaddr_in);     
     struct timeval wait;


	  
     fprintf(stderr,"Start wait loop\n");
     while(!kbhit()) {
        fd_count=0;
	/* add all sockets to our fdset */
	FD_ZERO(&read_fdset);
	FD_ZERO(&write_fdset);
	for(n=0;n<MAXCONNECTIONS;n++) {
	    if(my_fds[n].inuse==TRUE) {
	      FD_SET(my_fds[n].fd,&read_fdset);
  	      FD_SET(my_fds[n].fd,&write_fdset);
	      fd_count++;
	    }	    
	}

	/* lets not block our select and bail after 20us */
        wait.tv_sec = 0;
        wait.tv_usec = 20;

	/* see if any sockets need service */
	n=select(FD_SETSIZE,&read_fdset,&write_fdset,0,&wait);
	if(n==-1) {
	    fprintf(stderr,"socket error\n");
        } else {
	    /* check every socket to find the one that needs service */
	    for(n=0;n<MAXCONNECTIONS;n++) {
	        if(my_fds[n].inuse==TRUE) {
		    
		    /* check read fd */
		    if(FD_ISSET(my_fds[n].fd,&read_fdset)) {
			/* if this is a listening socket then we accept on it and get a new client socket */
			if(my_fds[n].fd_type==LISTEN_SOCKET) { 
				newsockfd = accept(listen_sock_fd,(struct sockaddr *) &peer_addr,&clilen);
				if(newsockfd!=-1) {
				    added_id=add_fd(newsockfd,CLIENT_SOCKET);
				    if(added_id>=0) {
					fprintf(stderr,"socket connected\n");
					/* adding anything to the fifo must be malloced */
					tempbuffer=strdup("!SER2SOCK Connected\r\n");
					fifo_add(&my_fds[added_id].send_buffer,tempbuffer);
				    }
				}
			} else {
			  if(my_fds[n].fd_type==SERIAL) {
			    errno=0;
			    while ((received = read(my_fds[n].fd, buffer, sizeof(buffer))) > 0)
		   	    {
			        if(received>0) {
			          buffer[received] = 0;
				  add_to_all_socket_fds(buffer);
			          fprintf(stderr,"%s",buffer);
			        } 
			    }

			    if(received<0) {
				/*
					todo: add error handeling  
				*/
			    }
			  } else {
			    errno=0;
			    received = recv(my_fds[n].fd, buffer, sizeof(buffer), 0);
			    buffer[received] = 0;
			    if(received==0) {
			      fprintf(stderr,"closing socket errno: %i\n", errno);
			      cleanup_fd(n);
			    } else {
			      add_to_serial_fd(buffer);
			      fprintf(stderr,"Message from socket: %i '%s'\n", received, buffer);
			    }
			  }
			}			  
		    } /* end FD_ISSET() 
		    
		    /* check write fd */
		    if(FD_ISSET(my_fds[n].fd,&write_fdset) ) {
			if(!fifo_empty(&my_fds[n].send_buffer)) {
			    tempbuffer=(char *)fifo_get(&my_fds[n].send_buffer);
			    if(my_fds[n].fd_type==CLIENT_SOCKET)
			       send(my_fds[n].fd,tempbuffer,strlen(tempbuffer),0);
			    if(my_fds[n].fd_type==SERIAL)
			       write(my_fds[n].fd,tempbuffer,strlen(tempbuffer));
 
			    free(tempbuffer);
			} else {
			   //printf("nothing to send\n");
			}
		    }		   
		}
	    }	  
	}
       
	msleep(20);
     }

     fprintf(stderr,"\ncleaning up\n");
     free_system();

     fprintf(stderr,"done.\n");
     exit(1);
}

/* 
 add_to_all_socket_fds 
 adds a buffer to ever connected socket fd ie multiplexes
*/
void add_to_all_socket_fds(char * message) {

	char * tempbuffer;
	int n;

	/* 
            Adding anything to the fifo must be allocated so it can be free'd later
           Not very efficient but we have plenty of mem with as few connections as we
           will use. If we needed many more I would need to refactor how this works
         */
	for(n=0;n<MAXCONNECTIONS;n++) {
		if(my_fds[n].fd_type==CLIENT_SOCKET) {
			/* caller of fifo_get must free this */
			tempbuffer=strdup(message);
			fifo_add(&my_fds[n].send_buffer,tempbuffer);
		}	
	}

}

/*
 adds data to the serial fifo buffer should be at 0 ever time
*/
void add_to_serial_fd(char *message) {
	char * tempbuffer;
	tempbuffer=strdup(message);
	fifo_add(&my_fds[0].send_buffer,tempbuffer);
}

/*
 parse_args
*/
int parse_args(int argc, char * argv[]) {

	char **loc_argv=argv;
	int loc_argc = argc;

	while (loc_argc > 1) {	
		if(loc_argv[1][0] == '-')
		switch (loc_argv[1][1])
		{
			case 'h':
				show_help(argv[0]);
				exit(1);
				break;
			case 'p':
				listen_port=atoi(&loc_argv[1][3]);
				break;
			case 's':
				serial_device_name=&loc_argv[1][3];
				break;
			default:
				show_help(argv[0]);
				error("ERROR Wrong argument: %s\n", loc_argv[1]);
				exit(1);
		}

		++loc_argv;
		--loc_argc;
	}

	if(serial_device_name==0) {
		show_help(argv[0]);
		error("ERROR Missing serial device name\n");
	}

	return TRUE;
}

/*
 main()
*/
int main(int argc, char *argv[])
{
     /* startup banner and args check */
     fprintf(stderr,"Serial 2 Socket Relay version %s\n", SER2SOCK_VERSION);
     if (argc < 2) {
        show_help(argv[0]); 
	error("ERROR insufficient arguments\n");
	exit(1);
     }

     /* parse args and set global vars as needed */ 
     parse_args(argc,argv);
    
     /* initialize the system */
     init_system();

     /* initialize the serial port */
     init_serial_fd(serial_device_name);

     /* initialize our listening socket */
     if(!init_listen_socket_fd()) {
	error("ERROR initializing listen socket\n");
     }

     /* begin our listen loop */
     listen_loop();
     
     return 0; 
}

/* 
 good old kbhit funciton just back in the day 
*/
int kbhit(void)
{
   struct termios oldt, newt;
   int ch;
   int oldf;


   tcgetattr(STDIN_FILENO, &oldt);
   newt = oldt;
   newt.c_lflag &= ~(ICANON | ECHO);
   tcsetattr(STDIN_FILENO, TCSANOW, &newt);
   oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
   fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

   ch = getchar();

   tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
   fcntl(STDIN_FILENO, F_SETFL, oldf);

   if(ch != EOF)
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
void fifo_init (fifo *f,int size) {
   f->avail=0; f->in=0;f->out=0;
   f->size=size;
   f->table=(void**)malloc(f->size*sizeof(void*));
}

/* 
 fifo empty if queue = 1 else 0 
*/
int fifo_empty(fifo *f) {
   return(f->avail==0);
}

/* 
 free up any memory we allocated 
*/
void fifo_destroy(fifo *f) {
   int i;
   if(!fifo_empty(f)) free(f->table);
   else{
        for(i=f->out;i<f->in;i++){
	    /* free actual block of memory */
            free(f->table[i]);
        }
        free(f->table);
   }
}

/* 
 remove all stored pending data 
*/
void fifo_clear(fifo *f) {
   int done=FALSE;
   void *p;
   while(!fifo_empty(f)) {
     p=fifo_get(f);
     if(p) free(p);
   }
}

/* 
 insert an element 
 this must be already allocated with malloc or strdup
*/
int fifo_add(fifo *f,void *next) {
   if(f->avail==f->size) {
     return(0);
   } else {
       f->table[f->in]=next;
       f->avail++;
       f->in=(f->in+1)%f->size;
       return(1);
   }   
}

/* 
 return next element 
*/
void* fifo_get(fifo *f){
   void* get;
   if (f->avail>0) {
       get=f->table[f->out];
       f->out=(f->out+1)%f->size;
       f->avail--;
       return(get);
   } 
   return 0;
}   
//</Fifo Buffer>

/* </Code> */
