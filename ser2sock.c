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
*  COMMENTS:
*
*
*
*  DEVELOPED BY: Sean Mathews
*
*  REV INFO: ver 1.0 05/08/10
*
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

/* <Structures> */
 
typedef struct
{
  int size,in,out,avail;
  void **table;
} fifo;

typedef struct  {
  int inuse;
  int listening;
  int socket;
  fifo send_buffer;
} Socket;

/* </Structures> */

/* <Prototypes> */
int init_listen_socket();
void set_non_blocking(int socket);
void listen_loop();
void show_help();
int init_system();
void error(char *msg);
int kbhit();
int add_socket(int socket,int listening);
int msleep(unsigned long milisec);

// fifo buffer stuff
void  fifo_init(fifo *f, int size);
void  fifo_destroy(fifo *f);
int   fifo_empty(fifo *f);
int   fifo_add(fifo *f,void *next);
void* fifo_get(fifo *f);
void fifo_clear(fifo *f);
/* </Prototypes> */


/* <Globals> */
// soon to be params or config values
int listen_port = 10001;
int socket_timeout = 10;
int listen_backlog = 10;
Socket my_sockets[MAXCONNECTIONS];
// our fdsets
fd_set read_set, write_set;
// our listen socket */
int listen_sock_fd=-1;
struct sockaddr_in serv_addr;
struct sockaddr_in peer_addr;
// fifo buffer 
fifo data_buffer;

/* </Globals> */


/* 
 show our error message and die 
*/
void error(char *msg)
{
    perror(msg);
    exit(1);
}

/* nanosecond seleep */
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
void showHelp(const char *appName)
{
    fprintf(stderr, "Serial 2 Socket Relay version %s\n", SER2SOCK_VERSION);
    if (error)
        fprintf(stderr, "%s: %s\n", appName, error);

    fprintf(stderr, "Usage: %s [options] <serial port dev>\n\n"
            "  -h, -help                 display this help and exit\n"
            "  -v, -version              display version\n"
            "  -p port                   socket port to listen on\n"
            "\n", appName);
}

/*
 Initialize any structures etc.
*/
int init_system() {
  int x;
  for(x=0;x<MAXCONNECTIONS;x++) {
    my_sockets[x].inuse=FALSE;
    my_sockets[x].socket=-1;
    my_sockets[x].listening=FALSE;
    fifo_init(&my_sockets[x].send_buffer,MAX_FIFO_BUFFERS);
  }
}

/*
 clear all memeor used before we exit
*/
int free_system() {
  int x;
  for(x=0;x<MAXCONNECTIONS;x++) {
    cleanup_socket(x);
    fifo_destroy(&my_sockets[x].send_buffer);
  }  
}

/*
 Initialize our listening socket and related api's
*/
int init_listen_socket() {
    BOOL bOptionTrue=TRUE;
    int res;
    
    /* clear our read and write sets */
    FD_ZERO(&read_set);
    FD_ZERO(&write_set);
    
    /* create a listening socket */
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
  
     add_socket(listen_sock_fd,TRUE);
    
     return TRUE;
}


/* 
 Makes a socket non blocking 
*/
void set_non_blocking(int socket) {
  int nonb = 0;
  int res = 1;
  nonb |= O_NONBLOCK;
  if (ioctl(socket, FIONBIO, &res) < 0)
    error("ERROR setting FIONBIO failed\n");
}

/*
 Add a socket to our array so we can process it in our loop
*/
int add_socket(int socket,int listening) {
  int x;
  int results=-1;
  for(x=0;x<MAXCONNECTIONS;x++) {
      if(my_sockets[x].inuse==FALSE) {
	printf("adding socket at %i\n",x);
	my_sockets[x].inuse=TRUE;
	my_sockets[x].socket=socket;
	my_sockets[x].listening=listening;
	results=x;
	break;
      }
  }
  
  return results;
}

int cleanup_socket(int n) {
  if(my_sockets[n].inuse) {
  /* close the socket */
    close(my_sockets[n].socket);
  /* clear any data we have saved */
    fifo_clear(&my_sockets[n].send_buffer);
  /* mark the element as free for reuse */
    my_sockets[n].inuse=FALSE;
  }
}

/*
  this loop processes all of our sockets
*/
void listen_loop() {
     int clilen;
     int newsockfd;
     int added_id;
     char *tempbuffer;
     char buffer[256];
     int n,fd_count,received;
     fd_set read_fdset,write_fdset;
     clilen = sizeof(struct sockaddr_in);     
     struct timeval wait;


	  
     printf("Start wait loop\n");
     while(!kbhit()) {
        fd_count=0;
	/* add all sockets to our fdset */
	FD_ZERO(&read_fdset);
	FD_ZERO(&write_fdset);
	for(n=0;n<MAXCONNECTIONS;n++) {
	    if(my_sockets[n].inuse==TRUE) {
	      FD_SET(my_sockets[n].socket,&read_fdset);
	      FD_SET(my_sockets[n].socket,&write_fdset);
	      fd_count++;
	    }	    
	}

	/* lets not block our select and bail after 20us */
        wait.tv_sec = 0;
        wait.tv_usec = 20;

	/* see if any sockets need service */
	n=select(FD_SETSIZE,&read_fdset,&write_fdset,0,&wait);
	if(n==-1) {
	    printf("socket error\n");
        } else {
	    /* check every socket to find the one that needs service */
	    for(n=0;n<MAXCONNECTIONS;n++) {
	        if(my_sockets[n].inuse==TRUE) {
		    
		    /* check read fd */
		    if(FD_ISSET(my_sockets[n].socket,&read_fdset)) {
			/* if this is a listening socket then we accept on it and get a new client socket */
			if(my_sockets[n].listening) { 
				newsockfd = accept(listen_sock_fd,(struct sockaddr *) &peer_addr,&clilen);
				if(newsockfd!=-1) {
				    added_id=add_socket(newsockfd,FALSE);
				    if(added_id>=0) {
					printf("socket connected\n");
					/* adding anything to the fifo must be malloced */
					tempbuffer=strdup("!SER2SOCK Connected\r\n");
					fifo_add(&my_sockets[added_id].send_buffer,tempbuffer);
				    }
				}
			} else {
			  errno=0;
			  received = recv(my_sockets[n].socket, buffer, sizeof(buffer), 0);
			  buffer[received] = 0;
			  printf("Message from socket: %i '%s'\n", received, buffer);
			  if(received==0) {
			    printf("closing socket errno: %i\n", errno);
			    cleanup_socket(n);
			  }
			}			  
		    } else {
			//printf("socket rx/tx or error\n");
		    }
		    
		    /* check write fd */
		    if(FD_ISSET(my_sockets[n].socket,&write_fdset)) {
			if(!fifo_empty(&my_sockets[n].send_buffer)) {
			    tempbuffer=(char *)fifo_get(&my_sockets[n].send_buffer);
			    send(my_sockets[n].socket,tempbuffer,strlen(tempbuffer),0);
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

	
     printf("exiting\n");
     
     
     free_system();

}


/*
 main()
*/
int main(int argc, char *argv[])
{
     
     if (argc < 2) {
         fprintf(stderr,"ERROR, no port provided\n");
         exit(1);
     }
     
     /* initialize the system */
     init_system();
     
     /* initialize our listening socket */
     if(!init_listen_socket()) {
	error("ERROR initializing listen socket\n");
     }

     /* begin our listen loop */
     listen_loop();
     
     return 0; 
}


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
