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
*  A PARTICULAR PURPOSE. Trolltech reserves all rights not expressly
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


//#include <sys/stat.h>
//#include <time.h>
//#include <unistd.h>
//#include <sys/poll.h>

#define SER2SOCK_VERSION "V1.0"
#define TRUE 1
#define FALSE 0
typedef int BOOL;
#define MAXCONNECTIONS 10

/* <Prototypes> */
int init_listen_socket();
void set_non_blocking(int socket);
void listen_loop();
void show_help();
int init_system();
void error(char *msg);
int kbhit();
/* </Prototypes> */


/* <Structures */
typedef struct  {
  int inuse;
  int listening;
  int socket;
} Socket;
/* </Structures> */

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
/* </Globals> */


/* 
 show our error message and die 
*/
void error(char *msg)
{
    perror(msg);
    exit(1);
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
void add_socket(int socket,int listening) {
  int x;
  for(x=-0;x<MAXCONNECTIONS;x++) {
      if(my_sockets[x].inuse==FALSE) {
	my_sockets[x].inuse=TRUE;
	my_sockets[x].socket=socket;
	my_sockets[x].listening=listening;
      }
  }
}


/*
  this loop processes all of our sockets
*/
void listen_loop() {
     int clilen;
     int newsockfd;
     char buffer[256];
     int n;
     
     printf("Start wait loop\n");
     while(!kbhit()) {
       usleep(100);
     }

     clilen = sizeof(struct sockaddr_in);     
     newsockfd = accept(listen_sock_fd, 
                 (struct sockaddr *) &peer_addr, 
                 &clilen);
	
     printf("test\n");
		 
     if (newsockfd < 0) 
          error("ERROR on accept");
     bzero(buffer,256);
     n = read(newsockfd,buffer,255);
     if (n < 0) error("ERROR reading from socket");
     printf("Here is the message: %s\n",buffer);
     n = write(newsockfd,"I got your message",18);
     if (n < 0) error("ERROR writing to socket");
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
