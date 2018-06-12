/*
 * Copyright (c) 2018, Linaro Limited
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/******************************************************************************
 * Name: fd-tests (server)
 *
 * Description: Simple test application to qualify passing a file descriptor
 * between processes. It can be configured to pass a ION file descriptor 
 * (default) or to pass an open file descriptor. 
 *****************************************************************************/

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/mman.h>

#include <rpc/rpc.h>


/*
By default the test is performed on a file descriptor from an allocated ION buffer.
To perform the test with an open file descriptor, enable the following definition
in the client and server source code. On the target, create the file by running:

dd if=/dev/random of=/tmp/anyfile.bin count=1 bs=16
*/
//#define SHARE_OPENED_FILE


int connectSocket(void)
{
    int status = 0;
    int l_socket_fd = -1;   // Listening socket
    int c_socket_fd = -1; // Connected socket
    struct sockaddr_un socketAddress; 
    uint32_t trials = 1000;
  
    l_socket_fd = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if(l_socket_fd < 0) {
        printf("connectSocket: Failure to create socket\n");
        status = -1;
        goto handle_error;
    }

    /* Use abstract socket (First byte is \0). */
    memset(&socketAddress, 0, sizeof(socketAddress));
    socketAddress.sun_family = AF_UNIX;
    strcpy(&socketAddress.sun_path[1], "secure_fd_socket");

    if(bind(l_socket_fd,  (struct sockaddr *)&socketAddress, sizeof(socketAddress)) < 0) {
        printf("connectSocket: Failure to bind socket\n");
        status = -1;
        goto handle_error;
    }

    if(listen(l_socket_fd, 1) < 0) {
        printf("connectSocket: Failure to set to listen state\n");
        status = -1;
        goto handle_error;   
    }

    while(trials > 0)
    {
        c_socket_fd = accept(l_socket_fd, NULL, NULL);
        if(c_socket_fd >= 0) {
            /* Connection accepted */
            break; 
        } else if(errno != EWOULDBLOCK &&
                  errno != EAGAIN) {
            printf("connectSocket: Failure to accept connection\n");
            status = -1;
            goto handle_error; 
        }

        usleep(5000); // TODO Reduce this in real SDP use case
        trials--;
    }
    if(trials == 0) {
        printf("connectSocket: Timeout to accept connection\n");
        status = -1;
        goto handle_error; 
    }
    
handle_error:
    if(status < 0) {
        if(c_socket_fd >= 0) {
            close(c_socket_fd);
            c_socket_fd = -1;
        }
    }

    if(l_socket_fd >= 0) {
        // Listening socket may be closed
        close(l_socket_fd);
        l_socket_fd = -1;
    }
  
    return c_socket_fd;
}

int receiveFileDescriptor(int socket_fd)
{
    int status = 0; 
    int secure_fd = -1;
    uint32_t secure_size = 0;
  
	  struct msghdr msg; 
	  struct iovec iov;

    /* Control message buffer contains the control message structure plus
       one file descriptor. */
    #define CMSG_SIZE (sizeof(struct cmsghdr) + sizeof(int))
    uint8_t cmsg_buffer[CMSG_SIZE] = {0};
    struct cmsghdr *cmsg; // Pointer to control message

    /* Send secure buffer size with the file descriptor */
	  iov.iov_base = &secure_size;
	  iov.iov_len  = sizeof(secure_size);

    msg.msg_name       = NULL;
    msg.msg_namelen    = 0;
    msg.msg_iov        = &iov;
    msg.msg_iovlen     = 1;
    msg.msg_control    = cmsg_buffer;
    msg.msg_controllen = CMSG_SIZE;
    msg.msg_flags      = 0; /* ignored */

    cmsg = CMSG_FIRSTHDR(&msg);
    cmsg->cmsg_len = CMSG_SIZE;
    cmsg->cmsg_level = SOL_SOCKET;
    cmsg->cmsg_type = SCM_RIGHTS;
    *(int *)CMSG_DATA(cmsg) = -1;

    status = recvmsg(socket_fd, &msg, 0);
    status = (status < 0) ? -1 : 0;
	  if(status < 0) {
		    printf("Cannot receive FD\n");
        goto handle_error;
    }

    printf("Buffer size is %u bytes\n", secure_size);
  
    secure_fd = *(int *)CMSG_DATA(cmsg);
    if(secure_fd < 0) {
		    printf("Invalid FD received\n");
    }
  
handle_error:
    return secure_fd;
}

void printFileData(int secure_fd)
{
    uint8_t buffer[4] = {0};
    uint32_t bufferSize = 0;

    bufferSize = lseek(secure_fd, 0, SEEK_END);
    printf("File size is %u bytes\n", bufferSize);
  
    if(lseek(secure_fd, 0, SEEK_SET) != 0) {
        printf("printMemory: Failure to seek\n");
    }

    if(read(secure_fd, buffer, 4) != 4) {
        printf("printMemory: Failure to read\n");
    }

    printf("Data is 0x%02x %02x %02x %02x\n", buffer[0], buffer[1], buffer[2], buffer[3]);
  
    return;
}

void printSecureMemory(int secure_fd)
{
    uint8_t *secureMem = NULL;
    uint32_t secureMemSize = 0;

    secureMemSize = lseek(secure_fd, 0, SEEK_END);
    printf("Secure memory size is %u bytes\n", secureMemSize);
  
    // Map the ION buffer
    secureMem = (uint8_t *)mmap(0, (secureMemSize), PROT_READ | PROT_WRITE, MAP_SHARED, secure_fd, 0);
    if(secureMem == NULL) {
        printf("Cannot map ION\n");
        return;
    }

    printf("Secure memory is 0x%02x %02x %02x %02x\n", secureMem[0], secureMem[1], secureMem[2], secureMem[3]);
    //printf("                 ...\n");
    //printf("                 0x%02x %02x %02x %02x\n", secureMem[secureMemSize - 4], secureMem[secureMemSize - 3], secureMem[secureMemSize - 2], secureMem[secureMemSize - 1]);

    if(secureMem != NULL) munmap(secureMem, secureMemSize); 
}


void main(void) {
    int socket_fd = -1;
	  int fd = -1;
	
    printf("Starting server application\n");
	
	  printf("Creating a socket and waiting for a connection\n");
	  socket_fd = connectSocket();
    if(socket_fd < 0) {
        goto handle_error;
    }

	  printf("Receiving the file descriptor\n");
	  fd = receiveFileDescriptor(socket_fd);
    if(fd < 0) {
        goto handle_error;
    }
  
	  printf("Reading some data from the file descriptor (%d)\n", fd);
#ifdef SHARE_OPENED_FILE
	  printFileData(fd);
#else
	  printSecureMemory(fd);
#endif

handle_error:
    if(socket_fd >= 0) {
        close(socket_fd); 
    }
    if(fd >= 0) {
        close(fd); 
    }
    
	  printf("Exiting\n");

	  return;
}


