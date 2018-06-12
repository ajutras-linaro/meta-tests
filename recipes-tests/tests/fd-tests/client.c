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
 * Name: fd-tests (client)
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

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <rpc/rpc.h>

#include "linux/ion.h"  


// i.MX8M VPU secure heap
#define ION_SECURE_HEAP_ID_DECODER 2

#define SECURE_BUFFER_SIZE (16)

/*
By default the test is performed on a file descriptor from an allocated ION buffer.
To perform the test with an open file descriptor, enable the following definition
in the client and server source code. On the target, create the file by running:

dd if=/dev/random of=/tmp/anyfile.bin count=1 bs=16
*/
//#define SHARE_OPENED_FILE


static int connectSocket(void)
{
  int socket_fd = -1;
  struct sockaddr_un socketAddress; 
    
  socket_fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if(socket_fd < 0) {
    printf("connectSocket: Failure to create socket\n");
    goto handle_error;
  }
  
  /* Use abstract socket (First byte is \0). */
  memset(&socketAddress, 0, sizeof(socketAddress));
  socketAddress.sun_family = AF_UNIX;
  strcpy(&socketAddress.sun_path[1], "secure_fd_socket");
  
  if(connect(socket_fd, (struct sockaddr *)&socketAddress, sizeof(socketAddress)) < 0) {
	  close(socket_fd);
    socket_fd = -1;
    printf("connectSocket: Failure to connect socket\n");
    goto handle_error;
  }
    
handle_error:
  return socket_fd;
}

static int sendFileDescriptor(int socket_fd, int secure_fd, uint32_t secure_size)
{
    int status = -1;
 
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
    *(int *)CMSG_DATA(cmsg) = secure_fd;
  
    status = sendmsg(socket_fd, &msg, 0);
    status = (status < 0) ? -1 : 0;
	  if(status < 0) {
		    printf("Cannot send FD\n");
    }
	
    return status;
}

static int openFile(void)
{
  int fd = -1;
  
  fd = open("/tmp/anyfile.bin", O_RDONLY);
  if(fd < 0) {
     printf("openFile: Failure to open: /tmp/anyfile.bin\n");
  }
  
  return fd;
}


// The caller is responsible to close the returned file descriptor.
static int allocate_ion_buffer(size_t size)
{
  struct ion_allocation_data alloc_data;
  struct ion_handle_data hdl_data;
  struct ion_fd_data fd_data;
  int ion;
  int fd = -1;

  ion = open("/dev/ion", O_RDWR);
  if (ion < 0) {
    printf("Failed to open /dev/ion\n");
    return fd;
  }

  alloc_data.len = size;
  alloc_data.align = 0;
  alloc_data.flags = 0;
  alloc_data.heap_id_mask = 1 << ION_SECURE_HEAP_ID_DECODER;

  if (ioctl(ion, ION_IOC_ALLOC, &alloc_data) == -1) {
    printf("Failed to allocate secure buffer from ION heap ID %u\n", alloc_data.heap_id_mask);
    goto out;
  }

  fd_data.handle = alloc_data.handle;
  if (ioctl(ion, ION_IOC_SHARE, &fd_data) != -1)
    fd = fd_data.fd;
  else
    printf("Failed to share secure buffer.\n");

  hdl_data.handle = alloc_data.handle;
  (void)ioctl(ion, ION_IOC_FREE, &hdl_data);
out:
  close(ion);
  return fd;
}


void main(void)
{
	  int status = 0;
    int socket_fd = -1;
	  int fd = -1;
    uint8_t *secureMem = NULL;

    printf("Starting client application\n");

    printf("Connecting to the server\n");
    socket_fd = connectSocket();
    if(socket_fd < 0) {
        status = -1;
        goto handle_error;
    }

#ifdef SHARE_OPENED_FILE
    printf("Opening a file\n");
    fd = openFile();
    if(fd < 0) {
      status = -1;
      goto handle_error;
    }  
  
    // Send file FD
    printf("Sending the file descriptor (%d)\n", fd);
    status = sendFileDescriptor(socket_fd, fd, 0);
    if(status < 0) {
      goto handle_error;
    }

#else // Share secure FD
    printf("Allocating an ION buffer\n");
	  fd = allocate_ion_buffer(SECURE_BUFFER_SIZE);
    if(fd < 0) {
        status = -1;
        goto handle_error;
    }

    // Map the ION buffer
    printf("Writing some data into the ION buffer\n");
    secureMem = (uint8_t *)mmap(0, (SECURE_BUFFER_SIZE), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if(secureMem == NULL) {
        printf("Cannot map ION\n");
        status = -1;
        goto handle_error;      
    }
    memset(secureMem, 0xa3, SECURE_BUFFER_SIZE);
    if(secureMem != NULL) munmap(secureMem, SECURE_BUFFER_SIZE);
	
	  printf("Sending the ION FD (%d)\n", fd);
    status = sendFileDescriptor(socket_fd, fd, SECURE_BUFFER_SIZE);
    if(status < 0) {
      goto handle_error;
    }
#endif
  
handle_error:
    if(fd >= 0) close(fd);
    if(socket_fd >= 0) close(socket_fd);
  
    printf("Exiting\n");
    return;
}


