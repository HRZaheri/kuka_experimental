/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Norwegian University of Science and Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Norwegian University of Science and
 *     Technology, nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <stdio.h>
#include <stdlib.h>
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>

#include <errno.h>
#include <string.h>

#define BUFSIZE 1024


class TCPServer
{
public:
  TCPServer(unsigned short port)
  {
    /* First call to socket() function */
    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd < 0) {
    perror("ERROR opening socket");
    exit(1);
    }

    /* Initialize socket structure */
    bzero((char *) &serv_addr, sizeof(serv_addr));
    //portno = 6008;

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(port);
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    {
      std::cout << "***********************" << std::endl;
      std::cout << "ERROR in binding socket" << std::endl;
      std::cout << "Cannot bind socket: " <<  errno << " " << strerror(errno) << std::endl;
      std::cout << "***********************" << std::endl;
    }
    else{
        std::cout << "connection established " << std::endl;
    }
    listen(sockfd,5);
    clilen = sizeof(cli_addr);

    /* Accept actual connection from the client */
    newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);

    if (newsockfd < 0) {
      perror("ERROR on accept");
      exit(1);
    }
  }

  ~TCPServer()
  {
    //close(sockfd_);
  }

  int send(std::string& buffer)
  {
      /* Write a response to the client */
    n = write(newsockfd, buffer.c_str(), buffer.size());

    if (n < 0) {
        perror("ERROR while writing to socket");
    }
    return n;
  }


  int recv(std::string& buffer)
  {
      bzero(buffer_,BUFSIZE);
      n = read( newsockfd, buffer_, BUFSIZE-1 );

      if (n < 0) {
        perror("ERROR while reading from socket");
        exit(1);
      }
      buffer = std::string(buffer_);

      return n;
  }

private:
  int sockfd, newsockfd, portno;
  socklen_t clilen;
  char buffer_[BUFSIZE];
  struct sockaddr_in serv_addr, cli_addr;
  int  n;
};

#endif // TCP_SERVER_H
