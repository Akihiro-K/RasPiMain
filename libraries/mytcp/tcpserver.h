// Simple tcp server class
#ifndef MYTCPSERVER_H_
#define MYTCPSERVER_H_

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <functional>

#define SERVER_BUF_SIZE 16

using namespace std;

class tcp_server
{
private:
  int new_socket;
  int server_fd;
  struct sockaddr_in address;
  char buffer[SERVER_BUF_SIZE];

public:
  tcp_server() : new_socket(-1) {};
  ~tcp_server() {close(new_socket); close(server_fd);};
  bool start_listen(int);
  bool start_accept();
  bool send_data(const char *, size_t);
  bool recv_data(function<void (const char *, size_t)>);           
};

#endif // MYTCPSERVER_H_