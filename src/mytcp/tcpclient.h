// Simple tcp client class
#ifndef MYTCPCLIENT_H_
#define MYTCPCLIENT_H_

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <functional>

#define CLIENT_BUF_SIZE 64

using namespace std;

class tcp_client
{
private:
  int sock;
  struct sockaddr_in server;
  char buffer[CLIENT_BUF_SIZE];

public:
  tcp_client() : sock(-1) {};
  ~tcp_client() {close(sock);};
  bool start_connect(string, int);
  bool send_data(const char *, size_t);
  bool recv_data(function<void (const char *, size_t)>);   
};

#endif // MYTCPCLIENT_H_