#include "tcpserver.h"

bool tcp_server::start_listen(int port)
{
  // Creating socket file descriptor
  if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
  {
    perror("socket failed");
    return false;
  }

  // Forcefully attaching socket to the port
  int opt = 1;
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                                                &opt, sizeof(opt)))
  {
    perror("setsockopt");
    return false;
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(port);

  // Forcefully attaching socket to the port 8080
  if (bind(server_fd, (struct sockaddr *)&address,
                                sizeof(address))<0)
  {
    perror("bind failed");
    return false;
  }
  if (listen(server_fd, 3) < 0)
  {
    perror("listen");
    return false;
  }

  return true;
}

bool tcp_server::start_accept()
{
  int addrlen = sizeof(address);
  if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
                      (socklen_t*)&addrlen))<0)
  {
    perror("accept");
    return false;
  }

  return true;
}

bool tcp_server::send_data(const char * src, size_t len)
{
  if( send(new_socket , src, len, 0) < 0)
  {
    perror("Send failed");
    return false;
  }

  return true;
}

bool tcp_server::recv_data(std::function<void (const char *, size_t)> handler)
{
  if( recv(new_socket , buffer , sizeof(buffer) , 0) < 0)
  {
    perror("recv failed");
    return false;
  }

  handler((const char *)&buffer, sizeof(buffer));
  return true;
}
