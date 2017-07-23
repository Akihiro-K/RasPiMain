#include "tcpclient.h"

bool tcp_client::start_connect(string address, int port)
{
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        perror("Socket creation error");
        return false;
    }
  
    memset(&server, '0', sizeof(server));
  
    server.sin_family = AF_INET;
    server.sin_port = htons(port);
    
    if(inet_pton(AF_INET, address.c_str(), &server.sin_addr)<=0) 
    {
        perror("Invalid address/ Address not supported");
        return false;
    }
  
    if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0)
    {
        perror("Connection Failed");
        return false;
    }   
}

bool tcp_client::send_data(const char * src, size_t len)
{
    if( send(sock , src, len, 0) < 0)
    {
        perror("Send failed : ");
        return false;
    }
     
    return true;
}

bool tcp_client::recv_data(function<void (const char *, size_t)> handler)
{
    if( recv(sock , buffer , sizeof(buffer) , 0) < 0)
    {
        perror("recv failed");
        return false;
    }

    handler((const char *)&buffer, sizeof(buffer));
    return true;
}
