//
//  LSM_handler.cpp
//  FC_Safe_vector
//
//  Created by blue-i on 02/09/2017.
//  Copyright © 2017 blue-i. All rights reserved.
//

#include "LSM_handler.hpp"

FromLSMVector   LSMVector;


void RecvFromLSM()
{
    tcp_client c;
    c.start_connect(TCP_ADDRESS , TCP_PORT_LSM);

    for(;;)
    {
        c.recv_data(LSMHandler); // at HZ
    }
}

void LSMHandler(const char * src, size_t len)
{
    char temp[CLIENT_BUF_SIZE];
    memcpy(temp, src, len);
    struct FromLSM * struct_ptr = (struct FromLSM *)temp;
    FromLSM temp_s;
    temp_s.status = struct_ptr->status;

    for (int i=0;i<3;i++){
        temp_s.mag[i] = struct_ptr->mag[i];
    }

    LSMVector->push_back(temp_s);

}
