//
//  GPS_handler.cpp
//  FC_Safe_vector
//
//  Created by blue-i on 02/09/2017.
//  Copyright © 2017 blue-i. All rights reserved.
//

#include "GPS_handler.hpp"

FromGPSVector   GPSVector;

void GPSHandler()
{
    int fd = open("/dev/gps_fifo", O_RDONLY | O_NONBLOCK);
    for(;;)
    {
        struct pollfd temp;
        temp.fd = fd;
        temp.events = POLLIN;
        poll(&temp, POLLIN, -1);
        unsigned char c;
        int r = (int) read(fd, &c, 1);
        if(r)
        {
            ProcessIncomingUBloxByte(c);
        }

        if(UBXNewDataAvailable())
        {

            const struct UBXPayload * struct_ptr;
            struct_ptr = UBXPayload();
            printf("\n lon:%u lat:%u height:%f v:[%f][%f][%f] stat:%u",
                   struct_ptr->longitude, struct_ptr->latitude, struct_ptr->z,
                   struct_ptr->velocity[0], struct_ptr->velocity[1], struct_ptr->velocity[2],
                   struct_ptr->gps_status);

            FromGPS temp;
            temp.longitude = struct_ptr->longitude;
            temp.latitude = struct_ptr->latitude;
            temp.z = struct_ptr -> z;
            temp.gps_status = struct_ptr->gps_status;
            for (int i=0;i<3;i++)
            {
                temp.velocity[i] = struct_ptr->velocity[i];
            }


            GPSVector->push_back(temp); //put the fresh frame in the vector


            ClearUBXNewDataFlags();
        }
    }

}
