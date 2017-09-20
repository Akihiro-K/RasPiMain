
#include "../../libraries/Handlers/handlers.h"
#include "../../libraries/timer/timer.hpp"

#include <thread>
#include <mutex>
#include <unistd.h>

// TODO: remove this in the future
#define SERIAL_BAUDRATE_FC (57600)
#define MAIN_FREQ 64 // hz

#define ENABLE_DISP_FROM_FC (0)
#define ENABLE_DISP_TO_FC (1)
#define ENABLE_DISP_FROM_MARKER (0)
#define ENABLE_DISP_FROM_GPS (0)

const char SERIAL_PORT_FC[] = "/dev/ttyAMA0";
const char WAYPOINT_FILENAME[] = "../input_data/wp_ina_2.json";

//==============================================================================

int main(int argc, char const *argv[])
{
    InitLogging();
    ut_serial FC_comm(SERIAL_PORT_FC, SERIAL_BAUDRATE_FC);
    std::thread dp_comm(&RecvFromDP);
    std::thread gps_handler(&GPSHandler);
    std::thread mkr_handler(&MarkerHandler);

    ReadWPfromFile(WAYPOINT_FILENAME);
    if (argc == 2) {
        if (!SetRouteNumber(atoi(argv[1]))) {
            return -1;
        }
    }

    Timer Main_timer(MAIN_FREQ);

    for(;;)
    {
      
        FC_comm.recv_data(FCHandler); // process new bytes and push back

        if(!FCVector->empty())
        {
            from_fc = FCVector->back();     //get most recent data
            FCVector->clear();              //remove old datas

            // From FC
            if(ENABLE_DISP_FROM_FC) DispFromFC();
            ToFCLogging();
            ToFCLogging2(); // This will be removed in the future
            FromFCLogging();
            NavigatorLogging();
            PositionTimeUpdate();
            AttitudeTimeUpdate();
            PositionMeasurementUpdateWithBar();
            UpdateNavigation();

            if(!GPSVector->empty())
            {
                from_gps = GPSVector->back();   //get most recent data
                GPSVector->clear();             //remove old datas

                //GPS
                UpdateGPSPosFlag();
                UpdateGPSVelFlag();
                PositionMeasurementUpdateWithGPSPos();
                PositionMeasurementUpdateWithGPSVel();
                if(ENABLE_DISP_FROM_GPS) DispFromGPS();
                GPSLogging();
            }

            if(!MarkerVector->empty())
            {
                from_marker = MarkerVector->back();   //get most recent data
                MarkerVector->clear();             //remove old datas

                //Marker
                UpdateMarkerFlag();
                AttitudeMeasurementUpdateWithMarker();
                PositionMeasurementUpdateWithMarker();
                if(ENABLE_DISP_FROM_MARKER) DispFromMarker();
                VisionLogging();
            }

            if(!DPSetDronePortModeVector->empty())
            {
                from_dp_set_dp_mode = DPSetDronePortModeVector->back();   //get most recent data
                DPSetDronePortModeVector->clear();             //remove old datas

                drone_port_mode_request = from_dp_set_dp_mode.drone_port_mode_request;
                SetDronePortMode();
                SendDPSetDronePortModeResponse();
                FromDPSetDronePortModeLogging();
                ToDPSetDronePortModeLogging();
            }

            //if(!LSMVector->empty())
            //{
            //    from_lsm = LSMVector->back();   //get most recent data
            //    LSMVector->clear();             //remove old datas
            //
            //    //LSM
            //    UpdateLSMFlag();
            //    AttitudeMeasurementUpdateWithLSM();
            //    LSMLogging();
            //}

            // To FC
            if(ENABLE_DISP_TO_FC) DispToFC();
            FC_comm.send_data(UT_SERIAL_COMPONENT_ID_RASPI, 1, (uint8_t *)&to_fc, sizeof(to_fc));
            ResetHeadingCorrectionQuat();
        }
    }


    // dp_comm.join();
    // gps_handler.join();

    return 0;
}
