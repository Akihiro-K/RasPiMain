
#include "../../libraries/Handlers/handlers.h"
#include "../../libraries/timer/timer.hpp"

#include <thread>
#include <mutex>
#include <unistd.h>

// TODO: remove this in the future
#define SERIAL_BAUDRATE_FC (57600)
#define MAIN_FREQ (64) // hz

#define ENABLE_DISP_FROM_FC (0)
#define ENABLE_DISP_TO_FC (1)
#define ENABLE_DISP_FROM_MARKER (0)
#define ENABLE_DISP_FROM_GPS (0)

const char SERIAL_PORT_FC[] = "/dev/ttyAMA0";
const char WAYPOINT_FILENAME[] = "../input_data/wp_ina_2.json";

NC nc;

//==============================================================================

int main(int argc, char const *argv[])
{
  nc.InitLogging();

  ut_serial FC_comm(SERIAL_PORT_FC, SERIAL_BAUDRATE_FC);
  std::thread dp_comm(&RecvFromDP);
  std::thread gps_handler(&GPSHandler);
  std::thread mkr_handler(&MarkerHandler);

  nc.ReadWPfromFile(WAYPOINT_FILENAME);
  if (argc == 2) {
      if (!nc.SetRouteNumber(atoi(argv[1]))) {
        return -1;
      }
  }

  Timer Main_timer(MAIN_FREQ);

  for(;;)
  {

    FC_comm.recv_data(FCHandler); // process new bytes and push back

    if(!FCVector->empty())
    {
      nc.SetFCBuffer(FCVector->back());     //get most recent data
      FCVector->clear();              //remove old datas

      // From FC
      if(ENABLE_DISP_FROM_FC) nc.DispFromFC();
      nc.ToFCLogging();
      nc.ToFCLogging2(); // This will be removed in the future
      nc.FromFCLogging();
      nc.NavigatorLogging();
      nc.PositionTimeUpdate();
      nc.AttitudeTimeUpdate();
      nc.PositionMeasurementUpdateWithBar();
      nc.UpdateNavigation();

      if(!GPSVector->empty())
      {
        nc.SetGPSBuffer(GPSVector->back());   //get most recent data
        GPSVector->clear();             //remove old datas

        //GPS
        nc.UpdateGPSPosFlag();
        nc.UpdateGPSVelFlag();
        nc.PositionMeasurementUpdateWithGPSPos();
        nc.PositionMeasurementUpdateWithGPSVel();
        if(ENABLE_DISP_FROM_GPS) nc.DispFromGPS();
        nc.GPSLogging();
      }

      if(!MarkerVector->empty())
      {
        nc.SetMarkerBuffer(MarkerVector->back());   //get most recent data
        MarkerVector->clear();             //remove old datas

        //Marker
        nc.UpdateMarkerFlag();
        nc.AttitudeMeasurementUpdateWithMarker();
        nc.PositionMeasurementUpdateWithMarker();
        if(ENABLE_DISP_FROM_MARKER) nc.DispFromMarker();
        nc.VisionLogging();
      }

      if(!DPSetDronePortModeVector->empty())
      {
        nc.SetDPBuffer(DPSetDronePortModeVector->back());   //get most recent data
        DPSetDronePortModeVector->clear();             //remove old datas

        nc.SetDronePortMode();
        SendDPSetDronePortModeResponse();
        nc.FromDPSetDronePortModeLogging();
        nc.ToDPSetDronePortModeLogging();
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
      if(ENABLE_DISP_TO_FC) nc.DispToFC();
      FC_comm.send_data(UT_SERIAL_COMPONENT_ID_RASPI, 1, (uint8_t *)nc.PayloadToFC(), sizeof(*nc.PayloadToFC()));
      nc.ResetHeadingCorrectionQuat();
    }
  }

  dp_comm.join();
  gps_handler.join();
  mkr_handler.join();

  return 0;
}
