
#include "../../libraries/Handlers/handlers.h"
#include "../../libraries/timer/timer.hpp"
#include "../../libraries/adctrl/adctrl.h"

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

#define ENABLE_ADAPTIVE_CONTROL (0)

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
  std::thread pl_handler(&PayloadHandler);

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
      FCVector->clear();              //remove old data

      // From FC
      if(ENABLE_DISP_FROM_FC) nc.DispFromFC();
      nc.ToFCLogging();
      nc.ToFCLogging2(); // This will be removed in the future
      nc.FromFCLogging();
      nc.NavigatorLogging();
      // Update state estimate using data from FC
      nc.PositionTimeUpdate();
      nc.AttitudeTimeUpdate();
      nc.PositionMeasurementUpdateWithBar();

      // Refine state estimate using data from GPS
      if(!GPSVector->empty())
      {
        nc.SetGPSBuffer(GPSVector->back());   //get most recent data
        GPSVector->clear();             //remove old data

        //GPS
        nc.UpdateGPSPosFlag();
        nc.UpdateGPSVelFlag();
        nc.PositionMeasurementUpdateWithGPSPos();
        nc.PositionMeasurementUpdateWithGPSVel();
        if(ENABLE_DISP_FROM_GPS) nc.DispFromGPS();
        nc.GPSLogging();
      }

      // Refine state estimate using data from marker
      if(!MarkerVector->empty())
      {
        nc.SetMarkerBuffer(MarkerVector->back());   //get most recent data
        MarkerVector->clear();             //remove old data

        //Marker
        nc.UpdateMarkerFlag();
        nc.AttitudeMeasurementUpdateWithMarker();
        nc.PositionMeasurementUpdateWithMarker();
        if(ENABLE_DISP_FROM_MARKER) nc.DispFromMarker();
        nc.VisionLogging();
      }

      if(!PayloadVector->empty())
      {
        nc.SetPayloadBuffer(PayloadVector->back());
        PayloadVector->clear();

        // Execute Kalman filter on payload states: [theta phidot phi]T
        nc.PayloadStatesKalmanUpdate();
        nc.FromPayloadLogging();
      }

      //if(!LSMVector->empty())
      //{
      //    from_lsm = LSMVector->back();   //get most recent data
      //    LSMVector->clear();             //remove old data
      //
      //    //LSM
      //    UpdateLSMFlag();
      //    AttitudeMeasurementUpdateWithLSM();
      //    LSMLogging();
      //}

      // Receive command from drone port
      if(!DPSetDronePortModeVector->empty())
      {
        nc.SetDPBuffer(DPSetDronePortModeVector->back());   //get most recent data
        DPSetDronePortModeVector->clear();             //remove old data

        nc.SetDronePortMode();
        SendDPSetDronePortModeResponse();
        nc.FromDPSetDronePortModeLogging();
        nc.ToDPSetDronePortModeLogging();
      }

      // Update navigation to generate payload to FC
      nc.UpdateNavigation();

      if(ENABLE_ADAPTIVE_CONTROL){
        static L1 l1z = ReadL1Params("../input_data/l1z.json");

        // Obtain nominal target position
        std::vector<float> target_position = nc.ToFCTargetPosition();

        // L1 z control
        VectorXf zstates = nc.ZStates();
        float z_ad = l1z.update(zstates,target_position[2]);

        // Baseline x swing control
        Eigen::Vector3f xpmstates = nc.XPMStates();
        Eigen::Vector3f ypmstates = nc.YPMStates();

        // Set adaptive target position
        target_position[2] = z_ad;
        nc.SetToFCTargetPosition(target_position);
      }

      // Send data to FC
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
