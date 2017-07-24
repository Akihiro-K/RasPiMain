#include "utserial/utserial.h"
#include "mytcp/tcpserver.h"
#include "mytcp/tcpclient.h"
#include "shared/shared.h"
#include "navigator/navigator.h"
#include "stateestimator/stateestimator.h"
#include "logger/logger.h"
#include "disp/disp.h"

#include <thread>
#include <mutex>

#define UT_SERIAL_COMPONENT_ID_RASPI (2)

std::mutex m; // for lock

void FChandler(uint8_t component_id, uint8_t message_id, const uint8_t * data_buffer);

void RecvFromMarker();
void MarkerHandler(const char * src);

void RecvFromGPS();
void GPSHandler(const char * src);

void RevFromLSM();
void LSMHandler(const char * src);

void RecvFromDP();
void DPHandler(const char * src);

int main(int argc, char const *argv[])
{    
    InitLogging();

	ut_serial FC_comm("/dev/ttyAMA0", 57600);

	std::thread marker_comm(&RecvFromMarker);
	//std::thread gps_comm(&RecvFromGPS);
	//std::thread lsm_comm(&RecvFromLSM);
	//std::thread dp_comm(&RecvFromDP);
	
    for(;;) {
		if (FC_comm.recv_data(FChandler)){
			// at 128Hz

			m.lock();
			Disp();
			FCLogging();
			ToFCLogging();
			PositionTimeUpdate();
			UpdateNavigation();
			FC_comm.send_data(UT_SERIAL_COMPONENT_ID_RASPI, 1, (uint8_t *)&to_fc, sizeof(to_fc));
			m.unlock();
		}
	}
	
    marker_comm.join(); 
	//gps_comm.join();
	//lsm_comm.join();
	//dp_comm.join();
    
    return 0;
}

void FChandler(uint8_t component_id, uint8_t message_id, const uint8_t * data_buffer)
{    
    m.lock();
	struct FromFlightCtrl * struct_ptr = (struct FromFlightCtrl *)data_buffer;
	
	from_fc.timestamp = struct_ptr->timestamp;
	from_fc.nav_mode_request = struct_ptr->nav_mode_request;
	from_fc.pressure_alt = struct_ptr->pressure_alt;
	for (int i = 0; i < 3; i++) {
		from_fc.accelerometer[i] = struct_ptr->accelerometer[i];
		from_fc.gyro[i] = struct_ptr->gyro[i];
	}
	for (int i = 0; i < 4; i++) {
		from_fc.quaternion[i] = struct_ptr->quaternion[i];
	}
	m.unlock();
}

void RecvFromMarker()
{
    tcp_client c;
    c.start_connect("127.0.0.1" , 8080);

    for(;;){
		// at 10 ~ 15HZ
        c.recv_data(MarkerHandler);
    }
}

void MarkerHandler(const char * src)
{    
    m.lock();
	struct FromMarker * struct_ptr = (struct FromMarker *)src;
	
	from_marker.timestamp = struct_ptr->timestamp;
	from_marker.status = struct_ptr->status;

	
	for (int i=0;i<3;i++){
		from_marker.position[i] = struct_ptr->position[i];
		from_marker.quaternion[i] = struct_ptr->quaternion[i];
		from_marker.r_var[i] = struct_ptr->r_var[i];
	}
	
	// AttitudeMeasurementUpdateWithMarker();
	PositionMeasurementUpdateWithMarker();
	VisionLogging();
	m.unlock();
}

void RecvFromGPS()
{
    tcp_client c;
    c.start_connect("127.0.0.1" , 8000);

    for(;;){
		// at 5HZ
        c.recv_data(GPSHandler);
    }
}

void GPSHandler(const char * src)
{
    m.lock();
	struct FromGPS * struct_ptr = (struct FromGPS *)src;

	from_gps.status = struct_ptr->status;
	
	for (int i=0;i<3;i++){
		from_gps.position[i] = struct_ptr->position[i];
		from_gps.velocity[i] = struct_ptr->velocity[i];
		from_gps.r_var[i] = struct_ptr->r_var[i];
		from_gps.v_var[i] = struct_ptr->v_var[i];
	}
	
	PositionMeasurementUpdateWithGPSPos();
	PositionMeasurementUpdateWithGPSVel();
	GPSLogging();
	m.unlock();
}

void RevFromLSM()
{
    tcp_client c;
    c.start_connect("127.0.0.1" , 80);

    for(;;){
		// at HZ
        c.recv_data(LSMHandler);
    }	
}

void LSMHandler(const char * src)
{
    m.lock();
	struct FromLSM * struct_ptr = (struct FromLSM *)src;

	from_lsm.status = struct_ptr->status;
	
	for (int i=0;i<3;i++){
		from_lsm.mag[i] = struct_ptr->mag[i];
	}
	
	AttitudeMeasurementUpdateWithLSM();
	LSMLogging();
	m.unlock();
}

void RecvFromDP()
{
    tcp_client c;
    c.start_connect("127.0.0.1" , 9090);

    for(;;){
		// at HZ
        c.recv_data(DPHandler);
    }	
}

void DPHandler(const char * src)
{

}
