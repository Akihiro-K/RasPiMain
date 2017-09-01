
// Details:
// https://github.com/GAVLab/ublox/blob/master/include/ublox/ublox_structures.h

#include <iostream>
#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>        //Used for UART
#include <math.h>
#include <sys/ioctl.h>
#include "string.h"

#define UBX_PORT_NAME ("/dev/ttyUSB_Ublox")

#define UBX_SYNC_CHAR_1 (0xb5)
#define UBX_SYNC_CHAR_2 (0x62)
#define UBX_CLASS_NAV (0x01)
#define UBX_ID_POS_LLH (0x02)
#define UBX_ID_VEL_NED (0x12)
#define UBX_ID_SOL (0x06)
#define UBX_ID_TIME_UTC (0x21)

#define UBX_FRESHNESS_LIMIT (500)  // millisends

static void CopyUBloxMessage(uint8_t id);
struct UBXPosLLH
{
    uint32_t gps_ms_time_of_week;
    int32_t longitude; // longitude in degrees. Scaling 1e-7
    int32_t latitude; //!< latitude in degrees. Scaling 1e-7
    int32_t height_above_ellipsoid;
    int32_t height_mean_sea_level; //!< height above mean sea level [mm]
    uint32_t horizontal_accuracy; //!< horizontal accuracy estimate [mm]
    uint32_t vertical_accuracy; //!< vertical accuracy estimate [mm]
} __attribute__((packed));

struct UBXVelNED
{
    uint32_t gps_ms_time_of_week;
    int32_t velocity_north; //!< north velocity [cm/s]
    int32_t velocity_east; //!< east velocity [cm/s]
    int32_t velocity_down; //!< down velocity [cm/s]
    uint32_t total_speed;
    uint32_t horizontal_speed;
    int32_t course;
    uint32_t speed_accuracy;
    uint32_t course_accuracy;
} __attribute__((packed));

struct UBXSol
{
    uint32_t gps_ms_time_of_week;
    int32_t fractional_time_of_week;
    int16_t gps_week;
    uint8_t gps_fix_type;
    uint8_t gps_fix_status_flags;
    int32_t ecef_x_coordinate;
    int32_t ecef_y_coordinate;
    int32_t ecef_z_coordinate;
    uint32_t coordinate_accuracy;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t velocity_accuracy;
    uint16_t position_dop;
    uint8_t reserved1;
    uint8_t number_of_satelites_used;
    uint32_t reserved2;
} __attribute__((packed));

struct UBXTimeUTC
{
    uint32_t gps_ms_time_of_week;
    uint32_t t_acc;
    int32_t nano;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
} __attribute__((packed));

struct UBXPayload {
  int32_t longitude; // [10^-7 deg]
  int32_t latitude; // [10^-7 deg]
  float z; // height above sea level [m], downward positive
  float velocity[3]; // [m/s]
  uint8_t gps_status; // 3: pos & vel OK 2: only pos OK 1: only vel OK 0: unavailable
} __attribute__((packed));

enum UBXNewDataBits {
    UBX_NEW_DATA_BIT_POS_LLH  = 1<<0,
    UBX_NEW_DATA_BIT_VEL_NED  = 1<<1,
    UBX_NEW_DATA_BIT_SOL      = 1<<2,
    UBX_NEW_DATA_BIT_TIME_UTC = 1<<3,
};

enum UBXErrorBits {
    UBX_ERROR_BIT_STALE = 1<<0,
};

static struct UBXPosLLH ubx_pos_llh_;
static struct UBXVelNED ubx_vel_ned_;
static struct UBXSol ubx_sol_;
static struct UBXTimeUTC ubx_time_utc_;
static struct UBXPayload ubx_payload_;

static enum UBXErrorBits error_bits_ = UBX_ERROR_BIT_STALE;
static uint32_t status_ = 0;
static uint32_t new_data_bits_ = 0;
static uint32_t last_reception_timestamp_ = 0;

#define UBLOX_DATA_BUFFER_LENGTH (sizeof(struct UBXSol))

static uint8_t data_buffer_[UBLOX_DATA_BUFFER_LENGTH];


//int ublox_fd = -1;

// =============================================================================

uint32_t UBXNewDataBits(void)
{
  return new_data_bits_;
}

// -----------------------------------------------------------------------------
void ClearUBXNewDataBit(enum UBXNewDataBits new_data_bit)
{
  new_data_bits_ &= ~new_data_bit;
}

// -----------------------------------------------------------------------------
const struct UBXPosLLH * UBXPosLLH(void)
{
  return &ubx_pos_llh_;
}

// -----------------------------------------------------------------------------
const struct UBXVelNED * UBXVelNED(void)
{
  return &ubx_vel_ned_;
}

const struct UBXPayload * UBXPayload(void)
{
  ubx_payload_.longitude = ubx_pos_llh_.longitude;
  ubx_payload_.latitude = ubx_pos_llh_.latitude;
  ubx_payload_.z = ubx_pos_llh_.height_mean_sea_level;
  ubx_payload_.velocity[0] = float(ubx_vel_ned_.velocity_north)/100.0f;
  ubx_payload_.velocity[1] = float(ubx_vel_ned_.velocity_east)/100.0f;
  ubx_payload_.velocity[2] = float(ubx_vel_ned_.velocity_down)/100.0f;
  ubx_payload_.gps_status = 3;
  return &ubx_payload_;
}

// -----------------------------------------------------------------------------
uint8_t UBXNewDataAvailable(void){
  return new_data_bits_& UBX_NEW_DATA_BIT_POS_LLH;
}

// -----------------------------------------------------------------------------
void ClearUBXNewDataFlags(void){
  new_data_bits_ &= ~UBX_NEW_DATA_BIT_POS_LLH;
  new_data_bits_ &= ~UBX_NEW_DATA_BIT_VEL_NED;
}

// =============================================================================

static void ProcessIncomingUBloxByte(uint8_t byte)
{
    static size_t bytes_processed = 0, payload_length = 0;
    static uint8_t id, checksum_a, checksum_b;
    static uint8_t * data_buffer_ptr = NULL;
    printf("`\n bytes processed [%zu]", bytes_processed);

    switch (bytes_processed)
    {
        case 0:  // Sync char 1
            if (byte != UBX_SYNC_CHAR_1) goto RESET;
            break;
        case 1:  // Sync char 2
            if (byte != UBX_SYNC_CHAR_2) goto RESET;
            break;
        case 2:  // Class (NAV)
            if (byte != UBX_CLASS_NAV) goto RESET;
            checksum_a = byte;
            checksum_b = byte;
            break;
        case 3:  // ID
            id = byte;
            break;
        case 4:  // Payload length (lower byte)
            if (byte > UBLOX_DATA_BUFFER_LENGTH) goto RESET;
            payload_length = byte;
            data_buffer_ptr = &data_buffer_[0];
        case 5:  // Payload length (upper byte should always be zero)
            break;
        default:  // Payload or checksum
            if (bytes_processed < (6 + payload_length))  // Payload
            {
                *data_buffer_ptr++ = byte;
            }
            else if (bytes_processed == (6 + payload_length))  // Checksum A
            {

            }
            else  // Checksum B
            {
                CopyUBloxMessage(id);
                goto RESET;
            }
            break;
    }
    bytes_processed++;
    return;

RESET:
    bytes_processed = 0;
}

static void CopyUBloxMessage(uint8_t id)
{
    // TODO: do this in a more efficient way
    switch (id)
    {
        case UBX_ID_POS_LLH:
            memcpy(&ubx_pos_llh_, &data_buffer_[0], sizeof(struct UBXPosLLH));
            status_ = ubx_pos_llh_.horizontal_accuracy < 5000;
            new_data_bits_ |= UBX_NEW_DATA_BIT_POS_LLH;
            //UpdatePositionToFlightCtrl(UBLOX);
            break;
        case UBX_ID_VEL_NED:
            memcpy(&ubx_vel_ned_, &data_buffer_[0], sizeof(struct UBXVelNED));
            new_data_bits_ |= UBX_NEW_DATA_BIT_VEL_NED;
            //UpdateVelocityToFlightCtrl(UBLOX);
            break;
        case UBX_ID_SOL:
            memcpy(&ubx_sol_, &data_buffer_[0], sizeof(struct UBXSol));
            new_data_bits_ |= UBX_NEW_DATA_BIT_SOL;
            break;
        case UBX_ID_TIME_UTC:
            memcpy(&ubx_time_utc_, &data_buffer_[0], sizeof(struct UBXTimeUTC));
            new_data_bits_ |= UBX_NEW_DATA_BIT_TIME_UTC;
            break;
    }

    //printf("\nid:%x longitude: %u vel_north: %i sat:%u", id, ubx_pos_llh_.longitude, ubx_vel_ned_.velocity_north, ubx_sol_.number_of_satelites_used);
    //printf("\n GPS_fix_type %d", ubx_sol_.gps_fix_type);

    //last_reception_timestamp_ = GetTimestamp();
    //error_bits_ &= ~UBX_ERROR_BIT_STALE;
}
