
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

#define UBLOX_INITIAL_BAUD 9600
#define UBLOX_OPERATING_BAUD 57600

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


int ublox_fd = -1;

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
    //printf("`\n bytes processed [%zu]", bytes_processed);

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

void UBloxTxBuffer(const char b[], int t)
{
    int wr =0 ;
    while(wr != t)
    {
        if((int) write(ublox_fd,&b[wr],1)) wr++;
        printf("\n printing [%x] index[%d] written:%d", b[wr-1], wr-1, wr);
        if(wr == t || wr > t) break;
    }

    printf("\n wr = %d", wr);
}

void UART_Init(int b)
{
    //-------------------------
    //----- SETUP USART 0 -----
    //-------------------------
    //At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
    ublox_fd = -1;
    close(ublox_fd);
    //OPEN THE UART
    //The flags (defined in fcntl.h):
    //	Access modes (use 1 of these):
    //		O_RDONLY - Open for reading only.
    //		O_RDWR - Open for reading and writing.
    //		O_WRONLY - Open for writing only.
    //
    //	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //											immediately with a failure status if the output can't be written immediately.
    //
    //	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
    ublox_fd = open(UBX_PORT_NAME, O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
    if (ublox_fd == -1)
    {
        //ERROR - CAN'T OPEN SERIAL PORT
        printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
    }

    //CONFIGURE THE UART
    //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    //	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //	CSIZE:- CS5, CS6, CS7, CS8
    //	CLOCAL - Ignore modem status lines
    //	CREAD - Enable receiver
    //	IGNPAR = Ignore characters with parity errors
    //	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //	PARENB - Parity enable
    //	PARODD - Odd parity (else even)
    struct termios tty;

    memset (&tty, 0, sizeof tty);
    if (tcgetattr (ublox_fd, &tty) != 0)
    {
        if(b == 57600)
        {
              cfsetospeed (&tty, B57600);
            printf("57600 if");
        }
        else if(b == 9600)
        {
           cfsetospeed (&tty, B9600);
            printf("9600 if");
        }
    }


    if(b == 57600){
         printf("\n57600 ti");
        cfsetispeed (&tty, B57600);
    }
    if(b == 9600)
    {
        cfsetispeed (&tty, B9600);
 printf("\n9600 ti");

    }

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars

    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,

    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls

    // enable reading
    tty.c_cflag &= ~PARENB;      // shut off parity
    tty.c_cflag |= 0;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;


    tcsetattr (ublox_fd, TCSANOW, &tty);
    printf("\n Open status : %d", ublox_fd);



}

void UART_Close()
{
    close(ublox_fd);
}

void UbloxInit(){
  UART_Init(UBLOX_INITIAL_BAUD);

  {
      printf("\n Set the port to UART UBX @ 57600.\n");
      const char tx_buffer[28] = { '\xb5', '\x62', '\x06', '\x00', '\x14', '\x00', '\x01',
          '\x00', '\x00', '\x00', '\xd0', '\x08', '\x00', '\x00', '\x00', '\xe1', '\x00', '\x00', '\x01',
          '\x00', '\x01', '\x00', '\x00', '\x00', '\x00', '\x00', '\xd6', '\x8d' };
      UBloxTxBuffer(tx_buffer, 28);
  }

  usleep(150000);
  UART_Close();
  UART_Init(UBLOX_OPERATING_BAUD);

  // Enable UART Rx interrupt.


  {   printf("\n Configure USB for UBX input with no output.\n");
      const char tx_buffer[28] = { '\xb5', '\x62', '\x06', '\x00', '\x14', '\x00', '\x03',
          '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x01',
          '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x1e', '\x8c' };
      UBloxTxBuffer(tx_buffer, 28);
  }
  {   printf("\n Set antenna flags to '\x0b and pins to 0x380f.\n");
      const char tx_buffer[12] = { '\xb5', '\x62', '\x06', '\x13', '\x04', '\x00', '\x0b',
          '\x00', '\x0f', '\x38', '\x6f', '\x4f' };
      UBloxTxBuffer(tx_buffer, 12);
  }
  {   printf("\n Set measurement period to 200ms (5Hz) with UTC reference.\n");
      const char tx_buffer[14] = { '\xb5', '\x62', '\x06', '\x08', '\x06', '\x00', '\xc8',
          '\x00', '\x01', '\x00', '\x00', '\x00', '\xdd', '\x68' };
      UBloxTxBuffer(tx_buffer, 14);
  }
  {   printf("\n Configure TimPulse.\n");
      const char tx_buffer[28] = { '\xb5', '\x62', '\x06', '\x07', '\x14', '\x00', '\x40',
          '\x42', '\x0f', '\x00', '\x90', '\x86', '\x03', '\x00', '\xff', '\x01', '\x00', '\x00', '\x32',
          '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\xfd', '\x70' };
      UBloxTxBuffer(tx_buffer, 28);
  }
  {   printf("\n Configure SBAS.\n");
      const char tx_buffer[16] = { '\xb5', '\x62', '\x06', '\x16', '\x08', '\x00', '\x03',
          '\x03', '\x01', '\x00', '\x00', '\x00', '\x00', '\x00', '\x2b', '\xbd' };
      UBloxTxBuffer(tx_buffer, 16);
  }
  {   printf("\n Configure navigation engine.\n");
      const char tx_buffer[44] = { '\xb5', '\x62', '\x06', '\x24', '\x24', '\x00', '\xff',
          '\xff', '\x06', '\x02', '\x00', '\x00', '\x00', '\x00', '\x10', '\x27', '\x00', '\x00', '\x08',
          '\x3c', '\x50', '\x00', '\x32', '\x00', '\x23', '\x00', '\x23', '\x00', '\x00', '\x00', '\x00',
          '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x97',
          '\xfa' };
      UBloxTxBuffer(tx_buffer, 44);
  }
  {   printf("\n Configure navigation engine expert settings.\n");
      const char tx_buffer[48] = { '\xb5', '\x62', '\x06', '\x23', '\x28', '\x00', '\x00',
          '\x00', '\x4c', '\x06', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x04', '\x10', '\x14',
          '\x00', '\x01', '\x00', '\x00', '\x00', '\xf8', '\x05', '\x00', '\x00', '\x00', '\x00', '\x00',
          '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00',
          '\x00', '\x00', '\x00', '\xc9', '\xea' };
      UBloxTxBuffer(tx_buffer, 48);
  }
  {   printf("\n Request NAV-POSLLH message to be output every measurement cycle.\n");
      const char tx_buffer[11] = { '\xb5', '\x62', '\x06', '\x01', '\x03', '\x00', '\x01',
          '\x02', '\x01', '\x0e', '\x47' };
      UBloxTxBuffer(tx_buffer, 11);
  }
  {   printf("\n Request NAV-VELNED message to be output every measurement cycle.\n");
      const char tx_buffer[11] = { '\xb5', '\x62', '\x06', '\x01', '\x03', '\x00', '\x01',
          '\x12', '\x01', '\x1e', '\x67' };
      UBloxTxBuffer(tx_buffer, 11);
  }
  {   printf("\n Request NAV-SOL message to be output every measurement cycle.\n");
      const char tx_buffer[11] = { '\xb5', '\x62', '\x06', '\x01', '\x03', '\x00', '\x01',
          '\x06', '\x01', '\x12', '\x4f' };
      UBloxTxBuffer(tx_buffer, 11);
  }
  {   printf("\n Request Time-UTC message to be output every 5 measurement cycles.\n");
      const char tx_buffer[11] = { '\xb5', '\x62', '\x06', '\x01', '\x03', '\x00', '\x01',
          '\x21', '\x05', '\x31', '\x89' };
      UBloxTxBuffer(tx_buffer, 11);
  }
}

void UbloxLoop(){
  unsigned char c = '\0';
  int t = -1;
  while(t != 1)
  {

      t = (int)read(ublox_fd, &c, 1);
  }

  ProcessIncomingUBloxByte(c);

  //if(UBXNewDataAvailable()){
  //  const struct UBXPayload * temp_;
  //    temp_ = UBXPayload();
  //    printf("\n lon:%u lat:%u height:%f v:[%f][%f][%f] stat:%u",
  //      temp_->longitude, temp_->latitude, temp_->z,
  //      temp_->velocity[0], temp_->velocity[1], temp_->velocity[2],
  //      temp_->gps_status);
  //  ClearUBXNewDataFlags();
  //}

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
