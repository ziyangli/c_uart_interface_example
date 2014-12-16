/** This example is public domain. */

/**
 * @file mavlink_control.cpp
 *
 * @brief The serial interface process
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock, <jaycee.lock@gmail.com>
 *
 */

/**
 * TODO: Create simple trajectory, such as takeoff, flying in a circle, and landing
 * TODO: Add safety, such as instant landing, in case something goes wrong
 * TODO: Send attitude target message
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"
#include "serial_port.h"
// #include "system_ids.h"
#include "offboard_setup.h"

#include <time.h>
#include <sys/time.h>

const uint8_t sysid            = 1;   // The vehicle's system ID (parameter: MAV_SYS_ID)
const uint8_t autopilot_compid = 50;  // The autopilot component (parameter: MAV_COMP_ID)
const uint8_t compid           = 110; // The offboard computer component ID */

static int imu_cnt = 0;
static int att_cnt = 0;
static int manual_cnt = 0;
static int gps_cnt = 0;
static int sys_cnt = 0;
static int rc_cnt = 0;

void print_current_time(void)
{

    time_t current_time;
    char* c_time_string;

    /* Obtain current time as seconds elapsed since the Epoch. */
    current_time = time(NULL);

    if (current_time == ((time_t)-1))
        {
            (void) fprintf(stderr, "Failure to compute the current time.");
        }

    /* Convert to local time format. */
    c_time_string = ctime(&current_time);

    if (c_time_string == NULL)
        {
            (void) fprintf(stderr, "Failure to convert the current time.");
        }

    /* Print to stdout. */
    (void) printf("Current time is %s", c_time_string);

}

int
top(int argc, char **argv)
{

    char *uart_name = (char*)"/dev/ttyTHS1";
    int baudrate = 921600;

    // signal(SIGINT, quit_handler);

    parse_commandline(argc, argv, uart_name, baudrate);

    printf("OPEN PORT\n");

    open_serial(uart_name, baudrate);

    printf("\n");

    printf("READ MAVLINK\n");


    struct timeval att_tv1, att_tv2;
    struct timeval imu_tv1, imu_tv2;
    struct timeval manual_tv1, manual_tv2;
    struct timeval gps_tv1, gps_tv2;
    struct timeval sys_tv1, sys_tv2;
    struct timeval rc_tv1, rc_tv2;

    int cnt = 0;
    // clock_t t_prev;
    // clock_t t_curr;
    // t_prev = clock();

    // print_current_time();
    gettimeofday(&att_tv1, NULL);
    imu_tv1 = att_tv1;
    manual_tv1 = att_tv1;
    gps_tv1 = att_tv1;
    sys_tv1 = att_tv1;
    rc_tv1 = att_tv1;

    while(1)
        {
            read_message();
            // cnt++;

            // if (cnt == 200)
            //    {
            //       printf("...\n");
            //      cnt = 0;
            //  }

            if (att_cnt == 200)
                {
                    // t_curr = clock();
                    gettimeofday(&att_tv2, NULL);
                    // printf("%d frames takes %fms\n", cnt, (double)(t_curr-t_prev) * 1000.0 /CLOCKS_PER_SEC);

                    printf ("%d att time = %f ms\n", att_cnt,
                            (double) (att_tv2.tv_usec - att_tv1.tv_usec) / 1000 +
                            (double) (att_tv2.tv_sec - att_tv1.tv_sec) * 1000);
                    gettimeofday(&att_tv1, NULL);

                    // print_current_time();
                    att_cnt = 0;
                    // t_prev = t_curr;
                }

            if (imu_cnt == 200)
                {
                    gettimeofday(&imu_tv2, NULL);
                    printf("\t%d imu time = %f ms\n", imu_cnt,
                           (double) (imu_tv2.tv_usec - imu_tv1.tv_usec) / 1000 +
                           (double) (imu_tv2.tv_sec - imu_tv1.tv_sec) * 1000);
                    gettimeofday(&imu_tv1, NULL);
                    imu_cnt = 0;
                }

            if (manual_cnt == 200)
                {
                    gettimeofday(&manual_tv2, NULL);
                    printf("\t\t%d manual time = %f ms\n", manual_cnt,
                           (double) (manual_tv2.tv_usec - manual_tv1.tv_usec) / 1000 +
                           (double) (manual_tv2.tv_sec - manual_tv1.tv_sec) * 1000);
                    gettimeofday(&manual_tv1, NULL);
                    manual_cnt = 0;

                }

            if (rc_cnt == 200)
                {
                    gettimeofday(&rc_tv2, NULL);
                    printf("\t\t\t%d rc time = %f ms\n", rc_cnt,
                           (double) (rc_tv2.tv_usec - rc_tv2.tv_usec) / 1000 +
                           (double) (rc_tv2.tv_sec - rc_tv1.tv_sec) * 1000);
                    gettimeofday(&rc_tv1, NULL);
                    rc_cnt = 0;
                }

            if (sys_cnt == 50)
                {
                    gettimeofday(&sys_tv2, NULL);
                    printf("\t\t\t\t%d sys time = %f ms\n", sys_cnt,
                           (double) (sys_tv2.tv_usec - sys_tv2.tv_usec) / 1000 +
                           (double) (sys_tv2.tv_sec - sys_tv1.tv_sec) * 1000);
                    gettimeofday(&sys_tv1, NULL);
                    sys_cnt = 0;
                }

            if (gps_cnt == 50)
                {
                    gettimeofday(&gps_tv2, NULL);
                    printf("\t\t\t\t\t%d gps time = %f ms\n", gps_cnt,
                           (double) (gps_tv2.tv_usec - gps_tv2.tv_usec) / 1000 +
                           (double) (gps_tv2.tv_sec - gps_tv1.tv_sec) * 1000);
                    gettimeofday(&gps_tv1, NULL);
                    gps_cnt = 0;
                }

        }

    printf("\n");


    // --------------------------------------------------------------------------
    //   START OFFBOARD MODE
    // --------------------------------------------------------------------------

    printf("Start Off-Board Mode\n");

    // send an initial command
    write_message(0.0, 0.0, 0.0, 0.0);

    // now start the offboard mode
    start_offboard();


    // --------------------------------------------------------------------------
    //   STREAM COMMANDS
    // --------------------------------------------------------------------------

    // Pixhawk needs to see off-board commands at minimum 2Hz, otherwise it
    // will go into fail safe mode (and probably descend)

    // Setpoint Command
    float sp_x   =  0.0f;
    float sp_y   =  0.0f;
    float sp_z   = -1.0f; // Height above take-off point (z points down)
    float sp_yaw =  0.0f;

    printf("Write Off-Board Commands\n");

    // Start Streaming
    while( CMD_STREAM_FLAG )
        {
            // Stream at 4 Hz
            usleep(250000);

            // Write the setpoint message
            write_message(sp_x, sp_y, sp_z, sp_yaw);

            // this loop exits with Ctrl-C
        }


    // --------------------------------------------------------------------------
    //   STOP OFFBOARD MODE
    // --------------------------------------------------------------------------

    printf("Stop Off-Board Mode\n");

    stop_offboard();

    printf("\n");


    // --------------------------------------------------------------------------
    //   CLOSE PORT
    // --------------------------------------------------------------------------

    printf("CLOSE PORT\n");

    close_serial();

    printf("\n");

    return 0;
}


// ------------------------------------------------------------------------------
//   Read Message
// ------------------------------------------------------------------------------
int
read_message()
{

    bool received = false;      // receive only one message

    while (!received)
        {
            mavlink_message_t message;

            if( read_serial(message) )
                {
                    switch (message.msgid)
                        {
                        case MAVLINK_MSG_ID_HEARTBEAT:
                            {
                                mavlink_heartbeat_t heartbeat;
                                mavlink_msg_heartbeat_decode(&message, &heartbeat);

                                // printf("Got heartbeat.\n");
                                // printf("\t custom mode: %u\n", heartbeat.custom_mode);
                                // printf("\t base_mode: %u\n", heartbeat.base_mode);
                                // printf("\t system_status: %u\n", heartbeat.system_status);

                                received = true;

                                break;
                            }
                        case MAVLINK_MSG_ID_ATTITUDE:
                            {
                                // mavlink_attitude_t attitude;
                                // mavlink_msg_attitude_decode(&message, &attitude);
                                // printf("Got message ATTITUDE\n");
                                // printf("\t time: %u\n", attitude.time_boot_ms);
                                // printf("\t roll: %f\n", attitude.roll);
                                // printf("\t pitch: %f\n", attitude.pitch);
                                // printf("\t yaw: %f\n", attitude.yaw);
                                att_cnt++;
                                received = true;
                                break;
                            }
                        case MAVLINK_MSG_ID_RC_CHANNELS:
                            {

                                mavlink_rc_channels_t rc;
                                mavlink_msg_rc_channels_decode(&message, &rc);

                                //if (rc.rssi != 0 )
                                //  {
                                //     printf("Got RC\n");
                                //    printf("\t time (s): %u \n", rc.time_boot_ms/1000);
                                //  printf("\t chan1(roll): %u \n", rc.chan1_raw);
                                //  printf("\t chan2(pitch): %u \n", rc.chan2_raw);
                                //  printf("\t chan3(thrust): %u \n", rc.chan3_raw);
                                //  printf("\t chan4(yaw): %u \n", rc.chan4_raw);
                                //}

                                rc_cnt++;
                                received = true;
                                break;
                            }
                        case MAVLINK_MSG_ID_MANUAL_CONTROL:
                            {
                                mavlink_manual_control_t manual_control;
                                mavlink_msg_manual_control_decode(&message, &manual_control);

                                unsigned int b = manual_control.buttons;
                                printf("mode: %u\n", b);
                                printf("\tmode switch: %u\n", b & 0x0003);
                                printf("\treturn switch: %u\n", (b & 0x000B)>>2);
                                printf("\tposctl switch: %u\n", (b & 0x0030)>>4);
                                printf("\tloiter switch: %u\n", (b & 0x00B0)>>6);
                                printf("\tacro switch: %u\n", (b & 0x0300)>>8);
                                printf("\toffboard switch: %u\n", (b & 0xB000)>>10);

                                // printf("Got manual control\n");
                                // printf(" x: %d \n", manual_control.x);
                                // printf(" y: %d \n", manual_control.y);
                                // printf(" z: %d \n", manual_control.z);
                                // printf(" r: %d \n", manual_control.r);

                                manual_cnt++;
                                received = true;
                                break;
                            }
                        case MAVLINK_MSG_ID_HIGHRES_IMU:
                            {
                                // Decode Message
                                mavlink_highres_imu_t imu;
                                mavlink_msg_highres_imu_decode(&message, &imu);

                                // Do something with the message
                                // printf("Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
                                // printf("    time: %llu\n", imu.time_usec);
                                // printf("    acc  (NED):\t% f\t% f\t% f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
                                // printf("    gyro (NED):\t% f\t% f\t% f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
                                // printf("    mag  (NED):\t% f\t% f\t% f (Ga)\n"   , imu.xmag , imu.ymag , imu.zmag );
                                // printf("    baro: \t %f (mBar)\n"  , imu.abs_pressure);
                                // printf("    altitude: \t %f (m)\n" , imu.pressure_alt);
                                // printf("    temperature: \t %f C\n", imu.temperature );

                                imu_cnt++;
                                received = true;
                                break;
                            }
                        case MAVLINK_MSG_ID_GPS_RAW_INT:
                            {
                                mavlink_gps_raw_int_t gps;
                                mavlink_msg_gps_raw_int_decode(&message, &gps);
                                gps_cnt++;
                                received = true;
                                break;
                            }
                        case MAVLINK_MSG_ID_SYS_STATUS:
                            {
                                mavlink_sys_status_t sys;
                                mavlink_msg_sys_status_decode(&message, &sys);
                                sys_cnt++;
                                received = true;
                                break;
                            }
                        case MAVLINK_MSG_ID_COMMAND_LONG:
                            {
                                mavlink_command_long_t command;
                                mavlink_msg_command_long_decode(&message, &command);
                                printf("get command\n");
                                received = true;
                                break;}
                        default:
                            break;
                        }
                }
        }

    return 0;
}


// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
write_message(float x, float y, float z, float yaw)
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------
    mavlink_set_position_target_local_ned_t sp;
    sp.time_boot_ms     = 0;
    sp.type_mask        = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
    sp.target_system    = sysid;
    sp.target_component = autopilot_compid;
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
    sp.x = x;
    sp.y = y;
    sp.z = z;
    sp.yaw = yaw;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;
    mavlink_msg_set_position_target_local_ned_encode(sysid, compid, &message, &sp);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------
    int len = write_serial(message);
    printf("Sent buffer of length %i\n",len);

    // Done!
    return 0;
}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
/**
 * throws EXIT_FAILURE if could not open the port
 */
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

    const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";
    int i;

    // Read input arguments
    for (i = 1; i < argc; i++) { // argv[0] is "mavlink"

        // Help
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("%s\n",commandline_usage);
            throw EXIT_FAILURE;
        }

        // UART device ID
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
            if (argc > i + 1) {
                uart_name = argv[i + 1];

            } else {
                printf("%s\n",commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Baud rate
        if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
            if (argc > i + 1) {
                baudrate = atoi(argv[i + 1]);

            } else {
                printf("%s\n",commandline_usage);
                throw EXIT_FAILURE;
            }
        }

    }
    // end: for each input argument

    // Done!
    return;
}


// ------------------------------------------------------------------------------
//    Handle CTRL-C terminate commands from the terminal
// ------------------------------------------------------------------------------
void
quit_handler(int sig)
{
    printf("Terminating at user's request\n");
    CMD_STREAM_FLAG = 0;
}


// ------------------------------------------------------------------------------
//    Main
// ------------------------------------------------------------------------------
int
main (int argc, char **argv)
{

    try
        {
            return top( argc , argv );
        }
    catch ( int failure )
        {
            return failure;
        }

}


