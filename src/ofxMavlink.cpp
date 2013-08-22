#include "ofxMavlink.h"
#include "ofMain.h"
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */


ofxMavlink::ofxMavlink()
{
    msgcount = 0;
    bDebug = false;
    sysId = 2;      //The system ID reflects the current aircraft. MAVLink can be used with multiple aircraft simultaneously
    compId = 1;   //The component ID reflects the onboard component, e.g. the IMU, the autopilot or the onboard computer.
                    //This allows to use MAVLink both for onboard-communication and for off-board telemetry.
    targetId = 1;   // The target id, the vehicle you are trying to control

}

void ofxMavlink::close_port(int fd)
{
	close(fd);
}

int ofxMavlink::open_port(const char* port)
{

	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		/* Could not open the port. */
		return(-1);
	}
	else
	{
		/* Set to blocking mode */
		//fcntl(fd, F_SETFL, 0);
		/* set to non-blocking mode */
		fcntl(fd, F_SETFL, FNDELAY);
	}

	return (fd);
}


bool ofxMavlink::setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
	//struct termios options;

	struct termios  config;
	if(!isatty(fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
		return false;
	}
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}
	//
	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
	                    INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	                    ONOCR | OFILL | OPOST);

	#ifdef OLCUC
  		config.c_oflag &= ~OLCUC;
	#endif

  	#ifdef ONOEOT
  		config.c_oflag &= ~ONOEOT;
  	#endif


	//
	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	//
	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	//
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;

	/* turn off hardware flow control */
	config.c_cflag &= ~CRTSCTS;

	//
	// One input byte is enough to return from read()
	// Inter-character timer off
	//
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10; // was 0

	// Get the current options for the port
	//tcgetattr(fd, &options);

	switch (baud)
	{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;

		// These two non-standard (by the 70'ties ) rates are fully supported on
		// current Debian and Mac OS versions (tested since 2010).
		case 460800:
			if (cfsetispeed(&config, 460800) < 0 || cfsetospeed(&config, 460800) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 921600:
			if (cfsetispeed(&config, 921600) < 0 || cfsetospeed(&config, 921600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
			return false;

			break;
	}

	//
	// Finally, apply the configuration
	//
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}
	return true;
}


bool ofxMavlink::connect(string uart_name) {
	//char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 57600;
	bool bSuccess = false;

    ofLogNotice("ofxMavlink") << "Trying to connect to " << uart_name;

	fd = open_port(uart_name.c_str());
	if (fd == -1)
	{
		ofLogError("ofxMavlink") << "failure, could not open port.";
		//exit(EXIT_FAILURE);
	}
	else
	{
		ofLogNotice("ofxMavlink") << "success.";
	}
	ofLogNotice("ofxMavlink") << "Trying to configure " << uart_name;

	bool setup = setup_port(fd, baudrate, 8, 1, false, false);
	if (!setup)
	{
		ofLogError("ofxMavlink") << "failure, could not configure port.";
		//exit(EXIT_FAILURE);
	}
	else
	{
		ofLogNotice("ofxMavlink") << "success.";
	}

	if (fd == -1 || fd == 0)
	{
	    ofLogNotice("ofxMavlink") << "Connection attempt to port " << uart_name <<  " with " << baudrate << " baud, 8N1 failed, exiting.";
		//std::exit(1);
	}
	else
	{
	    bSuccess = true;
		ofLogNotice("ofxMavlink") << "Connected to" << uart_name <<  " with " << baudrate << " baud, 8 data bits, no parity, 1 stop bit (8N1)";
	}

	return bSuccess;
}

void ofxMavlink::disconnect()
{
    ofLogNotice("ofxMavlink") << "closing Mavlink serial connection...";
	close_port(fd);
}

void ofxMavlink::sendHeartbeat() {
    // Initialize the required buffers
    mavlink_message_t message;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Define the system type, in this case an airplane
    int system_type = MAV_TYPE_GCS;
    int autopilot_type = MAV_AUTOPILOT_GENERIC;

    // Pack the message
    mavlink_msg_heartbeat_pack(sysId, compId, &message, system_type, autopilot_type, 0, 0, MAV_STATE_ACTIVE);
    // mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC)

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &message);

    /* write packet via serial link */
    int returnval = write(fd, buf, len);
    if(returnval < 0) ofLogError("ofxMavlink") << "Error writing to serial port";

    /* wait until all data has been written */
    tcdrain(fd);

}

void ofxMavlink::requestDataStreams() {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_request_data_stream_t packet;
    mavlink_message_t message;

    packet.target_component = compId;
    packet.target_system = targetId;
    packet.req_message_rate = 20;
    packet.req_stream_id = MAV_DATA_STREAM_ALL;
    packet.start_stop = 1;

    mavlink_msg_request_data_stream_encode(sysId, compId, &message, &packet);
    unsigned len = mavlink_msg_to_send_buffer(buf, &message);

    /* write packet via serial link */
    int returnval = write(fd, buf, len);
    if(returnval < 0) ofLogError("ofxMavlink") << "Error writing to serial port";

    /* wait until all data has been written */
    tcdrain(fd);
}

void ofxMavlink::sendMissionWaypoint(double _latitude, double _longitude)
{
    cout << "Send mission waypoint" << endl;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_mission_item_t mission;
    mavlink_message_t message;

    mission.autocontinue = 0;
    mission.current = 2; //2 for guided mode
    mission.param1 = 0;
    mission.param2 = 0;
    mission.param3 = 0;
    mission.param4 = 0;
    mission.frame = 0;// MAV_FRAME_GLOBAL;
    mission.command =  MAV_CMD_NAV_WAYPOINT;
    mission.seq = 0; // don't read out the sequence number of the waypoint class
    mission.x = _latitude;   //latitude, e.g. -37.786818
    mission.y = _longitude; //longitude e.g. 144.960688
    mission.z = 4;  //altitude
    mission.target_system = 1;
    mission.target_component = MAV_COMP_ID_MISSIONPLANNER;

    mavlink_msg_mission_item_encode(sysId, compId, &message, &mission);
    unsigned len = mavlink_msg_to_send_buffer(buf, &message);

    /* write packet via serial link */
    int returnval = write(fd, buf, len);
    if(returnval < 0) ofLogError("ofxMavlink") << "Error writing to serial port";

    /* wait until all data has been written */
    tcdrain(fd);
}

// Auto Pilot modes
// ----------------
/*
#define STABILIZE 0                     // hold level position
#define ACRO 1                          // rate control
#define ALT_HOLD 2                      // AUTO control
#define AUTO 3                          // AUTO control
#define GUIDED 4                        // AUTO control
#define LOITER 5                        // Hold a single location
#define RTL 6                           // AUTO control
#define CIRCLE 7                        // AUTO control
#define POSITION 8                      // AUTO control
#define LAND 9                          // AUTO control
#define OF_LOITER 10                    // Hold a single location using optical flow
                                        // sensor
#define TOY_A 11                        // THOR Enum for Toy mode
#define TOY_M 12                        // THOR Enum for Toy mode
#define NUM_MODES 13
*/

void ofxMavlink::setAutoPilotMode(int _mode) {

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_set_mode_t mode;

    // Ardupilot/ArduCopter use custome mode only, but base mode MAV_MODE_FLAG_CUSTOM_MODE_ENABLED needs to be set
    mode.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    mode.custom_mode = _mode;
    mode.target_system = targetId;

    mavlink_message_t message;

    mavlink_msg_set_mode_encode(sysId, compId, &message, &mode);
    unsigned len = mavlink_msg_to_send_buffer(buf, &message);

    /* write packet via serial link */
    int returnval = write(fd, buf, len);
    if(returnval < 0) ofLogError("ofxMavlink") << "Error writing to serial port";

    /* wait until all data has been written */
    tcdrain(fd);
}


void ofxMavlink::sendMissionTakeoff()
{
    //cout << "Send mission takeoff" << endl;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_mission_item_t mission;
    mavlink_message_t message;

    mission.autocontinue = 0;
    mission.current = 0; //2 for guided mode
    mission.param1 = 0;
    mission.param2 = 0;
    mission.param3 = 0;
    mission.param4 = 0;
    mission.frame = 0;// MAV_FRAME_GLOBAL;
    mission.command =  MAV_CMD_NAV_TAKEOFF;
    mission.seq = 0; // don't read out the sequence number of the waypoint class
    mission.x = 0; //latitude
    mission.y = 0; //longitude
    mission.z = 3;  //altitude
    mission.target_system = targetId;
    mission.target_component = compId; //MAV_COMP_ID_MISSIONPLANNER;

    mavlink_msg_mission_item_encode(sysId, compId, &message, &mission);
    unsigned len = mavlink_msg_to_send_buffer(buf, &message);

    /* write packet via serial link */
    int returnval = write(fd, buf, len);
    if(returnval < 0) ofLogError("ofxMavlink") << "Error writing to serial port";

    /* wait until all data has been written */
    tcdrain(fd);
}

void ofxMavlink::sendMissionLand()
{
    //cout << "Send mission land" << endl;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_mission_item_t mission;
    mavlink_message_t message;

    mission.autocontinue = 0;
    mission.current = 2; //2 for guided mode
    mission.param1 = 0;
    mission.param2 = 0;
    mission.param3 = 0;
    mission.param4 = 0;
    mission.frame = 0;// MAV_FRAME_GLOBAL;
    mission.command =  MAV_CMD_NAV_LAND;
    mission.seq = 0; // don't read out the sequence number of the waypoint class
    mission.x = 0; //latitude
    mission.y = 0; //longitude
    mission.z = 3;  //altitude
    mission.target_system = 1;
    mission.target_component = MAV_COMP_ID_MISSIONPLANNER;

    mavlink_msg_mission_item_encode(sysId, compId, &message, &mission);
    unsigned len = mavlink_msg_to_send_buffer(buf, &message);

    /* write packet via serial link */
    int returnval = write(fd, buf, len);
    if(returnval < 0) ofLogError("ofxMavlink") << "Error writing to serial port";

    /* wait until all data has been written */
    tcdrain(fd);
}


void ofxMavlink::sendArmCommand(ArmCommand arm)
{

	char buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t message;

    mavlink_command_long_t cmd;
    cmd.command = (uint16_t) MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.confirmation = 0;
    cmd.param1 = arm;
    cmd.param2 = 0.0f;
    cmd.param3 = 0.0f;
    cmd.param4 = 0.0f;
    cmd.param5 = 0.0f;
    cmd.param6 = 0.0f;
    cmd.param7 = 0.0f;
    cmd.target_component = MAV_COMP_ID_SYSTEM_CONTROL;
    cmd.target_system = targetId;
    mavlink_msg_command_long_encode(sysId, compId, &message, &cmd);
    cout << "Send ARM request to drone" << endl;

    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

    /* write packet via serial link */
    int returnval = write(fd, buf, len);
    if(returnval < 0) ofLogError("ofxMavlink") << "Error writing to serial port";

    /* wait until all data has been written */
    tcdrain(fd);

}

/* NB: this should be threaded!! */
void ofxMavlink::readMessage() {

	uint8_t cp;
    mavlink_message_t message;
    mavlink_status_t status;
    uint8_t msgReceived = false;

//    static unsigned int imu_receive_counter = 0;

    if (read(fd, &cp, 1) > 0)
    {
        // Check if a message could be decoded, return the message in case yes
        msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
        if (status.packet_rx_drop_count > 0)
        {
            if(bDebug) ofLogWarning("ofxMavlink") << "ERROR: DROPPED " << status.packet_rx_drop_count <<" PACKETS:" << " " << cp;
        }
    }
    else
    {
        if(bDebug) ofLogNotice("ofxMavlink") << "ERROR: Could not read from fd " << fd;
    }

    // If a message could be decoded, handle it
    if(msgReceived)
    {
        msgcount++;

        if (bDebug)
        {
            fprintf(stderr,"Received serial data: ");
            unsigned int i;
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);
            if (messageLength > MAVLINK_MAX_PACKET_LEN)
            {
                fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
            }
            else
            {
                for (i=0; i<messageLength; i++)
                {
                    unsigned char v=buffer[i];
                    fprintf(stderr,"%02x ", v);
                }
                fprintf(stderr,"\n");
            }
        }

        if(bDebug) ofLogNotice("ofxMavlink") << "Received message from serial with ID #" << message.msgid << " (sys:" << message.sysid << "|comp:" << message.compid << "):\n";


        switch (message.msgid)
        {
            case MAVLINK_MSG_ID_COMMAND_ACK:
            {
                mavlink_command_ack_t ack;
                mavlink_msg_command_ack_decode(&message, &ack);
                switch (ack.result)
                {
                    case MAV_RESULT_ACCEPTED:
                    {
                        cout << "SUCCESS: Executed Command: " << ack.command;
                    }
                        break;
                    case MAV_RESULT_TEMPORARILY_REJECTED:
                    {
                        cout << "FAILURE: Temporarily rejected Command: " << ack.command;
                    }
                        break;
                    case MAV_RESULT_DENIED:
                    {
                        cout << "FAILURE: Denied Command: " << ack.command;
                    }
                        break;
                    case MAV_RESULT_UNSUPPORTED:
                    {
                        cout << "FAILURE: Unsupported Command: " << ack.command;
                    }
                        break;
                    case MAV_RESULT_FAILED:
                    {
                        cout << "FAILURE: Failed Command: " << ack.command;
                    }
                        break;
                    }

                break;
            }
//            case MAVLINK_MSG_ID_COMMAND_ACK:
//            {
//                mavlink_command_ack_t ack;
//                mavlink_msg_command_ack_decode(&message,&ack);
//                cout << "received CMD_ACK  command=" << ack.command << " result=" << ack.result <<  endl;
//
//                if(ack.command == MAV_CMD_COMPONENT_ARM_DISARM) {
//                   cout << "Acknowledge arm/disarm command";
//
//                }
//                if(ack.result == MAV_CMD_ACK_OK) {
//                    cout << "received CMD_ACK_OK" << endl;
//                }
//                break;
//            }
            case MAVLINK_MSG_ID_RAW_IMU:
            {
                mavlink_raw_imu_t imu;
                mavlink_msg_raw_imu_decode(&message, &imu);

                time_usec = imu.time_usec;
                ofLogVerbose("ofxMavlink") << "-- RAW_IMU message received --" << endl;
                ofLogVerbose("ofxMavlink") << "\t time: (us) " << imu.time_usec << endl;
                ofLogVerbose("ofxMavlink") << "\t acc:" << imu.xacc << " " << imu.yacc << " " << imu.zacc << endl;
                ofLogVerbose("ofxMavlink") << "\t gyro: (rad/s)" << imu.xgyro << " " << imu.ygyro << " " << imu.zgyro << endl;
                ofLogVerbose("ofxMavlink") << "\t mag: (Ga)" << imu.xmag << " " << imu.ymag << " " << imu.zmag << endl;
                break;
            }

            case MAVLINK_MSG_ID_GPS_RAW_INT:
            {
                ofLogNotice("ofxMavlink") << "-- GPS RAW INT message received --" << endl;
                mavlink_gps_raw_int_t packet;
                mavlink_msg_gps_raw_int_decode(&message, &packet);

               if (packet.fix_type > 2)
                {
                    latitude = packet.lat/(double)1E7;
                    longitude = packet.lon/(double)1E7;
                    altitude = packet.alt/1000.0;
                    ofLogNotice("ofxMavlink") << "Altitude:" << packet.alt/1000.0 << "latitude:" << (float)packet.lat / 10000000.0 << " longitude:" << (float)packet.lon / 10000000.0 << " ";
                    ofLogNotice("ofxMavlink") << "Satellites visible:" << packet.satellites_visible << " GPS fix:" << packet.fix_type << endl;
                //                if (packet.lat != 0.0) {
                //
                //                    latitude = (float)packet.lat;
                //                    longitude = (float)packet.lon;
                //                    altitude = (float)packet.alt;
                //                    //ModelData.speed = (float)packet.vel / 100.0;
                //                    //ModelData.numSat = packet.satellites_visible;
                //                    gpsfix = packet.fix_type;
                }
                break;
            }
            case MAVLINK_MSG_ID_HEARTBEAT:
            {

                mavlink_heartbeat_t packet;
                mavlink_msg_heartbeat_decode(&message, &packet);
                int droneType = packet.type;
                int autoPilot = packet.autopilot;

//                cout << "base mode:" << packet.base_mode << " custom mode:" << packet.custom_mode << endl;
//                if (packet.base_mode == MAV_MODE_MANUAL_ARMED) {
//                //ModelData.mode = MODEL_MODE_MANUAL;
//                    cout << "Manual Mode" << endl;
//                } else if (packet.base_mode == 128 + 64 + 16) {
//                //ModelData.mode = MODEL_MODE_RTL;
//                    cout << "RTL Mode" << endl;
//                } else if (packet.base_mode == 128 + 16) {
//                //ModelData.mode = MODEL_MODE_POSHOLD;
//                    cout << "Poshold Mode" << endl;
//                } else if (packet.base_mode == 128 + 4) {
//                //ModelData.mode = MODEL_MODE_MISSION;
//                    cout << "Mission Mode" << endl;
//                }
//                if(packet.base_mode == MAV_MODE_STABILIZE_DISARMED)
//                {
//                    cout << "Stablilize disarmed mode" << endl;
//                }

                ofLogWarning("ofxMavlink") << "-- Heartbeat -- sysId:" << message.sysid << "  compId:" << message.compid << " drone type:" << droneType << " autoPilot" <<  autoPilot;
//                if(droneType == MAV_TYPE_QUADROTOR) cout << " Quadrotor"; else cout << droneType;
//                cout << " Autopilot:";
//                if(autoPilot == MAV_AUTOPILOT_ARDUPILOTMEGA) cout << " ArduPilotMega"; else cout << autoPilot;
//                cout << endl;

                if (message.sysid != 0xff) {
                    targetId = message.sysid;
                    compId = message.compid;
                }
                break;
            }

            default:
            break;
        }
    }

}
