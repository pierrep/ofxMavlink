#ifndef OFX_MAVLINK_H
#define OFX_MAVLINK_H

#include "ofxMavlink.h"
#include <common/mavlink.h>
#include <string>

class ofxMavlink
{
    public:
        ofxMavlink();

        enum ArmCommand { DISARM = 0, ARM = 1};

        void readMessage();
        void sendArmCommand(ArmCommand cmd);
        void connect(std::string uart_name);
        void disconnect();
        int getMessageCount() {return msgcount;};
        void sendHeartbeat();
        void requestDataStreams();
        void sendMissionTakeoff();
        void sendMissionWaypoint(double _latitude, double _longitude);
        void sendMissionLand();
        void setAutoPilotMode(int mode);
        void setDebug(bool debug) {bDebug = debug;};

        double latitude;
        double longitude;
        double altitude;
        int gpsfix;
        unsigned long time_usec;


    private:

        void close_port(int fd);
        int open_port(const char* port);
        bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);

        int fd; /* File descriptor for the port */
        mavlink_status_t lastStatus;
        int msgcount;

        int sysId;    ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
        int compId;
        int targetId;   /// Target system id
        bool bDebug;    /// control some of the more verbose debug outputs
};

#endif // OFX_MAVLINK_H
