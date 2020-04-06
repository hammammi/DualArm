#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "epos_tutorial/realVel.h"
//#include "mobile_control/motorMsg.h"
#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include <iostream>

#include <net/if.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <xenomai/init.h>

#include "std_msgs/Int32.h"

#define CANID_DELIM '#'
#define DATA_SEPERATOR '.'

typedef void* HANDLE;
typedef int BOOL;

using namespace std;

RT_TASK measure_task;

void* g_pKeyHandle = 0;
int vel = 0;
unsigned short g_usNodeId[] = {1}; //{1,2,3,4};
int Id_length = sizeof(g_usNodeId)/sizeof(*g_usNodeId);

// CAN Parameters
int s;
struct can_frame frame;
string data;
string vel_desired_hex;
int setVel=0;
string Control_Enable = "0F00";
unsigned short COB_ID_Rx[4] = {0x221, 0x321, 0x421, 0x521};
unsigned short COB_ID_Tx[4] = {0x1A1, 0x2A1, 0x3A1, 0x4A1};


unsigned char asc2nibble(char c) {

    if ((c >= '0') && (c <= '9'))
        return c - '0';

    if ((c >= 'A') && (c <= 'F'))
        return c - 'A' + 10;

    if ((c >= 'a') && (c <= 'f'))
        return c - 'a' + 10;

    return 16; /* error */
}

int hexstring2data(char *arg, unsigned char *data, int maxdlen) {

    int len = strlen(arg);
    int i;
    unsigned char tmp;

    if (!len || len%2 || len > maxdlen*2)
        return 1;

    memset(data, 0, maxdlen);

    for (i=0; i < len/2; i++) {

        tmp = asc2nibble(*(arg+(2*i)));
        if (tmp > 0x0F)
            return 1;

        data[i] = (tmp << 4);

        tmp = asc2nibble(*(arg+(2*i)+1));
        if (tmp > 0x0F)
            return 1;

        data[i] |= tmp;
    }

    return 0;
}

int parse_canframe(char *cs, struct canfd_frame *cf) {
    /* documentation see lib.h */

    int i, idx, dlen, len;
    int maxdlen = CAN_MAX_DLEN;
    int ret = CAN_MTU;
    unsigned char tmp;

    len = strlen(cs);
    //printf("'%s' len %d\n", cs, len);

    memset(cf, 0, sizeof(*cf)); /* init CAN FD frame, e.g. LEN = 0 */

    if (len < 4)
        return 0;

    if (cs[3] == CANID_DELIM) { /* 3 digits */

        idx = 4;
        for (i=0; i<3; i++){
            if ((tmp = asc2nibble(cs[i])) > 0x0F)
                return 0;
            cf->can_id |= (tmp << (2-i)*4);
        }

    } else if (cs[8] == CANID_DELIM) { /* 8 digits */

        idx = 9;
        for (i=0; i<8; i++){
            if ((tmp = asc2nibble(cs[i])) > 0x0F)
                return 0;
            cf->can_id |= (tmp << (7-i)*4);
        }
        if (!(cf->can_id & CAN_ERR_FLAG)) /* 8 digits but no errorframe?  */
            cf->can_id |= CAN_EFF_FLAG;   /* then it is an extended frame */

    } else
        return 0;

    if((cs[idx] == 'R') || (cs[idx] == 'r')){ /* RTR frame */
        cf->can_id |= CAN_RTR_FLAG;

        /* check for optional DLC value for CAN 2.0B frames */
        if(cs[++idx] && (tmp = asc2nibble(cs[idx])) <= CAN_MAX_DLC)
            cf->len = tmp;

        return ret;
    }

    if (cs[idx] == CANID_DELIM) { /* CAN FD frame escape char '##' */

        maxdlen = CANFD_MAX_DLEN;
        ret = CANFD_MTU;

        /* CAN FD frame <canid>##<flags><data>* */
        if ((tmp = asc2nibble(cs[idx+1])) > 0x0F)
            return 0;

        cf->flags = tmp;
        idx += 2;
    }

    for (i=0, dlen=0; i < maxdlen; i++){

        if(cs[idx] == DATA_SEPERATOR) /* skip (optional) separator */
            idx++;

        if(idx >= len) /* end of string => end of data */
            break;

        if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
            return 0;
        cf->data[i] = (tmp << 4);
        if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
            return 0;
        cf->data[i] |= tmp;
        dlen++;
    }
    cf->len = dlen;

    return ret;
}

void LogInfo(string message)
{
    cout << message << endl;
}

template <typename T>
std::string dec_to_hex(T dec, int NbofByte){
    std::stringstream stream_HL;
    string s, s_LH;
    stream_HL << std::setfill ('0') << std::setw(sizeof(T)*2) <<std::hex << dec;
    s = stream_HL.str();
    for (int i=0; i<NbofByte; i++){
        s_LH.append(s.substr(2*(NbofByte-1-i),2));
    }
    return s_LH;
}



int hexarray_to_int(unsigned char *buffer, int length){
    int hextoint = 0;
    for (int i=0; i<length; i++)
    {
        hextoint += (buffer[i] << 8*i);
    }
    return hextoint;
}

std::string stringappend(string a, string b)
{
    string s;
    s.append(a);
    s.append(b);
    return s;
}


void commandCallback(const std_msgs::Int32::ConstPtr& msg)
{

    setVel = msg->data;

// Convert dec to hex
    vel_desired_hex = dec_to_hex(setVel, 4);
    // Send Profile Velocity control command to EPOS
    frame.can_id = COB_ID_Rx[3];
    frame.can_dlc = 6;
    data = stringappend(Control_Enable, vel_desired_hex);
    hexstring2data((char *) data.c_str(), frame.data, 8);
    write(s, &frame, sizeof(struct can_frame));
    ros::Duration(0.00001).sleep();


}

void measure_run(void *arg)
{

    int argc;
    char** argv;

    ros::init(argc, argv, "controller_pdo_xeno");
    ros::NodeHandle n;
    string rtr;
    unsigned char buffer[4];

    // SocketCAN Initialize
    struct sockaddr_can addr;
    //struct can_frame frame;
    struct canfd_frame frame_fd;
    int required_mtu;
    struct ifreq ifr;

    struct iovec iov;
    struct msghdr canmsg;
    char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3*sizeof(struct timespec) + sizeof(__u32))];
    struct canfd_frame frame_get;

    const char *ifname = "slcan0";

    if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return -1;
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

    if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -2;
    }

    // Initialize NMT Services
    LogInfo("Initialize NMT Services");
    // Reset Communication
    frame.can_id  = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x81;
    frame.data[1] = 0x00;
    write(s, &frame, sizeof(struct can_frame));
    LogInfo("Reset Communication");
    sleep(2);

    // Start Remote Node
    frame.can_id  = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x01;
    frame.data[1] = 0x00;
    write(s, &frame, sizeof(struct can_frame));
    LogInfo("Start Remote Node");
    sleep(1);

    // Velocity Control mode Initialize
    frame.can_id  = COB_ID_Rx[1];
    frame.can_dlc = 3;
    frame.data[0] = 0x00;
    frame.data[1] = 0x00;
    frame.data[2] = 0x03;

    write(s, &frame, sizeof(struct can_frame));
    LogInfo("Velocitiy mode initialized");
    sleep(1);

    // Shutdown Controlword
    frame.can_id  = COB_ID_Rx[0];
    frame.can_dlc = 2;
    frame.data[0] = 0x06;
    frame.data[1] = 0x00;
    write(s, &frame, sizeof(struct can_frame));

    LogInfo("Shutdown controlword");
    sleep(1);

    // Enable Controlword
    frame.can_id  = COB_ID_Rx[0];
    frame.can_dlc = 2;
    frame.data[0] = 0x0F;
    frame.data[1] = 0x00;
    write(s, &frame, sizeof(struct can_frame));

    LogInfo("Enable controlword");
    sleep(1);

    ros::Subscriber sub = n.subscribe("input_msg", 1, commandCallback);
    ros::Publisher measure_pub = n.advertise<std_msgs::Int32>("measure", 1);
    ros::Rate loop_rate(100);

    iov.iov_base = &frame_get;
    canmsg.msg_name = &addr;
    canmsg.msg_iov = &iov;
    canmsg.msg_iovlen = 1;
    canmsg.msg_control = &ctrlmsg;

    iov.iov_len = sizeof(frame_get);
    canmsg.msg_namelen = sizeof(addr);
    canmsg.msg_controllen = sizeof(ctrlmsg);
    canmsg.msg_flags = 0;
    sleep(1);

    int nnbytes = 0;
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 50;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

    for (int i=0; i<3; i++)
    {
        recvmsg(s, &canmsg, 0);
    }

    std_msgs::Int32 msg;


    rt_task_set_periodic(NULL, TM_NOW, 1e7)  //tick : 1 nanosec?  100Hz

    while (run)
    {
        rt_task_wait_period(NULL);
        frame.can_id  = COB_ID_Tx[1] | CAN_RTR_FLAG;
        write(s, &frame, sizeof(struct can_frame));

        nnbytes = recvmsg(s,&canmsg, 0);
        msg.data = hexarray_to_int(frame_get.data,4);

        measure_pub.publish(msg);

    }

    // Reset Remote Node
    frame.can_id  = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x81;
    frame.data[1] = 0x00;
    write(s, &frame, sizeof(struct can_frame));
    LogInfo("Reset Remote Node");
    sleep(2);

    // Stop Remote Node
    frame.can_id  = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x02;
    frame.data[1] = 0x00;
    write(s, &frame, sizeof(struct can_frame));
    LogInfo("Stop Remote Node");

}

int main(int argc, char **argv) {

  cpu_set_t cpu_set_measure;
  CPU_ZERO(&cpu_set_measure);
  CPU_SET(0, &cpu_set_measure);

  rt_task_create(&measure_task, 0, 90, 0);
  rt_task_set_affinity(&measure_task, &cpu_set_measure);

  rt_task_start(&measure_task, &measure_run, NULL)

  while (run) {
    usleep(1000000);
  }

  return 0;

}
