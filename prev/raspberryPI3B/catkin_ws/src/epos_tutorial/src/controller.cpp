#include "ros/ros.h"
#include "std_msgs/String.h"
#include "Definitions.h"
#include "epos_tutorial/realVel.h"
//#include "epos_tutorial/DesiredVel.h"
#include "mobile_control/motorMsg.h"
#include <sstream>

typedef void* HANDLE;
typedef int BOOL;

using namespace std;

void* g_pKeyHandle = 0;
int vel[] = {0,0,0,0};
unsigned short g_usNodeId[] = {1,2,3,4};
int Id_length = sizeof(g_usNodeId)/sizeof(*g_usNodeId);
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
int g_baudrate = 0;

#ifndef MMC_SUCCESS
#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
#define MMC_MAX_LOG_MSG_SIZE 512
#endif

void SeparatorLine()
{
    const int lineLength = 65;
    for(int i=0; i<lineLength; i++)
    {
        cout << "-";
    }
    cout << endl;
}

void PrintSettings()
{
    std_msgs::String msg;

    std::stringstream ss;
    ss << endl;

    ss << "number of node      = " << Id_length << endl;
    ss << "device name         = '" << g_deviceName << "'" << endl;
    ss << "protocal stack name = '" << g_protocolStackName << "'" << endl;
    ss << "interface name      = '" << g_interfaceName << "'" << endl;
    ss << "port name           = '" << g_portName << "'"<< endl;
    ss << "baudrate            = " << g_baudrate;

    msg.data = ss.str();
    ROS_INFO("%s",msg.data.c_str());

    SeparatorLine();
}

void SetDefaultParameters()
{
    //USB
    g_deviceName = "EPOS4";
    g_protocolStackName = "MAXON SERIAL V2";
    g_interfaceName = "USB";
    g_portName = "USB0";
    g_baudrate = 1000000;
}

int OpenDevice(unsigned int* p_pErrorCode)
{
    int lResult = MMC_FAILED;

    char* pDeviceName = new char[255];
    char* pProtocolStackName = new char[255];
    char* pInterfaceName = new char[255];
    char* pPortName = new char[255];

    strcpy(pDeviceName, g_deviceName.c_str());
    strcpy(pProtocolStackName, g_protocolStackName.c_str());
    strcpy(pInterfaceName, g_interfaceName.c_str());
    strcpy(pPortName, g_portName.c_str());

    ROS_INFO("Open device...");

    g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

    if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
    {
        unsigned int lBaudrate = 0;
        unsigned int lTimeout = 0;

        if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
        {
            if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0)
            {
                if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
                {
                    if(g_baudrate==(int)lBaudrate)
                    {
                        lResult = MMC_SUCCESS;
                        ROS_INFO("Open device Success");
                    }
                }
            }
        }
    }
    else
    {
        g_pKeyHandle = 0;
    }

    delete []pDeviceName;
    delete []pProtocolStackName;
    delete []pInterfaceName;
    delete []pPortName;

    return lResult;
}

bool ProfileVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, long vel_d)
{
    int lResult = MMC_SUCCESS;

//    ROS_INFO("move with target velocity : %ld rpm, node = %d", vel_d, p_usNodeId);

    if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, vel_d, &p_rlErrorCode) == 0)
    {
        lResult = MMC_FAILED;
        ROS_INFO("VCS_MoveWithVelocity failed");
    }

    return lResult;
}

int PrepareDriver(unsigned short NodeId, unsigned int* p_pErrorCode)
{
    int lResult = MMC_SUCCESS;
    BOOL oIsFault = 0;

    if(VCS_GetFaultState(g_pKeyHandle, NodeId, &oIsFault, p_pErrorCode ) == 0)
    {
//        ROS_INFO("Device (node: %d) is in fault state", NodeId);
        lResult = MMC_FAILED;
    }

    if(lResult==0)
    {
        if(oIsFault)
        {
//            ROS_INFO("Clear fault state (node: %d)", NodeId);
            if(VCS_ClearFault(g_pKeyHandle, NodeId, p_pErrorCode) == 0)
            {
//                ROS_INFO("Clearing is failed")
                lResult = MMC_FAILED;
            }
        }

        if(lResult==0)
        {
            BOOL oIsEnabled = 0;

            if(VCS_GetEnableState(g_pKeyHandle, NodeId, &oIsEnabled, p_pErrorCode) == 0)
            {
                lResult = MMC_FAILED;
            }

            if(lResult==0)
            {
                if(!oIsEnabled)
                {
                    if(VCS_SetEnableState(g_pKeyHandle, NodeId, p_pErrorCode) == 0)
                    {
                        ROS_INFO("Device (node: %d) is enabled", NodeId);
                        lResult = MMC_FAILED;
                    }
                }
            }
        }
    }
    return lResult;
}

void commandCallback(const mobile_control::motorMsg::ConstPtr& msg)
{
        double t_sub1 = ros::Time::now().toSec();

    int lResult = MMC_FAILED;
    unsigned int lErrorCode = 0;

    int64_t setVel[] = {msg->omega1, msg->omega2, msg->omega3, msg->omega4};
    ROS_INFO("Desired velocity (rpm) = %ld, %ld, %ld, %ld", (long int)setVel[0], (long int)setVel[1], (long int)setVel[2], (long int)setVel[3]);
       double t_sub2 = ros::Time::now().toSec();
    for (int i=0; i<Id_length;i++)
    {
        if((lResult = ProfileVelocityMode(g_pKeyHandle, g_usNodeId[i], lErrorCode, setVel[i])!=MMC_SUCCESS))
        {
            ROS_INFO("Velocity Control Failed node: %d", g_usNodeId[i]);
        }
    }
        double t_sub3 = ros::Time::now().toSec();
        ROS_INFO("t_sub31 : %lf t_sub21 = %lf tsub_32 = %lf  ",t_sub3-t_sub1, t_sub2-t_sub1, t_sub3-t_sub2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;

    int lResult = MMC_FAILED;
    unsigned int ulErrorCode = 0;
    SetDefaultParameters();
    PrintSettings();

    if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
    {
        ROS_INFO("OpenDevice Failed");
    }

    for (int i=0; i< Id_length;i++)
    {
        if((lResult = PrepareDriver(g_usNodeId[i], &ulErrorCode))!=MMC_SUCCESS)
        {
            ROS_INFO("PrepareDemo Failed node: %d", g_usNodeId[i]);
            return 0;
        }
    }

    for (int i=0; i<sizeof(g_usNodeId)/sizeof(*g_usNodeId);i++)
    {
        if(VCS_ActivateProfileVelocityMode(g_pKeyHandle, g_usNodeId[i], &ulErrorCode) == 0)
        {
            ROS_INFO("VCS_ActivateProfileVelocityMode node: %d Failed", g_usNodeId[i]);
            lResult = MMC_FAILED;
        }
        VCS_SetVelocityProfile(g_pKeyHandle, g_usNodeId[i], 5000, 5000, &ulErrorCode);
    }

    ROS_INFO("Ready to control velocity");

    ros::Subscriber sub = n.subscribe("input_msg", 1, commandCallback);
//    ros::spin();
    ros::Publisher measure_pub = n.advertise<epos_tutorial::realVel>("measure", 1);
    ros::Rate loop_rate(100);

    double t_pub1;
    double t_pub2;
    double t_sub;

    while (ros::ok())
    {
        epos_tutorial::realVel msg;

        for (int i=0; i<Id_length;i++)
        {
            VCS_GetVelocityIsAveraged(g_pKeyHandle, g_usNodeId[i], &vel[i], &ulErrorCode);
            msg.realVel[i] = vel[i];
        }
     t_pub1=ros::Time::now().toSec();

        measure_pub.publish(msg);
     t_pub2 = ros::Time::now().toSec();
    
    ROS_INFO("t_pub : %lf",t_pub2-t_pub1);

        ros::spinOnce();

        loop_rate.sleep();
    }

//          ros::spin(); 

    return 0;
}

