#include "ros/ros.h"
#include "std_msgs/String.h"
#include "Definitions.h"
#include "epos_tutorial/VelCommand.h"
#include "epos_tutorial/realVel.h"
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

void  PrintSettings();
int   OpenDevice(unsigned int* p_pErrorCode);
void  SetDefaultParameters();

#ifndef MMC_SUCCESS
#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
#define MMC_MAX_LOG_MSG_SIZE 512
#endif

void SetDefaultParameters()
{
    //USB
//    g_usNodeId = 1;
    g_deviceName = "EPOS2";
    g_protocolStackName = "MAXON SERIAL V2";
    g_interfaceName = "USB";
    g_portName = "USB0";
    g_baudrate = 1000000;
}

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
//    ss << "default settings:" << endl;
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

bool DemoProfileVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, long vel_d)
{
    int lResult = MMC_SUCCESS;

    list<long> velocityList;

    velocityList.push_back(vel_d);

    for(list<long>::iterator it = velocityList.begin(); it !=velocityList.end(); it++)
    {
        long targetvelocity = (*it);
        ROS_INFO("move with target velocity : %ld rpm, node = %d", targetvelocity, p_usNodeId);

        if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, targetvelocity, &p_rlErrorCode) == 0)
        {
            lResult = MMC_FAILED;
            ROS_INFO("VCS_MoveWithVelocity failed");
            break;
        }

    }

    return lResult;
}

int PrepareDemo(unsigned short NodeId, unsigned int* p_pErrorCode)
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

bool command(epos_tutorial::VelCommand::Request &req, epos_tutorial::VelCommand::Response &res)
{
    int lResult = MMC_FAILED;
    unsigned int lErrorCode = 0;

    res.setVel1 = req.Vel1;
    res.setVel2 = req.Vel2;
    res.setVel3 = req.Vel3;
    res.setVel4 = req.Vel4;

    int64_t setVel[] = {res.setVel1, res.setVel2, res.setVel3, res.setVel4};
    ROS_INFO("Desired velocity (rpm) = %ld, %ld, %ld, %ld", (long int)res.setVel1, (long int)res.setVel2, (long int)res.setVel3, (long int)res.setVel4);

    for (int i=0; i<Id_length;i++)
    {
        if((lResult = DemoProfileVelocityMode(g_pKeyHandle, g_usNodeId[i], lErrorCode, setVel[i])!=MMC_SUCCESS))
        {
            ROS_INFO("Velocity Control Failed node: %d", g_usNodeId[i]);
        }
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_con_server");
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
        if((lResult = PrepareDemo(g_usNodeId[i], &ulErrorCode))!=MMC_SUCCESS)
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

    ros::ServiceServer service = n.advertiseService("vel_con", command);
    ROS_INFO("Ready to control velocity");
//    ros::spin();

    ros::Publisher measure_pub = n.advertise<epos_tutorial::realVel>("measure", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        epos_tutorial::realVel msg;

        for (int i=0; i<Id_length;i++)
        {
            VCS_GetVelocityIsAveraged(g_pKeyHandle, g_usNodeId[i], &vel[i], &ulErrorCode);
            msg.realVel[i] = vel[i];
        }

        measure_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}