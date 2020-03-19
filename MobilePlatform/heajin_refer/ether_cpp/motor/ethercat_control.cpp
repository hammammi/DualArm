#include "ros/ros.h"
#include "msgpkg/realVal.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <inttypes.h>
#include <unistd.h>
#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <xenomai/init.h>

#include "soem/ethercat.h"
#include "pdo_def.h"
#include "servo_def.h"
#include "ecat_dc.h"

#include <serial/serial.h>

#define EC_TIMEOUTMON 500
#define NUMOFEPOS4_DRIVE	1
#define NSEC_PER_SEC 1000000000
unsigned int cycle_ns = 1000000;

EPOS4_Drive_pt	epos4_drive_pt[NUMOFEPOS4_DRIVE];
Arduino_Serial_pt	serial_pt;
using namespace std;


int started[NUMOFEPOS4_DRIVE]={0}, ServoState=0;
uint8 servo_ready = 0, servo_prestate = 0;

char IOmap[4096];
pthread_t thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

RT_TASK motion_task;
RT_TASK print_task;
RT_TASK ecceck_task;
RT_TASK serial_task;


RTIME now, previous;
long ethercat_time_send, ethercat_time_read = 0;
long ethercat_time = 0, worst_time = 0;
char ecat_ifname[32] = "enp0s31f6";
int run = 1;
int sys_ready = 0;

int recv_fail_cnt = 0;
double gt = 0;

int32_t zeropos[NUMOFEPOS4_DRIVE] = {0};
double sine_amp = 22000, f=0.1, period;

int os;
uint32_t ob;
uint16_t ob2;
uint8_t  ob3;



/* Controller part
float X[3];
float Y[3] = {0,0,0};
//float A_k[3][3] = {{-976.7212, 62.0783, 0.4408},{-136.92, -27.4, 0.151},{-3503.4, 932.45, -61.8426}};
//float B_k[3][2] ={{-0.0039,0.0039},{0.0053,-0.0053},{190,-190}};
//float C_k[3] = {-0.0029, -0.0356,-1.6014};
//float A_k[3][3] = {{-977.2127, -119.3153, -0.9185},{125.0017, -20.7509,0.3930},{2391.9, 639.4047, -72.2466}};
//float B_k[3][2] ={{0.0017,-0.0017},{0.0036,-0.0034},{104.9419,-104.9133}};
//float C_k[3] = {-0.002, -0.1356,-4.5155};
//float A_k[3][3] = {{-961.4278, 6.3895, -16.5237},{-82.6537, -60.5864, 28.6098},{207.4728, 52.045, -69.7277}};
//float B_k[3][2] ={{0,0},{-1.6003,1.6001},{3.7513,-4.0517}};
//float C_k[3] = {0.2567, 80.0958,-204.3057};
float A_k[3][3] = {{-974.4762, -90.3758, -0.5110},{136.4445, -23.4559, 0.1802},{3498.4, 1055.8, -66.0731}};
float B_k[3][2] ={{0.0036,-0.0036},{0.0047,-0.0048},{178.2814,-178.3136}};
float C_k[3] = {0.0015, -0.0450,-2.217};


typedef struct _RK_solver
{
float y1;
float y2;
float y3;
}RK_solver;


float func1(float x1,float x2, float x3,float z1,float z2)
{
	float Value = A_k[0][0]*x1+A_k[0][1]*x2+A_k[0][2]*x3+B_k[0][0]*z1+B_k[0][1]*z2;
	
	return Value;
}

float func2(float x1,float x2, float x3,float z1,float z2)
{
	float Value = A_k[1][0]*x1+A_k[1][1]*x2+A_k[1][2]*x3+B_k[1][0]*z1+B_k[1][1]*z2;
	
	return Value;
}

float func3(float x1,float x2, float x3,float z1,float z2)
{
	float Value = A_k[2][0]*x1+A_k[2][1]*x2+A_k[2][2]*x3+B_k[2][0]*z1+B_k[2][1]*z2;
	
	return Value;
}

RK_solver RK4(float x1,float x2, float x3,float z1,float z2, float t1, float t2)
{
	RK_solver value;
	float k[3][4]; 
	int n = 40.0;
	float dt = (t2-t1)/n;
	float Y[n][3];
	Y[0][0]= x1; 
	Y[0][1]= x2; 
	Y[0][2]= x3; 
	for (int i=0; i<n-1; i++){
		k[0][0] = func1(Y[i][0],Y[i][1],Y[i][2],z1,z2);
		k[0][1] = func1(Y[i][0]+0.5*dt*k[0][0],Y[i][1]+0.5*dt*k[0][0],Y[i][2]+0.5*dt*k[0][0],z1,z2);
		k[0][2] = func1(Y[i][0]+0.5*dt*k[0][1],Y[i][1]+0.5*dt*k[0][1],Y[i][2]+0.5*dt*k[0][1],z1,z2);
		k[0][3] = func1(Y[i][0]+dt*k[0][2],Y[i][1]+dt*k[0][2],Y[i][2]+dt*k[0][2],z1,z2);
		k[1][0] = func2(Y[i][0],Y[i][1],Y[i][2],z1,z2);
		k[1][1] = func2(Y[i][0]+0.5*dt*k[1][0],Y[i][1]+0.5*dt*k[1][0],Y[i][2]+0.5*dt*k[1][0],z1,z2);
		k[1][2] = func2(Y[i][0]+0.5*dt*k[1][1],Y[i][1]+0.5*dt*k[1][1],Y[i][2]+0.5*dt*k[1][1],z1,z2);
		k[1][3] = func2(Y[i][0]+dt*k[1][2],Y[i][1]+dt*k[1][2],Y[i][2]+dt*k[1][2],z1,z2);
		k[2][0] = func3(Y[i][0],Y[i][1],Y[i][2],z1,z2);
		k[2][1] = func3(Y[i][0]+0.5*dt*k[2][0],Y[i][1]+0.5*dt*k[2][0],Y[i][2]+0.5*dt*k[2][0],z1,z2);
		k[2][2] = func3(Y[i][0]+0.5*dt*k[2][1],Y[i][1]+0.5*dt*k[2][1],Y[i][2]+0.5*dt*k[2][1],z1,z2);
		k[2][3] = func3(Y[i][0]+dt*k[2][2],Y[i][1]+dt*k[2][2],Y[i][2]+dt*k[2][2],z1,z2);
		Y[i+1][0] = Y[i][0] + (1.0/6.0)*(k[0][0]+2*k[0][1]+2*k[0][2]+k[0][3])*dt;
		Y[i+1][1] = Y[i][1] + (1.0/6.0)*(k[1][0]+2*k[1][1]+2*k[1][2]+k[1][3])*dt;
		Y[i+1][2] = Y[i][2] + (1.0/6.0)*(k[2][0]+2*k[2][1]+2*k[2][2]+k[2][3])*dt;
	}
	value.y1 = Y[n-1][0];
	value.y2 = Y[n-1][1];
	value.y3 = Y[n-1][2];
	return value;
}

int des_vel_trajectory(float tor,double cnt){

	float dxl_goal_vel;
	float offset;
	des_tor = 0.0;
        RK_solver x;
    x = RK4(Y[0],Y[1],Y[2],tor,(2.0*des_tor)-tor,0.0,hz);
    //ROS_INFO("%f , %f, %f", x.y1, x.y2, x.y3 );
    Y[0] = x.y1;
    Y[1] = x.y2;
    Y[2] = x.y3;
    dxl_goal_vel = C_k[0]*Y[0]+C_k[1]*Y[1]+C_k[2]*Y[2];
    //dxl_goal_vel = dxl_goal_vel;

   // dxl_goal_vel = pid_controller(des_tor-tor);
    //ROS_INFO("%d\n", radtorpm(dxl_goal_vel));

            	
    return radtorpm(dxl_goal_vel);
}


    RK_solver x;
    x = RK4(Y[0],Y[1],Y[2],tor,(2.0*des_tor)-tor,0.0,hz);
    //ROS_INFO("%f , %f, %f", x.y1, x.y2, x.y3 );
    Y[0] = x.y1;
    Y[1] = x.y2;
    Y[2] = x.y3;
    dxl_goal_vel = C_k[0]*Y[0]+C_k[1]*Y[1]+C_k[2]*Y[2];
    //dxl_goal_vel = dxl_goal_vel;

   // dxl_goal_vel = pid_controller(des_tor-tor);
    //ROS_INFO("%d\n", radtorpm(dxl_goal_vel));

            	
    return radtorpm(dxl_goal_vel);

}


*/



serial::Serial ser;


void serial_run(void *arg)
{

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");

    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{

    }

    string tmp;
    char tmp2[8];

    rt_task_set_periodic(NULL, TM_NOW, 1.5e6);
    while (run)
    {
        rt_task_wait_period(NULL);
        ser.write("a");
        rt_task_sleep(5e4);
        for (int i=0;i<8;i++){
            tmp = ser.read();
            tmp2[i] = strtol(tmp.c_str(),NULL,16);
        }
        serial_pt.Data1 = (tmp2[0] & 0b0011)<<12 | tmp2[1]<<8 | tmp2[2]<<4 | tmp2[3];
	serial_pt.Data2 = (tmp2[4] & 0b0011)<<12 | tmp2[5]<<8 | tmp2[6]<<4 | tmp2[7];
        //rt_printf("%d", serial_pt.Data1);
        //rt_printf("\n");
    }
}




boolean ecat_init(uint32_t mode)
{
    int i, oloop, iloop, chk, wkc_count;
    needlf = FALSE;
    inOP = FALSE;

    rt_printf("Strating DC test\n");
    if (ec_init(ecat_ifname))
    {
        rt_printf("ec_init on %s succeeded. \n", ecat_ifname);

        /* find and auto-config slaves in network */
        if (ec_config_init(FALSE) > 0)
        {
            rt_printf("%d slaves found and configured.\n", ec_slavecount);

            for (int k=0; k<NUMOFEPOS4_DRIVE; ++k)
            {
                if (( ec_slavecount >= 1 ) && (strcmp(ec_slave[k+1].name,"EPOS4") == 0)) //change name for other drives
                {
                    rt_printf("Re mapping for EPOS4 %d...\n", k+1);
                    os=sizeof(ob); ob = 0x00;	//RxPDO, check MAXPOS ESI
                    //0x1c12 is Index of Sync Manager 2 PDO Assignment (output RxPDO), CA (Complete Access) must be TRUE
                    wkc_count=ec_SDOwrite(k+1, 0x1c12, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);
                    wkc_count=ec_SDOwrite(k+1, 0x1c13, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);

                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob); ob = 0x60410010;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x01, FALSE, os, &ob, EC_TIMEOUTRXM);
                    os=sizeof(ob); ob = 0x60640020;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x02, FALSE, os, &ob, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x02;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    if (wkc_count==0)
                    {
                        rt_printf("RxPDO assignment error\n");
                        //return FALSE;
                    }
                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1A01, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1A02, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1A03, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob); ob = 0x60400010;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x01, FALSE, os, &ob, EC_TIMEOUTRXM);
                    os=sizeof(ob); ob = 0x607A0020;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x02, FALSE, os, &ob, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x02;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    if (wkc_count==0)
                    {
                        rt_printf("TxPDO assignment error\n");
                        //return FALSE;
                    }

                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1601, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1602, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1603, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob2); ob2 = 0x1600;
                    wkc_count=ec_SDOwrite(k+1, 0x1C12, 0x01, FALSE, os, &ob2, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x01;
                    wkc_count=ec_SDOwrite(k+1, 0x1C12, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob2); ob2 = 0x1A00;
                    wkc_count=ec_SDOwrite(k+1, 0x1C13, 0x01, FALSE, os, &ob2, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x01;
                    wkc_count=ec_SDOwrite(k+1, 0x1C13, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob3); ob3 = 0x01;
                    wkc_count=ec_SDOwrite(k+1, 0x60C2, 0x01, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob3); ob3 = mode;
                    wkc_count=ec_SDOwrite(k+1, 0x6060, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob); ob = 0x000186A0;
                    wkc_count=ec_SDOwrite(k+1, 0x6065, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);
                }
            }

            ec_config_map(&IOmap);
            /* Configurate distributed clock */
            ec_configdc();
            rt_printf("Slaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            /* configure DC options for every DC capable slave found in the list */
            rt_printf("DC capable : %d\n", ec_configdc());

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;

            rt_printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

            rt_printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            rt_printf("Caculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            /* To enter state OP we need to send valid date to outpus */
            /* send one valid process data to make outputs in slaves happy */
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);

            /* wait for all slaves to reach OP state */
            chk = 200;
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                rt_printf("Operational state reached for all slaves.\n");
                for (int k=0; k<NUMOFEPOS4_DRIVE; ++k)
                {
                    epos4_drive_pt[k].ptOutParam=(EPOS4_DRIVE_RxPDO_t*)  ec_slave[k+1].outputs;
                    epos4_drive_pt[k].ptInParam= (EPOS4_DRIVE_TxPDO_t*)  ec_slave[k+1].inputs;
                }
                inOP = TRUE;
            }
            else
            {
                rt_printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (i=1; i<ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State 0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
                for (i=0; i<NUMOFEPOS4_DRIVE; i++)
                    ec_dcsync0(i+1, FALSE, 0, 0);
            }
        }
        else
        {
            rt_printf("No slaves found!\n");
            inOP = FALSE;
        }
    }
    else
    {
        rt_printf("No socket connection on %s\nExecute as root\n", ecat_ifname);
        return FALSE;
    }
    return inOP;
}

void EPOS_CSP(void *arg)
{
    unsigned long ready_cnt = 0;
    uint16_t controlword=0;
    int ival = 0, i;

    if (ecat_init(0x08)==FALSE)
    {
        run = 0;
        printf("EPOS CSP FAIL");
        return;
    }
    rt_task_sleep(1e6);

    /* Distributed clock set up */
    long long toff;
    long long cur_DCtime = 0, max_DCtime = 0;
    unsigned long long cur_dc32 = 0, pre_dc32 = 0;
    int32_t shift_time = 380000;
    long long diff_dc32;

    for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
        ec_dcsync0(i+1, TRUE, cycle_ns, 0);

    RTIME cycletime = cycle_ns;
    RTIME cur_time = 0; // current master time
    RTIME cur_cycle_cnt = 0; // number of cycles has been passed
    RTIME cycle_time;  // cur_cycle_cnt*cycle_ns
    RTIME remain_time; // cur_time%cycle_ns
    RTIME dc_remain_time; // remain time to next cycle of REF clock, cur_dc32 % cycletime
    RTIME rt_ts; // updated master time to REF clock
    toff = 0;

    ec_send_processdata();
    cur_time = rt_timer_read();
    cur_cycle_cnt = cur_time/cycle_ns;
    cycle_time = cur_cycle_cnt*cycle_ns;
    remain_time = cur_time%cycle_ns;

    rt_printf("cycles have been passed : %lld\n", cur_cycle_cnt);
    rt_printf("remain time to next cycle : %lld\n", remain_time);

    wkc = ec_receive_processdata(EC_TIMEOUTRET); // get reference DC time
    cur_dc32 = (uint32_t)(ec_DCtime & 0xFFFFFFFF); // consider only lower 32-bit, because epos has 32-bit processor
    dc_remain_time = cur_dc32%cycletime;
    rt_ts = cycle_time + dc_remain_time; // update master time to REF clock

    rt_printf("DC remain time to next cycle : %lld\n", dc_remain_time);

    rt_task_sleep_until(rt_ts); // wait until next REF clock

    while (run)
    {
        // wait for next cycle
        rt_ts += (RTIME) (cycle_ns + toff);
        rt_task_sleep_until(rt_ts);

        previous = rt_timer_read();

        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        if (wkc < 3*NUMOFEPOS4_DRIVE)
            recv_fail_cnt++;
        now = rt_timer_read();
        ethercat_time = (long) (now-previous);

        cur_dc32 = (uint32_t) (ec_DCtime & 0xFFFFFFFF);
        if (cur_dc32>pre_dc32)
            diff_dc32 = cur_dc32-pre_dc32;
        else
            diff_dc32 = (0xFFFFFFFF - pre_dc32) + cur_dc32;
        pre_dc32 = cur_dc32;
        cur_DCtime += diff_dc32;
        toff = dc_pi_sync(cur_DCtime, cycletime, shift_time);

        if (cur_DCtime > max_DCtime)
            max_DCtime = cur_DCtime;

        //servo-on
        for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
        {
            controlword=0;
            started[i]=ServoOn_GetCtrlWrd(epos4_drive_pt[i].ptInParam->StatusWord, &controlword);
            epos4_drive_pt[i].ptOutParam->ControlWord=controlword;
            if (started[i]) ServoState |= (1<<i);
        }

        if (ServoState == (1<<NUMOFEPOS4_DRIVE)-1) //all servos are in ON state
        {
            if (servo_ready==0)
                servo_ready=1;
        }
        if (servo_ready) ready_cnt++;
        if (ready_cnt>=3000) //wait for 3s after servo-on
        {
            ready_cnt=10000;
            sys_ready=1;
        }

        if (sys_ready)
        {   

	    int input_amp = 25000; //34000
	    int interval = 5;
            int tr = 5;

	/*    if (gt > 1.0) ival = 0;
            
            for (int j=0; j < interval; j++){
		if (gt > j*tr +1){
			ival = int((float(input_amp)/float(interval))*float(j));
			}
		}
	
            for (int j=0; j < interval*2; j++){
		if (gt > (j*tr)+tr*interval +1){
			ival = -int((float(input_amp)/float(interval))*float(j)) + input_amp;
			}
		}

            for (int j=0; j < interval; j++){
		if (gt > (j*tr)+tr*interval*3 +1){
			ival = int((float(input_amp)/float(interval))*float(j))- input_amp;
			}
		}	

	    if (gt > tr*4*interval+1) ival = 0;	
*/
	    /*if (gt > 31.0) sine_amp = sin_max - sin_max*((gt-31.0)/60.0);
	    else sine_amp =sin_max;

            if (sine_amp < 0) sine_amp = 0;*/
	    if (gt > 1.0) ival=(int) (input_amp*(sin(PI2*f*(gt-1.0)))); //(sine_amp*(sin(PI2*f*gt)))ival = sin_amp;
	    else ival = 0;
		if (gt > 61.0) ival = 0;



	    /*int ramp_max = 20000;
	    if (gt < 1.0) ival = 0;
	    else if(gt < 11.0){
			ival = (int) (ramp_max*((gt-1.0)/5));
			if (ival > ramp_max) ival = ramp_max;
		}
	    else if (gt<26.0){
			ival = (int) (-ramp_max*((gt-11.0)/5)+ramp_max);
			if (ival < -ramp_max) ival = -ramp_max;
		}
	    else {
			ival = (int) (ramp_max*((gt-26.0)/5)-ramp_max);
			if (ival > 0) ival = 0;
		}*/


            for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
            {
                if (i%2==0)
                    epos4_drive_pt[i].ptOutParam->TargetPosition=ival + zeropos[i];
                else
                    epos4_drive_pt[i].ptOutParam->TargetPosition=-ival + zeropos[i];
            }
            gt+=period;
        }
        else
        {
            for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
            {
                zeropos[i]=epos4_drive_pt[i].ptInParam->PositionActualValue;
                epos4_drive_pt[i].ptOutParam->TargetPosition=zeropos[i];
            }
        }


        if (sys_ready)
            if (worst_time<ethercat_time) worst_time=ethercat_time;
    }

    rt_task_sleep(cycle_ns);

    for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
        ec_dcsync0(i+1, FALSE, 0, 0); // SYNC0,1 on slave 1

    //Servo OFF
    for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
    {
        epos4_drive_pt[i].ptOutParam->ControlWord=6; //Servo OFF (Disable voltage, transition#9)
    }
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    rt_task_sleep(cycle_ns);

    rt_printf("End EPOS CSP control, close socket\n");
    /* stop SOEM, close socket */
    printf("Request safe operational state for all slaves\n");
    ec_slave[0].state = EC_STATE_SAFE_OP;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach state */
    ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
    ec_slave[0].state = EC_STATE_PRE_OP;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach state */
    ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);

    ec_close();

}

void print_run(void *arg)
{
    int i;
    unsigned long itime = 0;
    long stick = 0;
    int argc;
    char** argv;
    msgpkg::realVal msg;
    ros::init(argc, argv, "EtherCAT_control");
    ros::NodeHandle n;
    ros::Publisher pos_pub = n.advertise<msgpkg::realVal>("position",1);

    
    rt_task_set_periodic(NULL, TM_NOW, 5e6);

    while (run)
    {
        rt_task_wait_period(NULL);
        if (inOP==TRUE)
        {
            if (!sys_ready)
            {
                if(stick==0)
                    rt_printf("waiting for system ready...\n");
                if(stick%10==0)
                    rt_printf("%i \n", stick/10);
                stick++;
            }
            else
            {
		msg.realPos = epos4_drive_pt[0].ptInParam->PositionActualValue;
		msg.motPos = epos4_drive_pt[0].ptOutParam->TargetPosition;
                msg.adc1 = serial_pt.Data1;
		msg.adc2 = serial_pt.Data2;
		pos_pub.publish(msg);
            }
        }
    }
}



void catch_signal(int sig)
{
    run = 0;
    usleep(5e5);
    rt_task_delete(&motion_task);
    rt_task_delete(&print_task);
    exit(1);
}

int main(int argc, char *argv[])
{
    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
    mlockall(MCL_CURRENT | MCL_FUTURE);

//    cycle_ns=1000000; // nanosecond
    period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit
    if (argc > 1)
    {
        sine_amp=atoi(argv[1]);
    }
    printf("use default adapter %s\n", ecat_ifname);

    cpu_set_t cpu_set_ecat;
    CPU_ZERO(&cpu_set_ecat);
    CPU_SET(0, &cpu_set_ecat); //assign CPU#0 for ethercat task
    cpu_set_t cpu_set_print;
    CPU_ZERO(&cpu_set_print);
    CPU_SET(1, &cpu_set_print); //assign CPU#1 (or any) for main task
    cpu_set_t cpu_set_serial;
    CPU_ZERO(&cpu_set_serial);
    CPU_SET(2, &cpu_set_serial);

    rt_task_create(&motion_task, "SOEM_motion_task", 0, 90, 0 );
    rt_task_set_affinity(&motion_task, &cpu_set_ecat); //CPU affinity for ethercat task

    rt_task_create(&print_task, "ec_printing", 0, 50, 0 );
    rt_task_set_affinity(&print_task, &cpu_set_print); //CPU affinity for printing task

    rt_task_create(&serial_task, "serial_task", 0, 70, 0 );
    rt_task_set_affinity(&serial_task, &cpu_set_serial); //CPU affinity for ethercat task
    
    rt_task_start(&motion_task, &EPOS_CSP, NULL);
    rt_task_start(&print_task, &print_run, NULL);
    rt_task_start(&serial_task, &serial_run, NULL);

    while (run)
    {
        usleep(1000000);
    }


    printf("End program\n");
    return (0);


}
