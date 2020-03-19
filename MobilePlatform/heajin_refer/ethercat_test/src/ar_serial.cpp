//sudo slcand -o -c -f -s4 /dev/ttyUSB3 slcan0
//sudo ifconfig slcan0 up
//candump slcan0

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "msgpkg/fabric.h"

#include <sstream>
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
#include <unistd.h>
#include <iostream>
#include <iomanip>

#include "soem/ethercat.h"
#include "pdo_def.h"
#include "servo_def.h"
#include "ecat_dc.h"

#include <serial/serial.h>

using namespace std;


Arduino_Serial_pt	serial_pt;
int run = 1;
RT_TASK print_task;
RT_TASK serial_task;

serial::Serial ser;
msgpkg::fabric msg;



void serial_run(void *arg)
{

    try
    {
        ser.setPort("/dev/ttyACM0");
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
    char tmp2[4];

    rt_task_set_periodic(NULL, TM_NOW, 2e6);
    while (run)
    {
        rt_task_wait_period(NULL);
        ser.write("a");
        rt_task_sleep(5e4);
        for (int i=0;i<4;i++){
            tmp = ser.read();
            tmp2[i] = strtol(tmp.c_str(),NULL,16);
        }
        serial_pt.Data1 = (tmp2[0] & 0b0011)<<12 | tmp2[1]<<8 | tmp2[2]<<4 | tmp2[3];
        //rt_printf("%d", serial_pt.Data1);
        //rt_printf("\n");
    }
}

void print_run(void *arg){
    rt_task_set_periodic(NULL, TM_NOW, 1e8);
    while(run)
    {
        rt_task_wait_period(NULL);
        rt_printf("%d\n", serial_pt.Data1);
    }

}


int main(int argc, char **argv) {


    cpu_set_t cpu_set_serial;
    CPU_ZERO(&cpu_set_serial);
    CPU_SET(0, &cpu_set_serial); //assign CPU#0 for ethercat task
    cpu_set_t cpu_set_print;
    CPU_ZERO(&cpu_set_print);
    CPU_SET(1, &cpu_set_print); //assign CPU#1 (or any) for main task

    rt_task_create(&serial_task, "serial_task", 0, 90, 0 );
    rt_task_set_affinity(&serial_task, &cpu_set_serial); //CPU affinity for ethercat task

    rt_task_create(&print_task, "ec_printing", 0, 50, 0 );
    rt_task_set_affinity(&print_task, &cpu_set_print); //CPU affinity for printing task

    rt_task_start(&serial_task, &serial_run, NULL);
    rt_task_start(&print_task, &print_run, NULL);

    while (run) {
        
            usleep(1000000);
        }
        return 0;


}
