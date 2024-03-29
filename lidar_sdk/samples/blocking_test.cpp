#include "ord_lidar_driver.h"

using namespace std;
using namespace ordlidar;

unsigned char running = 1;
static void sig_handle(int signo)
{
    printf("program exit, [%s,%s] Receive SIGNAL %d ====== \r\n", __FILE__, __func__, signo);
    running = 0;
    delay(1000);
    exit(1);
}

int main(int argc, char *argv[])
{
    (void)argc;(void)argv;
    
    signal(SIGINT, sig_handle);

    uint8_t type = ORADAR_TYPE_SERIAL;
    int model = ORADAR_MS200;

    OrdlidarDriver device(type, model);
    full_scan_data_st scan_data;
    
    #if defined(_WIN32)
    std::string port_name("com18");
    #else
    std::string port_name("/dev/Lidar");
    #endif
    int serialBaudrate = 230400;
    bool is_logging = true;
    bool ret = false;
    long long count = 0;
    device.SetSerialPort(port_name, serialBaudrate);

    while (running)
    {
        if (device.Connect())
        {
            printf("scan_frame_data lidar device connect succuss..\n");
            break;
        }
        else
        {
            printf("lidar device connect %s fail..\n", port_name.c_str());
            delay(1000);
        }
    }

    while (running)
    {
        ret = device.GrabFullScanBlocking(scan_data, 1000);
        if (ret)
        {
            printf("count = %lld, point_num: %d\n", ++count, scan_data.vailtidy_point_num);
            if (is_logging)
            {
                for (int i = 0; i < scan_data.vailtidy_point_num; i++)
                {
                    printf("[%d: %X, %X] \n", i, (scan_data.data[i]), scan_data.data[i].angle);
                }
            }
        }
        else
        {
            printf("error: fail get full scan data\n");
        }
    }

    device.Disconnect();
exit:
    return 0;
}