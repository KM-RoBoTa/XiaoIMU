#include "xiao_imu.hpp"
#include <unistd.h>     // Sleep function

#define MAX_TIME    5       // 5sec
#define USLEEP_TIME 10000   // us
#define MAX_CTR     MAX_TIME*1000000/USLEEP_TIME


using namespace std;

int main()
{
    IMU imu("/dev/ttyACM0");
    IMUStruct imu_vals;
    imu.calibrateSensor();

    sleep(1);

    int ctr = 0;
    while(ctr < MAX_CTR) {
        imu.getValues(imu_vals);

        // Print read values
        cout << "\nMAIN THREAD: " << endl;
        cout << "Accelerations [m/s²]: " << endl;
        cout << "\tX: " << imu_vals.accX << endl;
        cout << "\tY: " << imu_vals.accY << endl;
        cout << "\tZ: " << imu_vals.accZ << endl;
        cout << "Angular velocities [deg/s]: " << endl;
        cout << "\tX: " << imu_vals.gyroX << endl;
        cout << "\tY: " << imu_vals.gyroY << endl;
        cout << "\tZ: " << imu_vals.gyroZ << endl;

        ctr++;

        usleep(10000);
    }
}