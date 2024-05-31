#include "xiao_imu.hpp"
#include <unistd.h>     // Sleep function
#include <sstream> 
#include <fstream> 


std::ofstream file_force_sensors;

using namespace std;

int main()
{
    IMU imu("/dev/ttyACM0");
    IMUStruct imu_vals;
    //file_force_sensors.open ("force_sensors.csv");
    sleep(1);

    while(1) {
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

        usleep(10000);
    }
}