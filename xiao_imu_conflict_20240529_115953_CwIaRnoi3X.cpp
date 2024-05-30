/**
 ****************************************************************************
 * @file        force_sensors.cpp
 * @brief       Handle force sensors reading in a thread
 ****************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  kamilo.melo@km-robota.com, 10/2023
 * @authors  katarina.lichardova@km-robota.com, 10/2023
 * @authors  melike.cezayirlioglu@km-robota.com, 10/2023
 ****************************************************************************
 */

#include <iostream>
#include <fcntl.h>      // Control over the open file referenced by file descriptor 
#include <string> 
#include <termios.h>    // POSIX Terminal Control Definitions   
#include <unistd.h>     // Sleep function
#include <cstring>

#include "xiao_imu.hpp"

#define NBR_SENSORS	1

#define ASCII_EXCLAMATION 33

using namespace std;

/**
 * @brief       Create and initialize a IMU object
 * @note        By default, open ttyACM0. If you're using a different
 *              port, use the overload of this constructor
 */
IMU::IMU()
{
    m_stopThread = false;

	clearBuffer();
	exit(1);

    m_thread = thread(&IMU::IMULoop, this, "/dev/ttyACM0");

    cout << "Creating the IMU thread..." << endl;
}


/**
 * @brief       Create a IMU object attached to a different port than ACMO.\n
 * 				The port name is in the form of /dev/ttyACMx
 * @retval      void
 */
IMU::IMU(const char* imu_portname)
{
    m_stopThread = false;

	clearBuffer();
	exit(1);

    m_thread = thread(&IMU::IMULoop, this, imu_portname);

    cout << "Creating the IMU thread..." << endl;
}

/**
 * @brief       Class destructor. Takes care of safely stopping the thread
 */
IMU::~IMU()
{
    m_stopThread = true;

    cout << "Safely stopping the force sensors' thread.... " << endl;

    if (m_thread.joinable()) 
        m_thread.join();
}

/**
 * @brief       Read and save the current IMU values. To loop in a thread
 * @param[in]   sensors_portname Port handling the sensors
 * @retval      1 when the thread is finished
 */
int IMU::IMULoop(const char* imu_portname)
{
    int fd; // File descriptor

    // O_RDWR   - Read/Write access to serial port 
    // O_NOCTTY - No terminal will control the process 
    // Open in blocking mode, read will wait
	fd = open(imu_portname, O_RDWR | O_NOCTTY);

	if(fd < 0) {
		printf("Error in opening the IMU's port!\n");
		exit(0);
	}
	printf("IMU port open successfully!\n");


	// ----- Setting the attributes of the serial port using termios structure -----

	struct termios SerialPortSettings;	// Create the structure                          
	tcgetattr(fd, &SerialPortSettings);	// Get the current attributes of the Serial port 

	// Setting the Baud rate 
	cfsetispeed(&SerialPortSettings, 230400 ); // Set Read  Speed as 230400                       
	cfsetospeed(&SerialPortSettings, 230400 ); // Set Write Speed as 230400                       

	/* 8N1 Mode */
	SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity   
	SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit 
	SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size             
	SerialPortSettings.c_cflag |=  CS8;      // Set the data bits = 8                                 

	SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control                  
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver,Ignore Modem Control lines

	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both i/p and o/p
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode                           

	SerialPortSettings.c_oflag &= ~OPOST;   // No Output Processing

	// Setting Time outs
	SerialPortSettings.c_cc[VMIN] = 40;  // Read at least 10 characters 
	SerialPortSettings.c_cc[VTIME] = 10; // Wait indefinetly   

    // Set the new attributes to the termios structure
	if((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) {
	    cout << "ERROR in setting serial port attributes!" << endl;
        sleep(1);
    }

    char read_buffer[256];   // Buffer to store the data received              
	char useful_buffer[256];
	int  bytes_read = 0;     // Number of bytes read by the read() system call 
	int bytes_useful = 0;

	// -----  Start of the main loop -----
	cout << "Starting sensor reading" << endl;
    while(!m_stopThread) {

		//tcflush(fd, TCIFLUSH);   // Discards old data in the rx buffer 
		bytes_read = read(fd, &read_buffer, 256);   // Read the data

		// Print read values
		cout << "Bytes read: " << bytes_read << endl;    
		for (int i=0; i<bytes_read; i++) {
			char c = read_buffer[i];

			if (c == 0)
				cout << "EOF";
			else if (c == 10) // newline
				cout << "newline";
			else if (c == 1) // start of heading
				cout << "heading";
			else if (c == 13) // carriage return
				cout << "carriage";
			else 
				cout << c;
		}
		cout << endl;
		
		
		/*cout << "Bytes read: " << bytes_useful << endl;    

		for (int i=0; i<bytes_read; i++) {
			if(read_buffer[i] != '\n') {
				useful_buffer[i] = read_buffer[i];
				bytes_useful++;
			}
			else
				break;
		}

		// Print read values
		cout << "Bytes read: " << bytes_useful << endl;    
		for(int i=0; i<bytes_useful; i++)	{ // printing only the received characters
			cout << useful_buffer[i]; 
		} 
		cout << endl; */


		// Interpret
		/*if (bytes_read > 0)
			interpretData(read_buffer, bytes_read);
		else {
			scoped_lock lock(m_mutex);

			m_IMU.accX = 0;
			m_IMU.accY = 0;
			m_IMU.accZ = 0;
			m_IMU.gyroX = 0; 
			m_IMU.gyroY = 0; 
			m_IMU.gyroZ = 0; 
		}*/

		usleep(50);
    }

    close(fd); // Close the serial port 
    cout << "Force sensors' serial port closed successfully! " << endl; 
    return 0;
}

void IMU::interpretData(char* read_buffer, int bytes_read)
{
	int offset = 0;
	char c;
	int nbr_digits = 0;
	string value_c;
	float value;
	vector<float> values;
	IMUStruct sensor;

	// First, transform the array of chars (the buffer) into real values
	for (int i=0; i<bytes_read; i++) {
		c = read_buffer[i];

		if (c != ',' && c != 0) 
			nbr_digits++;

		else {
			value_c.clear();

			for (int j=0; j<nbr_digits; j++)
				value_c.push_back(read_buffer[offset + j]);

			// Convert the string to a float	
			value = atof(value_c.c_str());
			values.push_back(value);

			// Update the offset
			offset = i+1;
			nbr_digits = 0;
		}

	}
 
	// Store those values into a comprehensive structure
	scoped_lock lock(m_mutex);

	m_IMU.accX = values[0];
	m_IMU.accY = values[1];
	m_IMU.accZ = values[2];
	m_IMU.gyroX = values[3];
	m_IMU.gyroY = values[4];
	m_IMU.gyroZ = values[5];


	// Print read values
	cout << "\nXIAO: " << endl;
	cout << "Accelerations: " << endl;
	cout << "\tX: " << m_IMU.accX << endl;
	cout << "\tY: " << m_IMU.accY << endl;
	cout << "\tZ: " << m_IMU.accZ << endl;
	cout << "Angular velocities: " << endl;
	cout << "\tX: " << m_IMU.gyroX << endl;
	cout << "\tY: " << m_IMU.gyroY << endl;
	cout << "\tZ: " << m_IMU.gyroZ << endl; 

	if (m_IMU.accX != 1)
		exit(1);

}

/*
IMUStruct IMU::getIMU()
{
	scoped_lock lock(m_mutex);

	return m_IMU;
} */

void IMU::getIMU(IMUStruct& imu)
{
	scoped_lock lock(m_mutex);

	imu.accX = m_IMU.accX;
	imu.accY = m_IMU.accY;
	imu.accZ = m_IMU.accZ;
	imu.gyroX = m_IMU.gyroX;
	imu.gyroY = m_IMU.gyroY;
	imu.gyroZ = m_IMU.gyroZ;
	
}

void IMU::clearBuffer()
{
	memset(m_read_buffer, ASCII_EXCLAMATION, BUFFER_SIZE);
}