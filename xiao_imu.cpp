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
#include <sys/ioctl.h> 
#include <linux/serial.h>

#include "xiao_imu.hpp"
#include "time.hpp"

#define ASCII_NULL			0  	// NULL
#define ASCII_SOH			1	// Start of header
#define ASCII_NEWLINE		10 	// Newline
#define ASCII_CR			13	// Carriage return
#define ASCII_EXCLAMATION 	33	// Exclamation point

using namespace std;

// About low latency: https://stackoverflow.com/questions/13126138/low-latency-serial-communication-on-linux
// https://forum.pjrc.com/index.php?threads/reducing-latency-in-serial-communication.72162/

/**
 * @brief       Create and initialize a IMU object
 * @note        By default, open ttyACM0. If you're using a different
 *              port, use the overload of this constructor
 */
IMU::IMU()
{
    m_stopThread = false;

	clearBuffer(m_buffer);
	clearBuffer(m_packet);

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

	clearBuffer(m_buffer);
	clearBuffer(m_packet);

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

	openPort(imu_portname);


    char tmp_buffer[BUFFER_SIZE];   // Buffer to store the data received              
	int  bytes_read = 0;     // Number of bytes read by the read() system call 


	// TIME TESTS
	struct timespec loop_start, now, prev, prev_valid;
	double elapsed = 0;
	double dur_reading, dur_buffering, dur_extracting, dur_interpret, dur_valid;

	prev_valid = time_s();

	// -----  Start of the main loop -----
	cout << "Starting sensor reading" << endl;
    while(!m_stopThread) {



		// TIME TEST
		//loop_start = time_s();

		//tcflush(fd, TCIFLUSH);   // Discards old data in the rx buffer 
		bytes_read = read(m_fd, &tmp_buffer, BUFFER_SIZE);   // Read the data

		// TIME TESTS
		//now = time_s();
        //dur_reading = get_delta_us(now, loop_start);

		// Print read values
		/*if (bytes_read != -1)
			cout << "\nBytes read: " << bytes_read << endl;    
		for (int i=0; i<bytes_read; i++) {
			char c = tmp_buffer[i];

			if (c == ASCII_NEWLINE) // newline
				cout << "newline";
			//else if (c == ASCII_NULL)
				//cout << "EOF";
			else if (c == ASCII_SOH) // start of heading
				cout << "heading";
			else if (c == ASCII_CR) // carriage return
				cout << "carriage";
			else 
				cout << c;
		}*/
		//cout << endl;
		
		// TIME TEST
		//prev = time_s();

		if (m_occupied_bytes == 0) {
			if (strchr(tmp_buffer, ASCII_SOH) != NULL) {
				memcpy(m_buffer, tmp_buffer, bytes_read);
				m_occupied_bytes = bytes_read;
			}
		}
		else {
			memcpy(m_buffer+m_occupied_bytes, tmp_buffer, bytes_read);
			m_occupied_bytes += bytes_read;
		}

		//now = time_s();
        //dur_buffering = get_delta_us(now, prev);
		//prev = now;

		//printBuffer(m_buffer);

		//prev = time_s();
		bool output = extractPacket();

		if (output) {
			now = time_s();
        	dur_valid = get_delta_us(now, prev_valid);
			cout << "Time between 2 readings: " << dur_valid << " uS" << endl;
			prev_valid = now;
		}


		//now = time_s();
        //dur_extracting = get_delta_us(now, prev);
		//prev = now;		

		//prev = time_s();

		// Interpret
		if (output)
			interpretData();

		

		/*else {
			scoped_lock lock(m_mutex);

			m_IMU.accX = 0;
			m_IMU.accY = 0;
			m_IMU.accZ = 0;
			m_IMU.gyroX = 0; 
			m_IMU.gyroY = 0; 
			m_IMU.gyroZ = 0; 
		}*/
	

		// TIME TESTS
		//now = time_s();
        //elapsed = get_delta_us(now, loop_start);
		//dur_interpret = get_delta_us(now, prev);

		/*if (output) {
			//cout << "\nDuration reading: " << dur_reading << " uS (" << dur_reading/elapsed*100 << "'%' of time)" << endl;
			//cout << "Duration buffering: " << dur_buffering << " uS (" << dur_buffering/elapsed*100 << "'%' of time)" << endl;
			//cout << "Duration extracting: " << dur_extracting << " uS (" << dur_extracting/elapsed*100 << "'%' of time)" << endl;
			cout << "Duration interpreting: " << dur_interpret << " uS (" << dur_interpret/elapsed*100 << "'%' of time)" << endl;
		}*/

		//if (output)
		//	cout << "Elapsed time without sleep: " << elapsed << " uS" << endl;


		std::this_thread::sleep_for(chrono::microseconds(50));

		//now = time_s();
        //elapsed = get_delta_us(now, loop_start);
		//cout << "Elapsed time with sleep: " << elapsed << " uS" << endl;
    }

    close(m_fd); // Close the serial port 
    cout << "Force sensors' serial port closed successfully! " << endl; 
    return 0;
}

bool IMU::openPort(const char* imu_portname)
{

    // O_RDWR   - Read/Write access to serial port 
    // O_NOCTTY - No terminal will control the process 
    // Open in blocking mode, read will wait
	m_fd = open(imu_portname, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);

	if(m_fd < 0) {
		printf("Error in opening the IMU's port!\n");
		exit(0);
	}
	printf("IMU port open successfully!\n");


	// ----- Setting the attributes of the serial port using termios structure -----

	struct termios SerialPortSettings;	// Create the structure   
	struct serial_struct ser_info;                       
	tcgetattr(m_fd, &SerialPortSettings);	// Get the current attributes of the Serial port 

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

	// Enable linux FTDI low latency mode
    ioctl(m_fd, TIOCGSERIAL, &ser_info);
    ser_info.flags |= ASYNC_LOW_LATENCY;
    ioctl(m_fd, TIOCSSERIAL, &ser_info);

    // Set the new attributes to the termios structure
	if((tcsetattr(m_fd, TCSANOW, &SerialPortSettings)) != 0) {
	    cout << "ERROR in setting serial port attributes!" << endl;
        sleep(1);
    }
}

void IMU::interpretData()
{
	int offset = 0;
	char c;
	int nbr_digits = 0;
	string value_c;
	float value;
	vector<float> values;
	IMUStruct sensor;
	int bytes_read = 0;

	// Get the useful length of the buffer
	while(m_packet[offset] != ASCII_NULL) {
		bytes_read++;
		offset++;
	}
	offset = 0;
	bytes_read++; // Use the first NULL as the end delimiter

	// First, transform the array of chars (the buffer) into real values
	for (int i=0; i<bytes_read; i++) {
		c = m_packet[i];

		if (c != ',' && c != ASCII_NULL) 
			nbr_digits++;

		else {
			value_c.clear();

			for (int j=0; j<nbr_digits; j++)
				value_c.push_back(m_packet[offset + j]);

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
	/*cout << "\nXIAO: " << endl;
	cout << "Accelerations: " << endl;
	cout << "\tX: " << m_IMU.accX << endl;
	cout << "\tY: " << m_IMU.accY << endl;
	cout << "\tZ: " << m_IMU.accZ << endl;
	cout << "Angular velocities: " << endl;
	cout << "\tX: " << m_IMU.gyroX << endl;
	cout << "\tY: " << m_IMU.gyroY << endl;
	cout << "\tZ: " << m_IMU.gyroZ << endl; */
}


void IMU::getIMU(IMUStruct& imu)
{
	scoped_lock lock(m_mutex);

	imu = m_IMU;	
}

void IMU::clearBuffer(char* buffer)
{
	memset(buffer, ASCII_NULL, BUFFER_SIZE);

	// debug
	//printBuffer(buffer);
}

void IMU::printBuffer(char* buffer)
{
	char c;

	for (int i=0; i<255; i++) {
		c = buffer[i];

		if (c == ASCII_NEWLINE) // newline
			cout << "newline";
		//else if (c == ASCII_NULL)
		//	cout << "EOF";
		else if (c == ASCII_SOH) // start of heading
			cout << "heading";
		else if (c == ASCII_CR) // carriage return
			cout << "carriage";
		else 
			cout << c;
	}
	cout << endl << endl;
	
}

bool IMU::extractPacket()
{
	char* ret = strchr(m_buffer, ASCII_SOH);
	int startIndex = 0, endIndex = 0;
	int CRIndex = 0;

	if (ret != NULL) {
		startIndex = (int) (ret - m_buffer);

		// Now find the first newline, indicating the end of the packet
		char* ret2 = strchr(ret, ASCII_NEWLINE);

		if (ret2 != NULL) {
			endIndex = (int) (ret2 - m_buffer);

			memcpy(m_packet, m_buffer+startIndex+1, endIndex-startIndex-1);

			// Check for wild carriage returns
			char* ret3 = strchr(m_packet, ASCII_CR);
			while(ret3 != NULL) {
				CRIndex =  (int) (ret3 - m_packet);
				m_packet[CRIndex] = ASCII_NULL;

				ret3 = strchr(m_packet, ASCII_CR);
			}

			//cout << "New full packet: " << endl;
			//printBuffer(m_packet);

			// Clean the buffer
			clearBuffer(m_buffer);
			m_occupied_bytes = 0;

			// Successfully read new packet
			return 1;
		}
	}

	return 0;
}