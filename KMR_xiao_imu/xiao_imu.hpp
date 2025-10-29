/**
 ******************************************************************************
 * @file            xiao_imu.hpp
 * @brief           Header for the xiao_imu.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  katarina.lichardova@km-robota.com, 05/2024
 * @authors  kamilo.melo@km-robota.com, 05/2024
 ******************************************************************************
 */

#pragma once

#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#define BUFFER_SIZE 255

namespace KMR::XIAO
{

/**
 * @brief   Structure for IMU data
 */
struct IMUStruct {
    float accX;     // [m/s^2]
    float accY;     // [m/s^2]
    float accZ;     // [m/s^2]
    float gyroX;    // [deg/s]
    float gyroY;    // [deg/s]
    float gyroZ;    // [deg/s]
};


/**
 * @brief   Class handling a Seeed Studio's XIAO nRF52840 Sense IMU
 * @details IMU reading runs in its own separate thread, handled by this class
 */
class IMU {
public:
    IMU();
    IMU(const char* sensors_portname);
    ~IMU();
    
    void getValues(IMUStruct& imu);
    void calibrateSensor();

private:
    bool m_stopThread = 0;
    IMUStruct m_IMU;
    IMUStruct m_tareOffsets;
    char m_buffer[BUFFER_SIZE];
    char m_packet[BUFFER_SIZE];
    int m_occupied_bytes = 0;

    std::thread m_thread;
    std::mutex m_mutex;
    int IMULoop(const char* imu_portname);
    void interpretData();
    void clearBuffer(char* buffer);
    void printBuffer(char* buffer);
    bool extractPacket();
    int openPort(const char* imu_portname);
    void nullifyStruct(IMUStruct& imu);
};

}