 /**
 ******************************************************************************
 * @file            time.hpp
 * @brief           Header file for time.cpp
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  kamilo.melo@km-robota.com, 10/2023
 * @authors  katarina.lichardova@km-robota.com, 10/2023
 * @authors  melike.cezayirlioglu@km-robota.com, 10/2023
 ******************************************************************************
 */

#ifndef TIME_HPP
#define TIME_HPP


#include <iostream>
#include <unistd.h> // Provides the usleep function

timespec time_s();
double get_delta_us(struct timespec t2, struct timespec t1);


#endif