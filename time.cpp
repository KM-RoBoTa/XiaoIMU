 /**
 ******************************************************************************
 * @file            time.cpp
 * @brief           Util functions related to time measurements
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  kamilo.melo@km-robota.com, 10/2023
 * @authors  katarina.lichardova@km-robota.com, 10/2023
 * @authors  melike.cezayirlioglu@km-robota.com, 10/2023
 ******************************************************************************
 */

#include <iostream>

#include "time.hpp"


using namespace std;


/**
 * @brief   Get current time structure
 * @return  Current time structure
 */
timespec time_s()
{
    struct timespec real_time;

    if (clock_gettime(CLOCK_REALTIME, &real_time) == -1 )
    {
        perror("clock gettime");
        exit( EXIT_FAILURE );
    }

    return real_time;
}

/**
 * @brief       Get elapsed time, in microseconds
 * @param[in]   t2 End time structure (gotten with time_s)
 * @param[in]   t2 Start time structure (gotten with time_s)
 * @return      Elapsed time between t1 and t2, in us
 */
double get_delta_us(struct timespec t2, struct timespec t1)
{
    struct timespec td;
    td.tv_nsec = t2.tv_nsec - t1.tv_nsec;
    td.tv_sec  = t2.tv_sec - t1.tv_sec;
    if (td.tv_sec > 0 && td.tv_nsec < 0)
    {
        td.tv_nsec += 1000000000;
        td.tv_sec--;
    }

    return(td.tv_sec*1000000 + td.tv_nsec/1000);
}