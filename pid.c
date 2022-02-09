/*
 *
 *  Code Created by "Lankash"
 *  @5/2/2022
 * File Contents: 2 PID Functions + PID Struct
 *
 */

#include "pid.h"

//...............................................

void PID_init (PID *pid)
{
    /*
     *  Clear all the PID parameters and memory values.
	 */

	pid->integ = 0.0f;
	pid->prevError = 0.0f;

	pid->diff = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->outPut = 0.0f;
}

//...............................................

void PID_update (PID *pid, float setPoint, float measurement)
{
    /*
    * Error Signal
    */
    float error = setPoint - measurement;


    /*
    * Proportional
    */
    float proportional = pid->kp * error;


    /*
    * Integral
    */
    pid->integ =  pid->integ + (0.5f * pid->ki * pid->T * (error + pid->prevError));


    /*
    * Anti windUp + Clamp the Integrator
    */
    if (pid->integ > pid->limMax_init)
    {
        pid->integ = pid->limMax_init;
    }
    else if (pid->integ < pid->limMin_init)
    {
        pid->integ = pid->limMin_init;
    }


    /*
    * Derivative (Band Limited) + Low Pass Filter
    */
    pid->diff = ((2.0f * pid->kd) * (measurement - pid->prevMeasurement)      //Diffrintiator
                         + (2 * (pid->taw - pid->T) * pid->diff))                        //Low Pass Filter
                         / (2 * pid->taw) + pid->T);



    /*
    * Compare O/P and apply clipping limits
    */
    pid->outPut = proportional + pid->integ + pid->diff;

    if (pid->outPut > pid->limMax)
    {
        pid->outPut = pid->limMax;
    }
    else if (pid->outPut < pid->limMin)
    {
        pid->outPut = pid->limMin;
    }


    /*
    * Store new values : Error & Measurements
    */
    pid->prevError = error;
    pid->prevMeasurement = measurement;


    /*
    * Return PID O/P
    */
    return pid->outPut;
}

//....................................................................................
