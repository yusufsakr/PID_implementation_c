
/*
 *
 *  Code Created by "Lankash"
 *  @5/2/2022
 * File Contents: 2 PID Functions + PID Struct
 *
 */


#ifndef PID_H
#define PID_H

typedef struct {

	float kp;    //Proportional PID Gain
	float ki;    //Integral PID Gain
	float kd;    //Derivative PID Gain

	float limMax;   //Output Max Limit
	float limMin;   //Output Min limit
	
	float limMax_init;    //Integrator limits (For Anti-windUp)
	float limMin_init;    //Integrator limits

	float taw;    //Response time
	float T;     //sample time (sec)

	float integ;            //Integrator updating memory
	float diff;             //Differentiation updating memory
	float prevError;        //Previous time sample error
	float prevMeasurement;  //Previous time sample sensor measurement

	float outPut;     //Controller Output Signal
} PID;


//..........................................................................

/*
 * Function name        : PID_init
 * Function returns     : void
 * Function arguments   : PID *pid
 * Function description : Initialize The PID parameters for the first shot
 */
void PID_init (PID *pid);


/*
 * Function name        : PID_update
 * Function returns     : void
 * Function arguments   : PID *pid, float setPoint, float measurement
 * Function description : Update the PID parameters and O/Ps every sample.
 */
void PID_update (PID *pid, float setPoint, float measurement);

//.................................................................

#endif
