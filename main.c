/******************************************
*
*	High-altitude drone balloon
*	Author: Chuhan Qin, Fraser Filice, Vietca Vo
*	Date: Feb. 16, 2014
*
********************************************/

#define SAFETY_LATITUDE 	0.1			// 0.1 degrees, approx. 11km
#define SAFETY_LONGITUDE	0.2			// 0.2 degrees, approx. 13km
#define TARGET_LATITUDE		43.364751f	// 43.121535  N, destination
#define TARGET_LONGITUDE	-81.481991f	// -81.331787 W, destination

#define FLIGHT_LIMIT		120			// in minutes

typedef int bool;
#define true 1
#define false 0

int 	c = 0;						// counter for calling able_to_fight_wind()
int		j = 0;						// counter for counting able_to_fight_wind() retries
int 	k = 0;						// counter for operations() -> calling sensor_package()
Time 	start_time = Time.now();	// set starting time
SDcard 	sd_card; 					// initialize SD card logging
Motor 	motors;						// propellers

double latitude_current;			
double longitude_current;

void setup()
{
	sd_card.write(Time.now());				// current time
	sd_card.write("Flight started.\n\n");
	sensor_package_1();
	sensor_package_2();
	sd_card.write("\n\n");
}

void loop()
{
	latitude_current  = GPS.get_latitude();
	longitude_current = GPS.get_longitude();
	
	if(check_flight_duration() >= FLIGHT_LIMIT)
		start_descent();
	else if(c % 50 == 0 && !check_coordinates()) // both flying and every 50 times
		if(!able_to_fight_wind(latitude_current, longitude_current))
		{
			j++;
			if(j >= 20)
				start_descent();
		}
	else;
	
	operations(check_coordinates());
	c++;
}

/******************************************
*	
*	void operations(bool in_safe_radius)
*	Check if the balloon is facing the correct direction
* 	  and if it is in the safe radius,
*	1/0: fly forward and log sensor data
*	1/1: log sensor data
* 	0/0: correct direction
*	0/1: let it flow, let it flow, let it flow  
*
*	Author: Chuhan Qin
*
********************************************/
void operations(bool in_safe_radius)
{
	sd_card.write(Time.now());					// current time
	if(check_direction()) 						// if direction is correct
	{
		if(in_safe_radius = false)				// not in safe radius means need to fly
		{
			sd_card.write("Direction correct, not in safe radius, need to move.\n");
			fly_forward();
		}
		else
		{
			sd_card.write("Direction correct, in safe radius.\n");
		}
	}
	else
	{	
		if(in_safe_radius = false)
		{
			sd_card.write("Direction incorrect, not in safe radius, correcting.\n");
			correct_direction();				
		}
		else
		{
			sd_card.write("Direction incorrect, in safe radius.\n");
		}
	}
	
	if(k%3 == 0)
		sensor_package_1();						// write first part of sensor info
	if(k%5 == 0)
		sensor_package_2();						// write first part of sensor info
		
	sd_card.write("\n\n");
	
	k++;
}

/******************************************
*	
*	bool check_coordinates()
*	Check if the balloon is in the safe radius, 
*	Returns 1 if in the safe radius,
*	Returns 0 if not.
*
*	Author: Chuhan Qin
*
********************************************/
bool check_coordinates()	
{
	double longitude = GPS.get_longitude();
	double latitude  = GPS.get_latitude();
	
	if(longitude > TARGET_LONGTITUDE - SAFETY_LONGTITUDE && longitude < TARGET_LONGTITUDE + SAFETY_LONGTITUDE
		&&
	   latitude  > TARGET_LATITUDE   - SAFETY_LATITUDE   && latitude  < TARGET_LATITUDE   + SAFETY_LATITUDE  )
		return 1;
	else
		return 0;
}

/******************************************
*	
*	void correct_direction()
*	Correct the balloon to face the destination,
*	The balloon chooses the smaller angle to turn.
*
*	Author: Chuhan Qin
*
********************************************/
void correct_direction()
{
	Time correction_start_time = Time.now();
	while(!check_direction())
	{	
		if(angle_left < angle_right)
			turn_left();
		else
			turn_right();
	}
	Time duration = correction_start_time - Time.now();
	sd_card.write("Direction corrected, duration: %s\n", duration);
}

/******************************************
*	
*	void sensor_package()
*	Log a whole bunch of sensor data
*
*	Author: Chuhan Qin
*
********************************************/
void sensor_package_1()
{
	sd_card.write(temperature());		// log temperature
	sd_card.write(barometer());			// log air pressure
}
void sensor_package_2()
{
	sd_card.write(accelerometer());		// log acceleration
	sd_card.write(GPS());				// log coordinates
}

/******************************************
*	
*	int check_flight_duration()
*	Return flight duration in minutes
*
*	Author: Chuhan Qin
*
********************************************/
int check_flight_duration()
{
	Time flight_duration = start_time - Time.now();
	return flight_duration.minutes();
}

/******************************************
*	
*	bool able_to_fight_wind()
*	Check if propellers are efficient enough to fight the wind
*	If not, it's a good indicator for landing.
*
*	Param: 
*	
*
*	Author: Chuhan Qin
*
********************************************/
bool able_to_fight_wind(double latitude_previous, double longitude_previous)
{
	double latitude = GPS.get_latitude()
}

void fly_forward()
{
	sd_card.write("Flying forward at: ")	// runs at certain percentage
	// runs for a few seconds
}