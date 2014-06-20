

/* declare the topic */
ORB_DECLARE(servo_control);
 
/* define the data structure that will be published where subscribers can see it */
struct servo_control_data {
	int r;
};

