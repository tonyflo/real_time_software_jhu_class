/*
 * Send roll, pitch, and yaw from BBB to Arduino Uno over USB serial
 */

#define CONTROLLER_ENABLED
#define IMU_ENABLED
#define ARDUINO_SERIAL
#define DEBUG
//#define RAW

//Arduino
#include <time.h>

//sleep
#include <unistd.h>

// Serial includes
#include <stdio.h>    // Standard input/output definitions
#include <stdlib.h>
#include <string.h>   // String function definitions
#include <unistd.h>   // for usleep()
#include <getopt.h>
#include "arduino-serial-lib.h"

// controller Socket includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <poll.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/time.h>

// imu socket includes
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>

#include "pid.h"

#include <utility>
#include <algorithm>

// IMU defines
#define PI 3.14159
#define IMU_UDP_REC_BUF_LEN (12*13) // 13 IMU values of length 12 bytes

// Socket defines
#define CONTROLLER_PORT    	(10000)
#define MAXBUFF          	(1024)
#define MAX_CONTROLLER_CONS (1)

// state flags
volatile bool armed_first_time = false;
volatile bool armed = false;
volatile bool initialized_esc = false;

//response sent to controller to acknowledge receipt of data
char ack[2];

// timer variables to allow ESCs to initialized for 5 seconds before being used
static time_t start_time;
static time_t wait_time = 5; //seconds
static time_t cur_time;

// PID
float ypr[3] = {0.0f,0.0f,0.0f}; // yaw pitch roll values
float yprLast[3] = {0.0f, 0.0f, 0.0f};

float ch1, ch2, ch3, ch4, ch5;  // RC channel inputs
float ch1Last, ch2Last, ch4Last, velocityLast;

float velocity;                // global velocity

float pid_roll, pid_pitch;     // motor balances can vary between -100 & 100
float pid_yaw;                 // throttle balance between axes -100:ac , +100:bd

float va, vb, vc, vd;          //velocities
float v_ac, v_bd;              // velocity of axes

#define CONTROL_MIN (-180)
#define CONTROL_MAX (180)

#define PITCH_MIN (-90)
#define PITCH_MAX (90)
#define ROLL_MIN (-180)
#define ROLL_MAX (180)
#define YAW_MIN (-180)
#define YAW_MAX (180)

const float ESC_MIN = 1000.0;
const float ESC_MAX = 2000.0;

//calibration

#define PITCH_P_VAL (0)
#define PITCH_I_VAL (0)
#define PITCH_D_VAL (0)

#define ROLL_P_VAL (0)
#define ROLL_I_VAL (0)
#define ROLL_D_VAL (0)

#define YAW_P_VAL (0)
#define YAW_I_VAL (0)
#define YAW_D_VAL (0)

PID pitchReg(&ypr[1], &pid_pitch, &ch2, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, DIRECT);
PID rollReg(&ypr[2], &pid_roll, &ch1, ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, DIRECT);
PID yawReg(&ypr[0], &pid_yaw, &ch4, YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);

template<typename tVal>
tVal map_value(std::pair<tVal,tVal> a, std::pair<tVal, tVal> b, tVal inVal)
{
    tVal inValNorm = inVal - a.first;
    tVal aUpperNorm = a.second - a.first;
    tVal normPosition = inValNorm / aUpperNorm;
    
    tVal bUpperNorm = b.second - b.first;
    tVal bValNorm = normPosition * bUpperNorm;
    tVal outVal = b.first + bValNorm;
    
    return outVal;
}

std::pair<float,float> control_range(CONTROL_MIN,CONTROL_MAX), pr_range(-30, 30), pwm_range(ESC_MIN ,ESC_MAX);

void computePID()
{
    ch1 = map_value(control_range, pr_range, ch1);
    ch2 = map_value(control_range, pr_range, ch2);
    
    ch1Last = ch1;
    ch2Last = ch2;
    ch4Last = ch4;

    //If command is BIG, ignore it
    if(abs(ypr[0]-yprLast[0])>30) ypr[0] = yprLast[0];
    if(abs(ypr[1]-yprLast[1])>30) ypr[1] = yprLast[1];
    if(abs(ypr[2]-yprLast[2])>30) ypr[2] = yprLast[2];

    yprLast[0] = ypr[0];
    yprLast[1] = ypr[1];
    yprLast[2] = ypr[2];

    pitchReg.Compute();
    rollReg.Compute();
    yawReg.Compute();
#ifdef DEBUG    
#ifdef RAW
    printf("%f,%f,%f\n", pid_roll, pid_pitch, pid_yaw);
#else
    printf("pid_roll: %f, pid_pitch: %f, pid_yaw: %f\n", pid_roll, pid_pitch, pid_yaw);
#endif // RAW
#endif // DEBUG
}
  
void calculateVelocities()
{
    velocity = map_value(control_range, pwm_range, ch3);
    
    if((velocity < ESC_MIN) || (velocity > ESC_MAX)) velocity = velocityLast;
    velocityLast = velocity;
    
    /*va = std::max(std::min(velocity + pid_roll + pid_yaw, ESC_MAX), ESC_MIN);
    vb = std::max(std::min(velocity + pid_pitch - pid_yaw, ESC_MAX), ESC_MIN);
    vc = std::max(std::min(velocity - pid_roll + pid_yaw, ESC_MAX), ESC_MIN);
    vd = std::max(std::min(velocity - pid_pitch - pid_yaw, ESC_MAX), ESC_MIN);*/

    va = std::max(std::min(velocity - pid_roll + pid_pitch + pid_yaw, ESC_MAX), ESC_MIN);
    vb = std::max(std::min(velocity - pid_roll - pid_pitch - pid_yaw, ESC_MAX), ESC_MIN);
    vc = std::max(std::min(velocity + pid_roll - pid_pitch + pid_yaw, ESC_MAX), ESC_MIN);
    vd = std::max(std::min(velocity + pid_roll + pid_pitch - pid_yaw, ESC_MAX), ESC_MIN); 
}

void error(char* msg)
{
    fprintf(stderr, "%s\n",msg);
    exit(EXIT_FAILURE);
}

#ifdef ARDUINO_SERIAL
#define buf_max 40 // 0000.0000,0000.0000,0000.0000,0000.0000\n
#define dev_name 13 // /dev/ttyACM0
char buf[buf_max];
#endif //ARDUINO_SERIAL

void disable_motors(int fd) {
	sprintf(buf, "%1.7f,%1.7f,%1.7f,%1.7f\n", 0.0, 0.0, 0.0, 0.0);
	printf("MOTOR (6,9,10,5): %s\n", buf);
	serialport_write(fd, buf);
}

/*
float PITCH_P_VAL;
float PITCH_I_VAL;
float PITCH_D_VAL;

float ROLL_P_VAL;
float ROLL_I_VAL;
float ROLL_D_VAL;
*/

/*****************************************
 *
 *
 * MAIN
 *
 *
 *****************************************/
int main(int argc, char *argv[])
{
/*
	PITCH_P_VAL = atof(argv[0]);
	PITCH_I_VAL = atof(argv[1]);
	PITCH_D_VAL = atof(argv[2]);

	ROLL_P_VAL = atof(argv[3]);
	ROLL_I_VAL = atof(argv[4]);
	ROLL_D_VAL = atof(argv[5]);
*/

	int yes = 1;
	ack[0] = 'qc';

#ifdef ARDUINO_SERIAL
    // serial init
    int fd = -1;
    char serialport[dev_name];
    int baudrate = 9600;  // default
    char eolchar = '\n';
    int timeout = 5000;
    int rc,n;
#endif // ARDUINO_SERIAL

#ifdef CONTROLLER_ENABLED
	// controller socket init
	int i, j, max = 0, controller_socket[MAX_CONTROLLER_CONS], controller_fd;
	size_t len;
	fd_set list;
	char controller_buf[MAXBUFF];
	memset(controller_buf, 0, MAXBUFF);
	struct sockaddr_in sock[MAX_CONTROLLER_CONS];
#endif //CONTROLLER_ENABLED
	
#ifdef IMU_ENABLED
	// imu init
	int imu_socket;
	int imu_n = 0;
	struct sockaddr_in servaddr,cliaddr;
	socklen_t imu_len;
	char mesg[IMU_UDP_REC_BUF_LEN];

	imu_socket=socket(AF_INET,SOCK_DGRAM,0);
	
	bzero(&servaddr,sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr=htonl(INADDR_ANY);
	servaddr.sin_port=htons(4774);
	bind(imu_socket,(struct sockaddr *)&servaddr,sizeof(servaddr));

	int nnn = IMU_UDP_REC_BUF_LEN;
	if (setsockopt(imu_socket, SOL_SOCKET, SO_RCVBUF, &nnn, sizeof(nnn)) == -1) {
		// deal with failure, or ignore if you can live with the default size
		printf("Default socket udp buffer value NOT changed for imu\r\n");
	}
	else
	{
		printf("Default socket udp buffer value changed for imu\r\n");
	}
#endif //IMU_ENABLED
	
#ifdef ARDUINO_SERIAL
    if( fd!=-1 ) {
            serialport_close(fd);
            printf("closed port %s\n",serialport);
    }
	sprintf(serialport, "%s", "/dev/ttyACM0");
	fd = serialport_init(serialport, baudrate);
	if( fd==-1 ) error("couldn't open port");
	printf("opened port %s\n",serialport);
	serialport_flush(fd);
	if( fd == -1 ) error("serial port not opened");
#endif // ARDUINO_SERIAL

#ifdef CONTROLLER_ENABLED
	// imu init
	int cont_socket;
	int cont_n = 0;
	struct sockaddr_in cont_servaddr,cont_cliaddr;
	socklen_t cont_len;
	char cont_mesg[MAXBUFF];

	cont_socket=socket(AF_INET,SOCK_DGRAM,0);
	
	bzero(&cont_servaddr,sizeof(cont_servaddr));
	cont_servaddr.sin_family = AF_INET;
	cont_servaddr.sin_addr.s_addr=htonl(INADDR_ANY);
	cont_servaddr.sin_port=htons(CONTROLLER_PORT);
	bind(cont_socket,(struct sockaddr *)&cont_servaddr,sizeof(cont_servaddr));

	int cont_nnn = MAXBUFF;
	if (setsockopt(cont_socket, SOL_SOCKET, SO_RCVBUF, &cont_nnn, sizeof(cont_nnn)) == -1) {
		// deal with failure, or ignore if you can live with the default size
		printf("Default socket udp buffer value NOT changed for controller\r\n");
	}
	else
	{
		printf("Default socket udp buffer value changed for controller\r\n");
	}
#endif //CONTROLLER_ENABLED

#ifdef TCP_CONTROLLER_ENABLED	
	/*
	 * We will loop through each file descriptor. First,
	 * we will create a socket bind to it and then call 
	 * listen. If we get and error we simply exit, 
	 * which is fine for demo code, but not good in the
	 * real world where errors should be handled properly. 
	 */
	for( i = 0; i < MAX_CONTROLLER_CONS; i++ )
	{
		/* check to see that we can create them */
		if( (controller_socket[i] = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0 )
		{
			perror("Cannot create socket");
			exit(1);
		}

		if ( setsockopt(controller_socket[i], SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1 )
		{
			perror("setsockopt failed for controller immediacy of socket reuse address");
		} else {
			printf("set controller socket to be immediately reusable\r\n");
		}

		struct timeval tv;
		tv.tv_sec = 0.1; // 100 ms timeout
		tv.tv_usec = 0;  // Not init'ing this can cause strange errors

		if(setsockopt(controller_socket[i], SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval)) == -1) {
			perror("setsockopt failed for controller recv timeout");
		} else {
			printf("set controller socket recv() timeout\r\n");
		}

		/* now fill out the socket stuctures */
		sock[i].sin_family = AF_INET;
		sock[i].sin_addr.s_addr = htonl(INADDR_ANY);
		sock[i].sin_port = htons(CONTROLLER_PORT + i);

		if( bind(controller_socket[i], (struct sockaddr *) &sock[i], sizeof(struct sockaddr_in)) < 0 )
		{
			perror("Can't bind to the socket");
		}

		/* set the socket to the listen state */
		if( listen(controller_socket[i], 1) < 0 )
		{
			perror("Failed to listen on the socket");
		}
	}/* for */
#endif //TCP_CONTROLLER_ENABLED	

	/*****************************************
	 *
	 *
	 * MAIN LOOP
	 *
	 *
	 *****************************************/
	char * err_str = "";
	long long error_count = 0;
	
	while(1)
	{
		float commands[9]; //number of raw controller values
#ifdef TCP_CONTROLLER_ENABLED
check_controller_again: //goto tag that jumps here until the controller connection is established
	for( i =0; i < MAX_CONTROLLER_CONS; i++ )
	{
		len = sizeof(struct sockaddr_in);
		printf("Waiting for new TCP packet via accept()...\n");
		if((controller_fd = accept(controller_socket[i], (struct sockaddr *)&sock[i], &len)) < 0)
		{
			printf("Controller accept() error\r\n");
			disable_motors(fd);
			goto check_controller_again;
		}
		
		printf("Waiting for new TCP packet via recv()...\n");
		int recvMsgSize;
		if ((recvMsgSize = recv(controller_fd, controller_buf, MAXBUFF, 0)) < 0) {
			printf("Controller recv() timeout\r\n");
			disable_motors(fd);
			goto check_controller_again;
		}
		sleep(0.02);
		write(controller_fd, ack, 2); //2 is a 2 byte acknowledgement
		close(controller_fd);
#ifdef DEBUG
		printf("CONTROL:%s", controller_buf);
#endif //DEBUG
		// parse controller data
		char seps[] = ",";
		char* token;
		float var;
		int i = 0;

		token = strtok (controller_buf, seps);
		while (token != NULL)
		{
			sscanf (token, "%f", &var);
			commands[i++] = var;

			token = strtok (NULL, seps);
		}
		
		// give values to the pid
		ch2 = commands[3];
		ch1 = commands[4];
		ch4 = commands[2];
		ch3 = commands[1];

		//check for STOP/QUIT
		if(commands[0] == -1)
		{
			#ifdef ARDUINO_SERIAL
			printf("STOP/QUIT\r\n");
			sprintf(buf, "%1.7f,%1.7f,%1.7f,%1.7f\n", 0.0, 0.0, 0.0, 0.0);
			rc = serialport_write(fd, buf);
			shutdown(imu_socket, SHUT_RDWR);
			close(imu_socket);
			shutdown(fd, SHUT_RDWR);
			close(fd);
			#endif // ARDUINO_SERIAL
			return 0;
		}
		//check for DISARM
		else if(commands[0] == 0)
		{
			//printf("DISARM\r\n");
			armed = false;	
			initialized_esc = false;
			armed_first_time = false;
			wait_time = 5;
			velocity = 0.0; 
			pid_roll = 0.0; 
			pid_pitch = 0.0; 
			pid_yaw = 0.0;
			va = 0.0;
			vb = 0.0; 
			vc = 0.0; 
			vd = 0.0;
			v_ac = 0.0; 
			v_bd = 0.0;   
		}
		//check for ARM
		else if(commands[0] == 1)
		{
			if(!armed_first_time)
			{
				printf("===== ARMED FIRST TIME =======================\r\n");
				armed = true;
				wait_time += time(NULL);
				armed_first_time = true;
			}
		} else {
			printf("UNKNOWN STATE\r\n");
		}
	} /* for */
#endif //TCP_CONTROLLER_ENABLED

#ifdef CONTROLLER_ENABLED
		sleep(0.1);
		cont_len = sizeof(cont_cliaddr);
		cont_n = recvfrom(cont_socket,cont_mesg,MAXBUFF,NULL,(struct sockaddr *)&cont_cliaddr,&cont_len);
			cont_mesg[cont_n] = 0;
#ifdef DEBUG
#ifdef RAW
			printf("%s\n", cont_mesg);
#else
			printf("CONTROL:%s\n", cont_mesg);
#endif //RAW
#endif //DEBUG
		if(cont_n > 0)
		{
			// parse controller data
			char cont_seps[] = ",";
			char* cont_token;
			float cont_var;
			int cont_i = 0;

			cont_token = strtok (cont_mesg, cont_seps);
			while (cont_token != NULL)
			{
				sscanf (cont_token, "%f", &cont_var);
				commands[cont_i++] = cont_var;

				cont_token = strtok (NULL, cont_seps);
			}

			// give values to the pid
			ch2 = commands[3];
			ch1 = commands[4];
			ch4 = commands[2];
			ch3 = commands[1];
			
			//check for STOP/QUIT
			if(commands[0] == -1)
			{
				#ifdef ARDUINO_SERIAL
				printf("STOP/QUIT\r\n");
				shutdown(imu_socket, SHUT_RDWR);
				close(imu_socket);
				shutdown(fd, SHUT_RDWR);
				close(fd);
				#endif // ARDUINO_SERIAL
				return 0;
			}
			//check for DISARM
			else if(commands[0] == 0)
			{
				//printf("DISARM\r\n");
				armed = false;	
				initialized_esc = false;
				armed_first_time = false;
				wait_time = 5;
				velocity = 0.0; 
				pid_roll = 0.0; 
				pid_pitch = 0.0; 
				pid_yaw = 0.0;
				va = 0.0;
				vb = 0.0; 
				vc = 0.0; 
				vd = 0.0;
				v_ac = 0.0; 
				v_bd = 0.0;   
			}
			//check for ARM
			else if(commands[0] == 1)
			{
				if(!armed_first_time)
				{
					printf("===== ARMED FIRST TIME =======================\r\n");
					armed = true;
					wait_time += time(NULL);
					armed_first_time = true;
				}
			} else {
				printf("UDP data not available\r\n");
			}
		} // end if no datagram data
#endif // CONTROLLER_ENABLED

#ifdef IMU_ENABLED
		imu_len = sizeof(cliaddr);
		imu_n = recvfrom(imu_socket,mesg,IMU_UDP_REC_BUF_LEN,MSG_DONTWAIT,(struct sockaddr *)&cliaddr,&imu_len);
		mesg[imu_n] = 0;

		char seps[] = ",";
		char* token;
		float var;
		float angles[3]; //number of raw imu values
		int i = 0;

		token = strtok (mesg, seps);
		while (token != NULL)
		{
			sscanf (token, "%f", &var);
			angles[i++] = var;

			token = strtok (NULL, seps);
		}
		
		ypr[0] = angles[0];
		ypr[1] = angles[1];
		ypr[2] = angles[2];
		
		ypr[0] = ypr[0] * 180/PI;
		ypr[1] = ypr[1] * 180/PI;
		ypr[2] = ypr[2] * 180/PI;
#ifdef DEBUG		
#ifdef RAW
		printf("%f,%f,%f\n", ypr[0], ypr[1], ypr[2]);
#else
		printf("IMU (ypr): %f,%f,%f\n", ypr[0], ypr[1], ypr[2]);
#endif //RAW
#endif //DEBUG

#endif // IMU_ENABLED
		
		//PID
		computePID();
		calculateVelocities();

#ifdef ARDUINO_SERIAL
		memset(buf, 0, buf_max);
		if(armed) {
			if(initialized_esc)
			{
				//write actual pid values to the motor after intial wait
				sprintf(buf, "%4.4f,%4.4f,%4.4f,%4.4f\n", vb, vc, vd, va);
			} else{
				cur_time = time(NULL);
				if(cur_time > wait_time)
				{
#ifdef DEBUG
					printf("===== INITIALIZED ESC =======================\r\n");
#endif //DEBUG
					initialized_esc = true;		
				}
				sprintf(buf, "%4.4f,%4.4f,%4.4f,%4.4f\n", 1000.0, 1000.0, 1000.0, 1000.0);
			}
		} else {
			// disarmed
			sprintf(buf, "%1.7f,%1.7f,%1.7f,%1.7f\n", 0.0, 0.0, 0.0, 0.0);
		}
		
#ifdef DEBUG
#ifdef RAW
		printf("%s\n", buf);
#else
		printf("MOTOR (6,9,10,5): %s\n", buf);
#endif //RAW
#endif //DEBUG
		rc = serialport_write(fd, buf);

		if(rc==-1)
		{
			printf("serialport_write error #:%d\r\n", rc);
			error("error writing");
			perror("error");
		}
#endif // ARDUINO_SERIAL

	
	
	} // end while(1)

    exit(EXIT_SUCCESS);
} // end main
