//Library for reading from Nano17 6-axis force/torque sensors in a separate thread

#include "nano17lib.h"

/*************************** 
COMMUNICATION SETUP COMMANDS
****************************/
// Communication Data Mode
// CD A setup communication for ascii output mode (default)
void setup_ascii_output(PORT *p){
  serialWriteString(p,"CD A\r");
}

// CD B setup communication for binary output mode
void setup_bin_output(PORT *p){
  serialWriteString(p,"CD B\r");
}

// Communication Checksum
// CD E enable a checksum at the end of binary communication
void enable_checksum(PORT *p){
  serialWriteString(p,"CD E\r");
}

// CD U disable sending checksum at end of binary communication (default)
void disable_checksum(PORT *p){
  serialWriteString(p,"CD U\r");
}

// Communication Data Type
// CD D setup communication for decimal strain gauge data output
void setup_dec_out(PORT *p){
  serialWriteString(p,"CD D\r");
}

// CD H setup communication for hexadecimal strain gauge data output
void setup_hex_out(PORT *p){
  serialWriteString(p,"CD H\r");
}

// CD R setup communication for resolved force/torque data output (default)
void setup_ft_out(PORT *p){
  serialWriteString(p,"CD R\r");
}

// Other Communication Setup Commands
// CB d Changes RS-232 serial baud rate (d = new baud rate)
void change_baud(PORT *p, int d){
  char* buffer;
  sprintf(buffer,"CB %d\r",d);
  serialWriteString(p,buffer);
}

// CE b Enables error display on front panel during error condition (b=1; Default).
void set_err_display(PORT *p, int b){
  assert(b==0||b==1);
  char* buffer;
  sprintf(buffer,"CE %d\r",b);
  serialWriteString(p,buffer);
}

// CS b Enables display of baud rate on front panel during power on (b=1; Default).
void set_baud_display(PORT *p, int b){
  assert(b==0||b==1);
  char* buffer;
  sprintf(buffer,"CS %d\r",b);
  serialWriteString(p,buffer);
}

// CL b Enables Linefeed <LF> with <CR> (b=1; Default) or disable <LF> output (b=0).
void set_linefeed(PORT *p, int b){
  assert(b==0||b==1);
  char* buffer;
  sprintf(buffer,"CL %d\r",b);
  serialWriteString(p,buffer);
}

// CV h Selects components of F/T values to be processed (Fr,Fx,Fy,Fz,Tr,Tx,Ty,Tz).
void select_proc_ft(PORT *p, int h){
  assert(h>=0&&h<=7);
  char* buffer;
  sprintf(buffer,"CV %d\r",h);
  serialWriteString(p,buffer);
}
// CR d Sets the analog output voltage range to either ±5V or ±10V (d=5; Default)
void set_output_range(PORT *p, int d){
  assert(d==5||d==-5||d==10||d==-10);
  char* buffer;
  sprintf(buffer,"CR %d\r",d);
  serialWriteString(p,buffer);
}

/***************************
QUERY DATA REQUESTS
****************************/
//Query F/T and Strain Gauge Data
//QR Query output of one Record of data in pre-selected communication setup.
void query_single(PORT *p){
  serialWriteString(p,"QR\r");
}

//^T Single record output with minimized handshaking; similar to QR.
void query_single_simple(PORT *p){
  serialWriteString(p,"\20");
}

//QS Query output of a Stream of data in pre-selected type and mode.
void query_stream(PORT *p){
  serialWriteString(p,"QS\r");
}

//QT Query output of one record of resultant axes Fr and Tr.
void query_axes(PORT *p){
  serialWriteString(p,"QT\r");
}

/***************************
SENSOR COMMANDS
****************************/
//Sensor Bias
//SB Performs a Sensor Bias. Stores bias reading in a 3-level buffer.
void perform_bias(PORT *p){
  serialWriteString(p,"SB\r");
}

//SU Performs a Sensor Unbias. Removes last bias command from buffer.
void undo_bias(PORT *p){
  serialWriteString(p,"SU\r");
}

//SZ Removes all previously stored biases from buffer.
void empty_biases(PORT *p){
  serialWriteString(p,"SZ\r");
}

//Sensor Peaks (see QP command)
//QP Query Peaks: show the maximum and minimum F/T values collected (see SP).
void query_peaks(PORT *p){
  serialWriteString(p,"QP\r");
}

//SP b Collects the max. and min. F/T values: start (b=1) or stop (b=0; Default).
void collect_peaks(PORT *p, int b){
  assert(b==0||b==1);
  char* buffer;
  sprintf(buffer,"SP %d\r",b);
  serialWriteString(p,buffer);
}

//SC Clear max. and min. F/T values by loading high minimum and maximum values.
void clear_peaks(PORT *p){
  serialWriteString(p,"SC\r");
}

//Other Sensor Commands
//CF d Controls automatic SF optimization for RS-232 output. (d=0; Default).
void SF_optimization(PORT *p, int d){
  char* buffer;
  sprintf(buffer,"CF %d\r",d);
  serialWriteString(p,buffer);
}

//SA d Performs a moving average of d sensor data samples (d=0; Default).
void sensor_avg(PORT *p, int d){
  char* buffer;
  sprintf(buffer,"SA %d\r",d);
  serialWriteString(p,buffer);
}

//SF d Sensor sampling Frequency allows optimizing for faster output when using CF.
void sampling_freq(PORT *p, int d){
  char* buffer;
  sprintf(buffer,"SF %d\r",d);
  serialWriteString(p,buffer);
}

//SM b Sensor Monitoring: disables (b=0) error message due to sensor error (saturation, disconnected transducer etc.) or enables error message (b=1; Default).
void sensor_monitoring(PORT *p, int b){
  assert(b==0||b==1);
  char* buffer;
  sprintf(buffer,"SM %d\r",b);
  serialWriteString(p,buffer);
}

//SR b Sets sensor power protection error (b=1; Default)
void set_power_protection(PORT *p, int b){
  assert(b==0||b==1);
  char* buffer;
  sprintf(buffer,"SR %d\r",b);
  serialWriteString(p,buffer);
}

/***************************
DISCRETE I/O COMMANDS
****************************/
//I/O Verification
//ID Reads and displays the state of all discrete input lines.
void get_input_state(PORT *p){
  serialWriteString(p,"ID\r");
}
//OD h Sets the state of all discrete outputs as specified by hexadecimal number h.
void set_input_state(PORT *p, int h){
  char* buffer;
  sprintf(buffer,"OD %x\r",h);
  serialWriteString(p,buffer);
}

//Force Monitor Commands
//MC s Creates a force Monitor statement s.
void create_force_monitor_statement(PORT *p, char* s){
  char* buffer;  
  sprintf(buffer,"MC %s\r",s);
  serialWriteString(p,buffer);
}

//MD d Deletes a force Monitor statement d.
void delete_force_monitor_statement(PORT *p, int d){
  char* buffer;
  sprintf(buffer,"MD %d\r",d);
  serialWriteString(p,buffer);
}

//MH b Sets monitor handshake mode to b (b=0; Default â require handshaking)
void set_monitor_handshake(PORT *p, int b){
  char* buffer;
  sprintf(buffer,"MH %d\r",b);
  serialWriteString(p,buffer);
}

//ML List all stored Force Monitor statements.
void list_all_monitor(PORT *p){
  serialWriteString(p,"ML\r");
}

//MV h Selects axes for resultants Fr and Tr.
void select_resultant_axes(PORT *p, int h){
  char* buffer;
  sprintf(buffer,"MV %x\r",h);
  serialWriteString(p,buffer);
}

/***************************
TOOL FRAME COMMANDS
****************************/
//TD d Delete tool frame (d=1, 2 or 3).
void delete_tool_frame(PORT *p, int d){
  char* buffer;
  sprintf(buffer,"TD %d\r",d);
  serialWriteString(p,buffer);
}
//TC d,s,x,y,z,R,P,Y 
//Constructs a new tool frame by changing the coordinate system (d=0..3;
//s=name; x, y, and z = translation along X, Y and Z axes; R, P, and Y = rotation
//about X, Y and Z axes).
void new_tool_frame(PORT *p, int d, char* s, int x, int y, int z, int R, int P, int Y){
  char* buffer;
  sprintf(buffer,"TC %d %s %d %d %d %d %d %d\r",d,s,x,y,z,R,P,Y);
  serialWriteString(p,buffer);
}

//TF d Selects a calibration matrix from tool frame list (d=0, 1, 2 or 3).
void select_calibration_matrix(PORT *p, int d){
  char* buffer;
  sprintf(buffer,"TF %d\r",d);
  serialWriteString(p,buffer);
}

//TL List available tool frames.
void list_tool_frames(PORT *p){
  serialWriteString(p,"TL\r");
}

//TT d Reports transducer serial number for tool frame d.
void get_frame_serial(PORT *p, int d){
  char* buffer;
  sprintf(buffer,"TT %d\r",d);
  serialWriteString(p,buffer);
}

//TU Reports torque distance unit scaling.
void get_unit(PORT *p){
  serialWriteString(p,"TU\r");
}

/***************************
MISCELLANEOUS COMMANDS
****************************/
//^W Warm start. Performs a system reset and is identical to pressing the reset button.
void warm_start(PORT *p){
  serialWriteString(p,"\23\r");
}

//^Q, ^S XON and XOFF.
void XON(PORT *p){
  serialWriteString(p,"\17\r");
}

void XOFF(PORT *p){
  serialWriteString(p,"\19\r");
}

//ZC 0, "s" Creates a buffer of commands, s, that are executed at system power-up or reset.
void create_command_buffer(PORT *p, char* s){
  char* buffer;
  sprintf(buffer,"ZC %s\r",s);
  serialWriteString(p,buffer);
}

//RS Save values from run memory into permanent memory.
void save_values(PORT *p){
  serialWriteString(p,"RS\r");
}

//RL Reload values from permanent memory into run memory.
void load_values(PORT *p){
  serialWriteString(p,"RL\r");
}

//% s Comment command. The s entry is ignored.

//HELP Lists a summary of available commands
void get_commands(PORT *p){
  serialWriteString(p,"HELP\r");
}



//read a line from port terminated by term, puts the string in buf and the length read in lineLen
int serialReadLineFast(PORT *port, char *buf, int *lineLen, int term)
{
   int bytesRead;
   int err;

   *lineLen = 0;
   //printf("term:%d\n",term);
   while(1) {
      err = serialRead(port, buf, 1, &bytesRead);
      //printf("bytesRead:%d\n", bytesRead);
      *lineLen += bytesRead;
      buf[bytesRead] = '\0'; // Null terminate
      //printf("serialRead:%s\n", buf);
      if(*buf == term) // If termination character is found
         return(0);
      //printf(".");
      buf += bytesRead;
      usleep(500); // Sleep for .5ms
   }
}


// parse values from single query
// example:
// input: buf="--0, -413, -1080, -2480, -3532, 45, 5890"
// output: values={-413,-1080,-2480,-3532,45,5890}
void parse_values(int* values, char* buf){
  int i,j,k;
  char tmp[100];
  i=0;
  k=0;
  while(buf[i]!=',') i++;
  while(k<5){
    j=0;
    i++;
    while (buf[i]!=',') {
      tmp[j]=buf[i];
      i++;
      j++;
    }
    values[k]=atoi(tmp);
    k++;
    //printf("%d\t",values[k]);
  }
  j=0;
  i++;
  while (buf[i]!='\n'){
    tmp[j]=buf[i];
    i++;
    j++;
  }
  values[k]=atoi(tmp);
  //printf("%d\n",values[k]);
}

//after sending a command, print the echoes
void get_echo(Nano17Struct *nano){
	char buf[255];
	int lineLen;
	int i;
	for(i=0; i<5; i++){
		serialReadLineFast(&(nano->port),buf,&lineLen,'\n');
		printf("echo%d: %s\n", i, buf);
	}
}

// thread worker function that keeps updating the readings
void *nano17_reader(void *arg){
	Nano17Struct *nano = (Nano17Struct *)arg;

  char buf[255];
  int lineLen;
  while(!nano->doneflag){
		serialReadLineFast(&(nano->port),buf,&lineLen,'\n');
     
    if (lineLen<57) {
      printf("Corrupted reading, line: %s\n", buf); //usually saturation
			query_stream(&(nano->port));
			get_echo(nano);
			serialReadLineFast(&(nano->port),buf,&lineLen,'\n');
			printf("next line: %s\n", buf);
    }
		else{
			pthread_mutex_lock(&(nano->mutex));
			parse_values(nano->values,buf);
			pthread_mutex_unlock(&(nano->mutex));
		}
  }
  //pthread_exit(NULL);
}


//find the contact location (inputs raw values, outputs location, finger surface normal, and force magnitude)
void process_values(int values[6], double loc[3], double normal[3], double *forcemag){
	double theta,cosT,sinT,Fr,x,y,z,Ffz,Fft,Fx,Fy,Fz,Tx,Ty,Tz,angle,zangle;
  Fx=values[0]/ACTUAL_COUNTS_PER_FORCE;
  Fy=values[1]/ACTUAL_COUNTS_PER_FORCE;
  Fz=values[2]/ACTUAL_COUNTS_PER_FORCE;
  Tx=values[3]/ACTUAL_COUNTS_PER_TORQUE;
  Ty=values[4]/ACTUAL_COUNTS_PER_TORQUE;
  Tz=values[5]/ACTUAL_COUNTS_PER_TORQUE;
  //printf("%d %d %d %d %d %d\n",values[0],values[1],values[2],values[3],values[4],values[5]);
  //printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",Fx,Fy,Fx,Tx,Ty,Tz,ACTUAL_COUNTS_PER_FORCE,ACTUAL_COUNTS_PER_TORQUE);

  //find contact point and normal to finger surface
  theta=atan2(Fy,Fx);
  cosT=cos(theta);
  sinT=sin(theta);
  if (theta<0) {
    angle=360+theta*180/PI;
  }else {
    angle=theta*180/PI;
  }

	*forcemag = sqrt(Fx*Fx + Fy*Fy + Fz*Fz);
	if(*forcemag > .0001){
		//angle from z-axis
		zangle = atan2(sqrt(Fx*Fx + Fy*Fy), -Fz);
		
    //contact on the cyl
		if(zangle >= 1.57){

			x = -RADIUS*cosT;
			y = -RADIUS*sinT;
			normal[0] = -cosT;
			normal[1] = -sinT;
			normal[2] = 0;
			
			double TxminusFz = Tx - Fz*RADIUS*cosT;
			double TyminusFz = Ty + Fz*RADIUS*sinT;
			z = sqrt(TxminusFz*TxminusFz + TyminusFz*TyminusFz) / sqrt(Fx*Fx + Fy*Fy);
			
			//z should be < 0
			if((Fx>0 && Fy>0 && Tx>0 && Ty<0) ||
				 (Fx<0 && Fy<0 && Tx<0 && Ty>0) ||
				 (Fx<0 && Fy>0 && Tx>0 && Ty>0) ||
				 (Fx>0 && Fy<0 && Tx<0 && Ty<0)) z = -z;			
		}
		//contact on the sphere
		else{
			x = -RADIUS*cosT*sin(zangle);
			y = -RADIUS*sinT*sin(zangle);
			z = LENGTH + RADIUS*cos(zangle);
			normal[0] = -cosT*sin(zangle);
			normal[1] = -sinT*sin(zangle);
			normal[2] = cos(zangle);
		}
  }
	else {
		x=0;
		y=0;
    z=0;
	}

	loc[0] = x;
	loc[1] = y;
	loc[2] = z;
  
}


//connect to the Nano17 on serial port 'portname' and return a new Nano17Struct
Nano17Struct *nano17_open(const char *portname){

	Nano17Struct *nano = (Nano17Struct *)malloc(sizeof(Nano17Struct));
	
	int err;
  err=serialOpen(&(nano->port),portname);
  if (err){
    printf("failed to open serial port %s\n",portname);
    return 0;
  }

  serialSetBaud(&(nano->port),9600);

	//send commands to initialize Nano17
  char buf[255];
  int lineLen, i;
	printf("initializing\n");
	serialWriteString(&(nano->port),"\r");
	empty_biases(&(nano->port));
  setup_ft_out(&(nano->port));
  perform_bias(&(nano->port));
  query_stream(&(nano->port));
	for(i=0; i<7; i++){
		serialReadLineFast(&(nano->port),buf,&lineLen,'\n');
		printf("%s\n",buf);
	}
	//read the first sensor readings and print them
	serialReadLineFast(&(nano->port),buf,&lineLen,'\n');
  printf("readings: %s\n",buf);


  nano->doneflag=0;
  int rc;

	//create mutex
	rc = pthread_mutex_init(&(nano->mutex), NULL);
  if (rc){
    printf("ERROR! return code from pthread_mutex_init() is %d\n",rc);
    exit(-1);
  }	

	//create worker thread
  printf("creating thread\n");
	rc = pthread_create(&(nano->thread), NULL, nano17_reader, (void *)nano);
  if (rc){
    printf("ERROR! return code from pthread_create() is %d\n",rc);
    exit(-1);
  }
	
	return nano;
}

//close the Nano17
void nano17_close(Nano17Struct *nano){
	if(nano){
		nano->doneflag = 1;
		pthread_cancel(nano->thread);
		pthread_mutex_destroy(&(nano->mutex));
		//free(nano);
	}	
}

//get the current raw sensor readings
void nano17_getValues(Nano17Struct *nano, int values[6]){
	pthread_mutex_lock(&(nano->mutex));
	int i;
	for(i=0; i<6; i++){
		values[i] = nano->values[i];
	}
	pthread_mutex_unlock(&(nano->mutex));
}

//get the current contact location and force
void nano17_getLocNormalAndForce(Nano17Struct *nano, double loc[3], double normal[3], double *forcemag){
	int values[6];
	nano17_getValues(nano, values);	
	process_values(values, loc, normal, forcemag);
}


