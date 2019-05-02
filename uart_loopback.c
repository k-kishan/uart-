    	#include <stdio.h>
	#include <string.h>
    	#include <fcntl.h>   /* File Control Definitions           */
    	#include <termios.h> /* POSIX Terminal Control Definitions */
    	#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
    	#include <errno.h>   /* ERROR Number Definitions           */

    	int main(void)
    	{
        	int fd;/*File Descriptor*/
		
		printf("\n +----------------------------------+");
		printf("\n |        Serial Port Write         |");
		printf("\n +----------------------------------+");

		/*------------------------------- Opening the Serial Port -------------------------------*/

		/* Change /dev/ttyUSB0 to the one corresponding to your system */

        	fd = open("/dev/ttyHS1",O_RDWR | O_NOCTTY | O_NDELAY);	/* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
			   						/* O_RDWR Read/Write access to serial port           */
									/* O_NOCTTY - No terminal will control the process   */
									/* O_NDELAY -Non Blocking Mode,Does not care about-  */
									/* -the status of DCD line,Open() returns immediatly */                                        
									
        	if(fd == -1)						/* Error Checking */
            	   printf("\n  Error! in Opening ttyHS1  ");
        	else
            	   printf("\n  ttyHS1 Opened Successfully ");

	
		/*---------- Setting the Attributes of the serial port using termios structure --------- */
		
		struct termios SerialPortSettings;	/* Create the structure                          */

		tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

		cfsetispeed(&SerialPortSettings,B115200); /* Set Read  Speed as 9600                       */
		cfsetospeed(&SerialPortSettings,B115200); /* Set Write Speed as 9600                       */

		SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
		SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
		SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
		SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */
	
		SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
		SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 
		
		
		SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
		SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

		SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

                SerialPortSettings.c_cc[VMIN] = 1; /* Read at least 10 characters */
                SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinetly   */

		if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
		    printf("\n  ERROR ! in Setting attributes");
		else
                    printf("\n  BaudRate = 115200 \n  StopBits = 1 \n  Parity   = none\n");
			
	        /*------------------------------- Write data to serial port -----------------------------*/

		char write_buffer[] ="uart test...";	/* Buffer containing characters to write into port	     */	
		int  bytes_written  = 0;  	/* Value for storing the number of bytes written to the port */ 
        
		char read_buffer[128]= ;  /* Buffer to store the data received              */
		int  bytes_read = 0;    /* Number of bytes read by the read() system call */
 
/*	       while (1) 
		{
               	 if ((bytes_written = read(fd, write_buffer,sizeof(write_buffer))) > 0) 
		
		 {*/
			if ((bytes_written = write(fd,write_buffer,strlen(write_buffer))) > 0) 
		  	{
		        	printf("%s", write_buffer);
                  	}


			 bytes_read = read(fd,&read_buffer,127); 
				 printf("done.. %s\n", read_buffer);
                

		close(fd);/* Close the Serial port */

    	}
