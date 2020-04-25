#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <inttypes.h>

int main(int argc, char** argv){
		int file;
		char *filename = "/dev/i2c-1";
		if ((file = open(filename, O_RDWR)) < 0) {
			/* ERROR HANDLING: you can check errno to see what went wrong */
			perror("Failed to open the i2c bus");
			exit(1);
		}

		int addr = 0b00001010;          // The I2C address of the ADC
		if (ioctl(file, I2C_SLAVE, addr) < 0) {
			printf("Failed to acquire bus access and/or talk to slave.\n");
			/* ERROR HANDLING; you can check errno to see what went wrong */
			exit(1);
		}

		// Write high vel
		u_int8_t vel = atoi(argv[1]); 
		if (write(file,&vel,1) != 1){
			printf("Failed to write velocity\n");
			//exit(1);
		}

		int16_t data;
		while (true) {
			int nb = 0;
			char buf[10] = {};
			if ((nb = read(file,buf,2)) != 2) {
				/* ERROR HANDLING: i2c transaction failed */
				printf("Failed to read from the i2c bus. %d\n", nb);
				//printf("\n\n");
				//exit(-2);
			} else {
					int8_t tics = 0;
					tics |= buf[0];
					printf("Tics %d", tics);
				if (buf[1] != 0b01010101){
					printf("\tError");
				}
					printf("\n");
			}
			usleep(10000);
		}
			

		close(file);
}

