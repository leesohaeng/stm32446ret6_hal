/*
 * my_mpu6050.h
 *
 *  Created on: 2016. 9. 4.
 *      Author: sohae
 */

char    whoAmI[2];
#define MPU6050					0x68<<1  // Device address when ADO = 0
#define RA_WHO_AM_I         	0x75
#define RA_CONFIG           	0x1A
#define RA_USER_CTRL        	0x6A
#define RA_PWR_ADDR   	    	0x6B
#define RA_SMPLRT_DIV	    	0x19
#define RA_GYRO_CONFIG      	0x1B
#define RA_ACCEL_CONFIG     	0x1C
#define RA_INT_PIN_CFG      	0x37
#define RA_ACCEL_XOUT_H     	0x3B
#define RA_ACCEL_XOUT_L     	0x3C
#define RA_ACCEL_YOUT_H     	0x3D
#define RA_ACCEL_YOUT_L     	0x3E
#define RA_ACCEL_ZOUT_H     	0x3F
#define RA_ACCEL_ZOUT_L     	0x40
#define RA_TEMP_OUT_H       	0x41
#define RA_TEMP_OUT_L       	0x42
#define RA_GYRO_XOUT_H      	0x43
#define RA_GYRO_XOUT_L      	0x44
#define RA_GYRO_YOUT_H      	0x45
#define RA_GYRO_YOUT_L      	0x46
#define RA_GYRO_ZOUT_H      	0x47
#define RA_GYRO_ZOUT_L     		0x48
#define AF_SEL 					0x00
#define ACC_HPF 				0x00
#define FS_SEL 					0x00
#define RA_PWR_MGMT_1  			0x00
#define USER_CTRL        		0x00
#define ACC_SELF_TEST_X 		0x00
#define ACC_SELF_TEST_Y		 	0x00
#define ACC_SELF_TEST_Z 		0x00
#define G_SELF_TEST_X 			0x00
#define G_SELF_TEST_Y 			0x00
#define G_SELF_TEST_Z 			0x00
#define PIN_CFG					2
#define CONFIG					4
#define SMPLRT_DIV				19

/*void InitMPU();
//int ReadData(int adress_L, int adress_H);
char reg_value[2];
void WriteData(uint8_t address, uint8_t subAddress, uint8_t data);
int ReadData(int adress_L, int adress_H);
*/
/*
int main()
{
	pc.baud(115200);
	i2c.frequency(400000);
	InitMPU();
*/
/*
	char whoAmI[1] = {0x75};
	i2c.write(MPU6050,whoAmI,1,true);
	char WhoAmI;
	i2c.read(MPU6050,&WhoAmI,1,false);

	pc.printf("I AM :%d\n\r",WhoAmI);*/
/*
  while(1) {
				pc.printf(" ");

		pc.printf("*");
		pc.printf("%i\t",ReadData(RA_ACCEL_XOUT_L ,RA_ACCEL_XOUT_H));
		pc.printf("%i\t",ReadData(RA_ACCEL_YOUT_L ,RA_ACCEL_YOUT_H));
		pc.printf("%i\t",ReadData(RA_ACCEL_ZOUT_L ,RA_ACCEL_ZOUT_H));
		pc.printf("\n\r");
		pc.printf("%i\t",ReadData(RA_GYRO_XOUT_L ,RA_GYRO_XOUT_H));
		pc.printf("%i\t",ReadData(RA_GYRO_YOUT_L ,RA_GYRO_YOUT_H));
		pc.printf("%i\t",ReadData(RA_GYRO_ZOUT_L ,RA_GYRO_ZOUT_H));
		pc.printf("\n\r");

  }
}*/

/*
void InitMPU(){
	WriteData(MPU6050,RA_PWR_ADDR,RA_PWR_MGMT_1);
	wait(0.1);
	WriteData(MPU6050,RA_USER_CTRL,USER_CTRL);
	WriteData(MPU6050, RA_INT_PIN_CFG,PIN_CFG);
	WriteData(MPU6050,RA_SMPLRT_DIV,SMPLRT_DIV);
	int GConfig = G_SELF_TEST_X | G_SELF_TEST_Y | G_SELF_TEST_Z | FS_SEL;
	int AConfig = ACC_SELF_TEST_X | ACC_SELF_TEST_Y | ACC_SELF_TEST_Z | AF_SEL | ACC_HPF;
	WriteData(MPU6050,RA_GYRO_CONFIG,GConfig);
	WriteData(MPU6050,RA_ACCEL_CONFIG,AConfig);

}

int ReadData(int adress_L, int adress_H){
		char data_L;
	char data_H;
	char adress_data_L[1] ;
	char adress_data_H[1] ;
	adress_data_L[0]= adress_L;
	adress_data_H[0]= adress_H;
	i2c.write(MPU6050, adress_data_L, 1,1);
	i2c.read(MPU6050,&data_L, 1,0);
	i2c.write(MPU6050, adress_data_H, 1,1);
	i2c.read(MPU6050,&data_H, 1,0);

	return (int16_t)((data_H<<8) | data_L);

}

 void WriteData(uint8_t address, uint8_t subAddress, uint8_t data)
{
   char data_write[2];
   data_write[0] = subAddress;
   data_write[1] = data;
   i2c.write(address, data_write, 2, false);
} */
