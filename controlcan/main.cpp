/*************************************************************************
    > Function: Radar raw data display
    > Author: HengZhang
    > Date: 2018.11.29 
*************************************************************************/
#include <stdio.h>
#include <sys/types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"
#include <string.h>
#include <ctime>
#include <cstdlib>
#include "unistd.h"

using namespace std;
using namespace cv;

VCI_BOARD_INFO pInfo;//obtain device information


void *receive_func(void* param) //receive thread
{
	int reclen=0;
	VCI_CAN_OBJ rec[3000];//receive cache, set to 3000 is preferable.
	int i,j;
	int count=0;
	int *run=(int*)param;//thread startup, exit control.
    int ind=0;
	
	while((*run)&0x0f)
	{
		if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//Call the receiving function, if there is data, display the data and return the number of frames actually read.
		{
			printf("reclen:%d\n",reclen);
			for(j=0;j<reclen;j++)
			{
				printf("Index:%04d  ",count);count++;
				printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
				if(rec[j].ExternFlag==0) printf(" Standard ");
				if(rec[j].ExternFlag==1) printf(" Extend   ");
				if(rec[j].RemoteFlag==0) printf(" Data   ");
				if(rec[j].RemoteFlag==1) printf(" Remote ");
				printf("DLC:0x%02X",rec[j].DataLen);
				printf(" data:0x");	
				for(i = 0; i < rec[j].DataLen; i++)
				{
					printf(" %02X", rec[j].Data[i]);
				}
				printf(" TimeStamp:0x%08X",rec[j].TimeStamp);
				printf("\n");
	        }		
		}
	}

	//else 
	printf("run thread exit\n");//exit thread
	pthread_exit(0);
}

int main()
{
	printf(">>this is hello !\r\n");
	
	if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)
	{
		printf(">>open deivce success!\n");
	}
	else
	{
		printf(">>open deivce error!\n");
		exit(1);
	}

	if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)
	{
        printf(">>Get VCI_ReadBoardInfo success!\n");
		//printf(" %08X", pInfo.hw_Version);printf("\n");
		//printf(" %08X", pInfo.fw_Version);printf("\n");
		//printf(" %08X", pInfo.dr_Version);printf("\n");
		//printf(" %08X", pInfo.in_Version);printf("\n");
		//printf(" %08X", pInfo.irq_Num);printf("\n");
		//printf(" %08X", pInfo.can_Num);printf("\n");
		printf(">>Serial_Num:%c", pInfo.str_Serial_Num[0]);
		printf("%c", pInfo.str_Serial_Num[1]);
		printf("%c", pInfo.str_Serial_Num[2]);
		printf("%c", pInfo.str_Serial_Num[3]);
		printf("%c", pInfo.str_Serial_Num[4]);
		printf("%c", pInfo.str_Serial_Num[5]);
		printf("%c", pInfo.str_Serial_Num[6]);
		printf("%c", pInfo.str_Serial_Num[7]);
		printf("%c", pInfo.str_Serial_Num[8]);
		printf("%c", pInfo.str_Serial_Num[9]);
		printf("%c", pInfo.str_Serial_Num[10]);
		printf("%c", pInfo.str_Serial_Num[11]);
		printf("%c", pInfo.str_Serial_Num[12]);
		printf("%c", pInfo.str_Serial_Num[13]);
		printf("%c", pInfo.str_Serial_Num[14]);
		printf("%c", pInfo.str_Serial_Num[15]);
		printf("%c", pInfo.str_Serial_Num[16]);
		printf("%c", pInfo.str_Serial_Num[17]);
		printf("%c", pInfo.str_Serial_Num[18]);
		printf("%c", pInfo.str_Serial_Num[19]);
		printf("\n");
		printf(">>hw_Type:%c", pInfo.str_hw_Type[0]);
		printf("%c", pInfo.str_hw_Type[1]);
		printf("%c", pInfo.str_hw_Type[2]);
		printf("%c", pInfo.str_hw_Type[3]);
		printf("%c", pInfo.str_hw_Type[4]);
		printf("%c", pInfo.str_hw_Type[5]);
		printf("%c", pInfo.str_hw_Type[6]);
		printf("%c", pInfo.str_hw_Type[7]);
		printf("%c", pInfo.str_hw_Type[8]);
		printf("%c", pInfo.str_hw_Type[9]);printf("\n");	
	}
	else
	{
		printf(">>Get VCI_ReadBoardInfo error!\n");
		exit(1);
	}

	//Initialization parameters, strict parameters secondary development function library instructions.
	VCI_INIT_CONFIG config;
	config.AccCode=0;
	config.AccMask=0xFFFFFFFF;
	config.Filter=1;//receive all frames
	config.Timing0=0x00;/*baud rate:500 Kbps  0x00  0x1C*/
	config.Timing1=0x1C;
	config.Mode=0;//normal mode	
	
	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
	if(VCI_ClearBuffer(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Clearbuffer CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
	
	//frames to be sent to radar, and structure settings
	//0x00000200
	VCI_CAN_OBJ send[1];
	send[0].ID=0x00000200;
	send[0].SendType=1;//SendType = 1, to improve the response speed of sending
	send[0].ExternFlag=0;
	send[0].DataLen=8;
	
	int i=0;
	int count1 =0;
	send[0].Data[0] = 0xBD;//Stores the current configuration to non-volatile memory to be read and set at sensor startup
	send[0].Data[1] = 0x19;//MaxDistance:200
	send[0].Data[2] = 0x00;
	send[0].Data[3] = 0x00;
	send[0].Data[4] = 0x68;
	send[0].Data[5] = 0x8C;//0C:receive extended information
	send[0].Data[6] = 0x01;
	send[0].Data[7] = 0x00;

	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
	{
		printf("Index:%04d  ",count1);count1++;
		printf("CAN1 TX ID:0x%08X",send[0].ID);
		if(send[0].ExternFlag==0) printf(" Standard ");
		if(send[0].ExternFlag==1) printf(" Extend   ");
		if(send[0].RemoteFlag==0) printf(" Data   ");
		if(send[0].RemoteFlag==1) printf(" Remote ");
		printf("DLC:0x%02X",send[0].DataLen);
		printf(" data:0x");

		for(i=0;i<send[0].DataLen;i++)
		{
			printf(" %02X",send[0].Data[i]);
		}

		printf("\n");
	}

	int m_run0=1;
	pthread_t threadid;
	int ret;
	ret=pthread_create(&threadid,NULL,receive_func,&m_run0);
		
	usleep(10000000);// Delay Unit, where 10000000=10s 10s is set to close the receiving thread and exit the main program.
	//m_run0=0;// Waiting for the thread to close.
	pthread_join(threadid,NULL);
	usleep(100000);
	VCI_ResetCAN(VCI_USBCAN2, 0, 0);//reset the CAN1 channel.
	usleep(100000);
	VCI_CloseDevice(VCI_USBCAN2,0);
	return 0;
	
}
/*In addition to the transceiver function, it is better to add a millisecond delay before and after other function calls, 
  that is, it does not affect the operation of the program, but also allows the USBCAN device to have sufficient time to process instructions.*/