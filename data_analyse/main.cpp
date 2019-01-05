/*************************************************************************
    > Function: radar data analysis and processing
    > Author: HengZhang
    > Date: 2018.12.5 
************************************************************************/
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
			//printf("***reclen is %d\n", reclen);
			Mat img = imread("coordinate.png");
			namedWindow("Target display",1);

			for(j=0;j<reclen;j++)
			{
				if (rec[j].ID == 0x0000060A)
				{
					printf("\n\n\n");
					printf("/****************Here is Coming!******************/\n");
					printf("RX ID:0x%08X\n", rec[j].ID);
					unsigned char Object_NofObjects;
					Object_NofObjects=rec[j].Data[0];
					printf("The number of Object is %d.\n",Object_NofObjects);
					
					union{
						unsigned short Object_MeasCounter;
						unsigned char temp[2];
					}trans;
					trans.temp[1] = rec[j].Data[1];
					trans.temp[0] = rec[j].Data[2];
					//printf("the Object_MeasCounter is %d\n",trans.Object_MeasCounter);

				}
				
				else if (rec[j].ID == 0x0000060B)
				{	
					printf("RX ID:0x%08X\n", rec[j].ID);//ID
					int Object_ID;
					Object_ID = rec[j].Data[0];
					printf("Object  ID:%d  ",Object_ID);

					union {
						unsigned short Object_DistLong;
						unsigned short Object_DistLat;
						unsigned short Object_VrelLong;
						unsigned short Object_VrelLat;
						unsigned char  Object_DistLong_tmp[2];
						unsigned char  Object_DistLat_tmp[2];
						unsigned char  Object_VrelLong_tmp[2];
						unsigned char  Object_VrelLat_tmp[2];
					}trans;

					trans.Object_DistLong_tmp[1] = rec[j].Data[1];
					trans.Object_DistLong_tmp[0] = rec[j].Data[2];
					trans.Object_DistLong >>= 3;
					printf("DistLong:%5.2f  ",trans.Object_DistLong * 0.2 - 500);
					float DistLong_y =0;
					DistLong_y = trans.Object_DistLong * 0.2 - 500;
					
					trans.Object_DistLat_tmp[1] = rec[j].Data[2];
					trans.Object_DistLat_tmp[0] = rec[j].Data[3];
					trans.Object_DistLat &= 0x07FF;
					printf("DistLat:%5.2f  ", trans.Object_DistLat * 0.2 - 204.6);
					float DistLong_x =0;
				    DistLong_x = trans.Object_DistLat * 0.2 - 204.6;
 
					trans.Object_VrelLong_tmp[1] = rec[j].Data[4];
					trans.Object_VrelLong_tmp[0] = rec[j].Data[5]; 
					trans.Object_VrelLong &= 0xFFC0;
					trans.Object_VrelLong >>= 6;
					printf("VrelLong:%5.2f  ", trans.Object_VrelLong * 0.25 - 128);
					float VrelLong_y =0;
                    VrelLong_y = trans.Object_VrelLong * 0.25 - 128;

					trans.Object_VrelLat_tmp[1] = rec[j].Data[5];
					trans.Object_VrelLat_tmp[0] = rec[j].Data[6];
					trans.Object_VrelLat &= 0x3FD0;
					trans.Object_VrelLat >>= 5;
					printf("VrelLat:%5.2f  ", trans.Object_VrelLat * 0.25 - 64);
					float VrelLat_x =0;
                    VrelLat_x = trans.Object_VrelLat * 0.25 - 64;

                    unsigned char Object_DynProp;
					Object_DynProp = rec[j].Data[6];
					Object_DynProp &= 0x07;	
					switch(Object_DynProp)
					{
						case 0x0:
							printf("DynProp:moving  ");
							break;
						case 0x1:
							printf("DynProp:stationary  ");
							break;
						case 0x2:
							printf("DynProp:oncoming  ");
							break;
						case 0x3:
							printf("DynProp:stationary candidate  ");
							break;
						case 0x4:
							printf("DynProp:unknown  ");
							break;
						case 0x5:
							printf("DynProp:crossing stationary");
							break;
						case 0x6:
							printf("DynProp:crossing moving  ");
							break;
						case 0x7:
							printf("DynProp:stopped  ");
							break;
						default:
							break;		
					}

					float Object_RCS = 0;
				    Object_RCS = rec[j].Data[7];
				    printf("RCS:%5.2f\n", Object_RCS * 0.5 - 64);

                    // Draw the target to the picture
                    if ((DistLong_x >= -5) && (DistLong_x <= 5) && (DistLong_y >= 0) && (DistLong_y <= 140))
					{
						const float fx = img.cols / 10.0;
						//const float fy = image.rows / 17.0 / 10.0;
						const float fy = 6.5;
						const int zero = 975;
						float circle_x;
						float circle_y;
						char text[100];
						if (1) 
						{
							circle_x = 100 + fx * DistLong_x;
							circle_y = zero - fy * DistLong_y;
							sprintf(text, "%d(%4.2f,%4.2f)", Object_ID,DistLong_y,DistLong_x);
						}
						else {
							circle_x = 100 - fx;
							circle_y = zero;
							sprintf(text, "0");
						}
						circle(img,cvPoint(circle_x,circle_y),4,Scalar( 0, 0, 255),-1,8);
						cv::putText(img, text, cv::Point(circle_x + 4, circle_y - 3), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, CV_RGB(0,0,0), 0.4, 8);
	                    
				    }
	
				}
			
                /*
				else if (rec[j].ID == 0x0000060C)
				{
					printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
					printf("\n");
					int Obj_ID;
					Obj_ID = rec[j].Data[0];
					printf("Obj_ID is %d\n", Obj_ID);

					unsigned char Obj_ProbOfExist;
					Obj_ProbOfExist = rec[j].Data[6];
					Obj_ProbOfExist >>= 5;

					switch(Obj_ProbOfExist)
					{
						case 0x0:
							printf("Obj_ProbOfExist is invalid\n");
							break;
						case 0x1:
							printf("Obj_ProbOfExist is < 0.25\n");
							break;
						case 0x02:
							printf("Obj_ProbOfExist is between 0.25 and 0.5\n");
							break;
						case 0x03:
							printf("Obj_ProbOfExist is between 0.5 and 0.75\n");
							break;
						case 0x04:
							printf("Obj_ProbOfExist is between 0.75 and 0.9\n");
							break;
						case 0x05:
							printf("Obj_ProbOfExist is between 0.9 and 0.99\n");
							break;
						case 0x06:
							printf("Obj_ProbOfExist is between 0.99 and 0.999\n");
							break;
						case 0x07:
							printf("Obj_ProbOfExist is between 0.999 and 1\n");
							break;
						default:
							break;
					}

				}

				else if (rec[j].ID = 0x0000060D)
				{
					printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
					printf("\n");
					int Object_ID = rec[j].Data[0] ;
					printf("Object_ID is %d\n", Object_ID);

					union 
					{
						unsigned short Object_ArelLong;
						unsigned char  Object_ArelLong_tmp[2];
						
					}Object_ArelLong_trans;

					Object_ArelLong_trans.Object_ArelLong_tmp[1] = rec[j].Data[1];
					Object_ArelLong_trans.Object_ArelLong_tmp[0] = rec[j].Data[2];
					Object_ArelLong_trans.Object_ArelLong >>= 5;
					printf("Relative acceleration in longitudinal direction is %f\n", Object_ArelLong_trans.Object_ArelLong * 0.01 -10);

					union
					{
						unsigned short Object_ArelLat;
						unsigned char  Object_ArelLat_tmp[2];
					}Object_ArelLat_trans;

					Object_ArelLat_trans.Object_ArelLat_tmp[1] = rec[j].Data[2];
					Object_ArelLat_trans.Object_ArelLat_tmp[0] = rec[j].Data[3];
					Object_ArelLat_trans.Object_ArelLat &= 0x1FF0;
					Object_ArelLat_trans.Object_ArelLat >>= 4;
					printf("Relative acceleration in lateral direction is %f\n", Object_ArelLat_trans.Object_ArelLat * 0.01 - 2.5);

					unsigned char Object_Class;
					Object_Class = rec[j].Data[3];
					Object_Class &= 0x07;
					switch (Object_Class)
					{
						case 0x0:
							printf("This object is point\n");
							break;
						case 0x1:
							printf("This object is car\n");
							break;
						case 0x2:
							printf("This object is truck\n");
							break;
						case 0x3:
							printf("This object is pedestrian\n");
							break;
						case 0x4:
							printf("This object is motorcycle\n");
							break;
						case 0x5:
							printf("This object is bicycle\n");
							break;
						case 0x6:
							printf("This object is wide\n");
							break;
						case 0x7:
							printf("reserved\n");
							break;
					}

					union 
					{
						unsigned short Object_OrientationAngel;
						unsigned char Object_OrientationAngel_tmp[2];
					}Object_OrientationAngel_trans;
					Object_OrientationAngel_trans.Object_OrientationAngel_tmp[1] = rec[j].Data[4];
					Object_OrientationAngel_trans.Object_OrientationAngel_tmp[0] = rec[j].Data[5];
					Object_OrientationAngel_trans.Object_OrientationAngel >>= 6;
					printf("Orientation angle of the object is %fdeg\n", Object_OrientationAngel_trans.Object_OrientationAngel * 0.4 - 180);

					unsigned char Object_Length;
					Object_Length = rec[j].Data[6];
					printf("Length of the tracked object is %fm\n", Object_Length * 0.2);

					unsigned char Object_Width;
					Object_Width = rec[j].Data[7];
					printf("Width of the tracked object is %fm\n", Object_Width * 0.2);

				}*/
			
	        }
			//display image
	        imshow("Target display", img);	
	        //waiting for the key, any key return on the keyboard
	        waitKey(100);						
		}
	}
	//else 
	printf("run thread exit\n");//exit
	pthread_exit(0);
}

int main()
{
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
	send[0].RemoteFlag=0;
	send[0].ExternFlag=0;
	send[0].DataLen=8;
	
	int i=0;
	int count1 =0;
	send[0].Data[0] = 0x9D;//Stores the current configuration to non-volatile memory to be read and set at sensor startup
	send[0].Data[1] = 0x19;//MaxDistance:200
	send[0].Data[2] = 0x00;
	send[0].Data[3] = 0x00;
	send[0].Data[4] = 0x68;
	send[0].Data[5] = 0x84;//0C:receive extended information
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
	usleep(5000);
    
	//0x00000202 	
    VCI_CAN_OBJ send_Distance[1];
	send_Distance[0].ID=0x00000202;
	send_Distance[0].SendType=1;
	send_Distance[0].RemoteFlag=0;
	send_Distance[0].ExternFlag=0;
	send_Distance[0].DataLen=5;
	
	int count2 =0;
	send_Distance[0].Data[0] = 0x8E;	
	send_Distance[0].Data[1] = 0x00;
	send_Distance[0].Data[2] = 0x00;
	send_Distance[0].Data[3] = 0x03;
	send_Distance[0].Data[4] = 0xE8;//filter:100m;filter 85m,0x03;0x52;filter 200m, 0x07;0xD0;filter:100,0x03;0xE8;
	
	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send_Distance, 1) == 1)
	{
		printf("Index:%04d  ",count2);count2++;
		printf("CAN1 TX ID:0x%08X",send_Distance[0].ID);
		if(send_Distance[0].ExternFlag==0) printf(" Standard ");
		if(send_Distance[0].ExternFlag==1) printf(" Extend   ");
		if(send_Distance[0].RemoteFlag==0) printf(" Data   ");
		if(send_Distance[0].RemoteFlag==1) printf(" Remote ");
		printf("DLC:0x%02X",send_Distance[0].DataLen);
		printf(" data:0x");

		for(int i=0;i<send_Distance[0].DataLen;i++)
		{
			printf(" %02X",send_Distance[0].Data[i]);
		}

		printf("\n");
	}
	usleep(5000);

    //0x0x00000202
	VCI_CAN_OBJ send_Azimuth[1];
	send_Azimuth[0].ID=0x00000202;
	send_Azimuth[0].SendType=1;
	send_Azimuth[0].RemoteFlag=0;
	send_Azimuth[0].ExternFlag=0;
	send_Azimuth[0].DataLen=5;
	
	int count3 =0;
	send_Azimuth[0].Data[0] = 0x96;	
	send_Azimuth[0].Data[1] = 0x00;
	send_Azimuth[0].Data[2] = 0xC8;
	send_Azimuth[0].Data[3] = 0x0E;
	send_Azimuth[0].Data[4] = 0xD8;//filter ±45°(3 lines):0x96,0x00,0xC8,0x0E,0xD8;filter ±10°(1 line):0x96,0x06,0x40,0x09,0x60;filter ±15°:0x96,0x05,0x78,0x0A,0x28;

	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send_Azimuth, 1) == 1)
	{
		printf("Index:%04d  ",count3);count3++;
		printf("CAN1 TX ID:0x%08X",send_Azimuth[0].ID);
		if(send_Azimuth[0].ExternFlag==0) printf(" Standard ");
		if(send_Azimuth[0].ExternFlag==1) printf(" Extend   ");
		if(send_Azimuth[0].RemoteFlag==0) printf(" Data   ");
		if(send_Azimuth[0].RemoteFlag==1) printf(" Remote ");
		printf("DLC:0x%02X",send_Azimuth[0].DataLen);
		printf(" data:0x");

		for(int i=0;i<send_Azimuth[0].DataLen;i++)
		{
			printf(" %02X",send_Azimuth[0].Data[i]);
		}

		printf("\n");
	}
    usleep(5000);

    //0x00000202
	VCI_CAN_OBJ send_RCS[1];
	send_RCS[0].ID=0x00000202;
	send_RCS[0].SendType=1;
	send_RCS[0].RemoteFlag=0;
	send_RCS[0].ExternFlag=0;
	send_RCS[0].DataLen=5;
	
	int count4 =0;
	send_RCS[0].Data[0] = 0xAE;	
	send_RCS[0].Data[1] = 0x06;
	send_RCS[0].Data[2] = 0x40;
	send_RCS[0].Data[3] = 0x0C;
	send_RCS[0].Data[4] = 0x80;//filter RCS:-10dBsm; RCS:-2dBsm 0xAE,0x07,0x80;RCS:5.1dBsm~30dBsm 0xAE,0x08,0x9C,0x0C,0x80;RCS:0dBsm 0xAE,0x07,0xD0;RCS:-15dBsm 0xAE,0x05,0x78;RCS:8dBm2 0xAE,0x09,0x10;
	
	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send_RCS, 1) == 1)
	{
		printf("Index:%04d  ",count4);count4++;
		printf("CAN1 TX ID:0x%08X",send_RCS[0].ID);
		if(send_RCS[0].ExternFlag==0) printf(" Standard ");
		if(send_RCS[0].ExternFlag==1) printf(" Extend   ");
		if(send_RCS[0].RemoteFlag==0) printf(" Data   ");
		if(send_RCS[0].RemoteFlag==1) printf(" Remote ");
		printf("DLC:0x%02X",send_RCS[0].DataLen);
		printf(" data:0x");

		for(int i=0;i<send_RCS[0].DataLen;i++)
		{
			printf(" %02X",send_RCS[0].Data[i]);
		}

		printf("\n");
	}
	usleep(5000);

	//0x00000202
	VCI_CAN_OBJ send_Size[1];
	send_Size[0].ID=0x00000202;
	send_Size[0].SendType=1;
	send_Size[0].RemoteFlag=0;
	send_Size[0].ExternFlag=0;
	send_Size[0].DataLen=5;
	
	int count5 =0;
	send_Size[0].Data[0] = 0xBE;	
	send_Size[0].Data[1] = 0x00;
	send_Size[0].Data[2] = 0x0C;
	send_Size[0].Data[3] = 0x0F;
	send_Size[0].Data[4] = 0xA0;//filter:0.3sm~100sm 

	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send_Size, 1) == 1)
	{
		printf("Index:%04d  ",count5);count5++;
		printf("CAN1 TX ID:0x%08X",send_Size[0].ID);
		if(send_Size[0].ExternFlag==0) printf(" Standard ");
		if(send_Size[0].ExternFlag==1) printf(" Extend   ");
		if(send_Size[0].RemoteFlag==0) printf(" Data   ");
		if(send_Size[0].RemoteFlag==1) printf(" Remote ");
		printf("DLC:0x%02X",send_Size[0].DataLen);
		printf(" data:0x");

		for(int i=0;i<send_Size[0].DataLen;i++)
		{
			printf(" %02X",send_Size[0].Data[i]);
		}

		printf("\n");
	}

	//0x00000202
	VCI_CAN_OBJ send_ProbExists[1];
	send_ProbExists[0].ID=0x00000202;
	send_ProbExists[0].SendType=1;
	send_ProbExists[0].RemoteFlag=0;
	send_ProbExists[0].ExternFlag=0;
	send_ProbExists[0].DataLen=5;
	
	int count6 =0;
	send_ProbExists[0].Data[0] = 0xC6;	
	send_ProbExists[0].Data[1] = 0x00;
	send_ProbExists[0].Data[2] = 0x01;
	send_ProbExists[0].Data[3] = 0x00;
	send_ProbExists[0].Data[4] = 0x07;

	if(VCI_Transmit(VCI_USBCAN2, 0, 0, send_ProbExists, 1) == 1)
	{
		printf("Index:%04d  ",count6);count6++;
		printf("CAN1 TX ID:0x%08X",send_ProbExists[0].ID);
		if(send_ProbExists[0].ExternFlag==0) printf(" Standard ");
		if(send_ProbExists[0].ExternFlag==1) printf(" Extend   ");
		if(send_ProbExists[0].RemoteFlag==0) printf(" Data   ");
		if(send_ProbExists[0].RemoteFlag==1) printf(" Remote ");
		printf("DLC:0x%02X",send_ProbExists[0].DataLen);
		printf(" data:0x");

		for(int i=0;i<send_ProbExists[0].DataLen;i++)
		{
			printf(" %02X",send_ProbExists[0].Data[i]);
		}

		printf("\n");
	}

	int m_run0=1;
	pthread_t threadid;
	int ret;
	ret=pthread_create(&threadid,NULL,receive_func,&m_run0);
		
	usleep(10000000);
	pthread_join(threadid,NULL);
	usleep(100000);
	VCI_ResetCAN(VCI_USBCAN2, 0, 0);
	usleep(100000);
	VCI_CloseDevice(VCI_USBCAN2,0);
	return 0;
}

