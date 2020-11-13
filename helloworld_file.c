#include <stdio.h>

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>

struct UART_Data_ele{	//新增UART_Data_ele結構
	unsigned char command[2];//新增Char command[2]陣列2byte空間
	unsigned char P1[6];//新增Char P1[6]陣列6byte空間
	unsigned char P2[6];//新增Char P2[6]陣列6byte空間
	//struct UART_Data_ele *next; 
};

FILE *fp = NULL;
int fp_skipall = 0;
long filezie = 0;

char *portname_0 = "/dev/ttyUSB0";
int uart_fd_0 = -1;
int Flag_get_CND_oxA1_reboot_ins=0,Flag_get_CND_oxA2_reboot_ins=0,Flag_get_CND_oxA3_reboot_ins=0,Flag_get_CND_oxA4_reboot_ins=0;
char buffertemp[512]={0};

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error from tcgetattr\r\n");
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error from tcsetattr\r\n");
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error  from tggetattr\r\n");
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error setting term attributes\r\n");
}


/////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
void *uart_0_worker_thread(void *data) //新創通道
{
	int x=0;

	uart_fd_0 = open (portname_0, O_RDWR | O_NOCTTY | O_SYNC);//O_RDWR讀取寫入權限
	if (uart_fd_0 < 0)
	{
			printf ("uart_0_worker_thread !!!! : error opening %s \r\n",portname_0);
			return;
	}
	set_interface_attribs (uart_fd_0, B115200, 0);//UART設定
	set_blocking(uart_fd_0, 0);//UART設定

	while(1)
	{
		usleep(500000);//500ms
		if ((x % 5) ==0)
		{

			printf("GGGGGGGGGG!\r\n");
			
			int free_readdd = 0;
			//unsigned char jjj[1024] = {0};//1024byte 內容全是0
			//unsigned int  uart_r_len = read(uart_fd_0, jjj, 1024);//將讀取到寫入uart_r_len

			struct UART_Data_ele m_kk;

			memset(&m_kk , 0 , sizeof(struct UART_Data_ele));
			unsigned int  uart_r_len = read(uart_fd_0, &m_kk, sizeof(struct UART_Data_ele));
			if(fp_skipall == 0)
			{
				if(uart_r_len > 0)//uart_r_len有寫入
				{
					//printf("m_kk.command[0] = %x m_kk.command[1] =%x \r\n" , m_kk.command[0] , m_kk.command[1]);//顯示出判斷m_kk.command[0]和[1]的內容是否有誤
					//write (uart_fd_0, jjj, uart_r_len);//回傳寫入uart_r_len的值

					if (m_kk.command[0] == 'A' && m_kk.command[1] == '1')//新增判斷m_kk.command[0] 內容為 'A'且m_kk.command[1] 內容為 '1'
					{//新增
						printf("收到主控A1\r\n");//新增指示有收到相同信號
						Flag_get_CND_oxA1_reboot_ins=1;//旗標Flag_get_CND_oxA1_reboot_ins設1指有收到A1
					}//新增
					else if (m_kk.command[0] == 'A' && m_kk.command[1] == '2')//新增判斷m_kk.command[0] 內容為 'A'且m_kk.command[1] 內容為 '2'
					{//新增
						printf("收到主控A2\r\n");//新增指示有收到相同信號
						Flag_get_CND_oxA2_reboot_ins=1;//旗標Flag_get_CND_oxA2_reboot_ins設1指有收到A2
					}//新增
					else if (m_kk.command[0] == 'A' && m_kk.command[1] == '3')//新增判斷m_kk.command[0] 內容為 'A'且m_kk.command[1] 內容為 '3'
					{//新增
						printf("收到主控A3\r\n");//新增指示有收到相同信號

						buffertemp[43]=m_kk.P1[0];

						sprintf(buffertemp,"mkdir /home/yu/example/20201110_practice/A3%s",&m_kk.P1[0]);
						Flag_get_CND_oxA3_reboot_ins=1;//旗標Flag_get_CND_oxA3_reboot_ins設1指有收到A3
					}//新增
					else if (m_kk.command[0] == 'A' && m_kk.command[1] == '4')//新增判斷m_kk.command[0] 內容為 'A'且m_kk.command[1] 內容為 '3'
					{//新增
						printf("收到主控A4\r\n");//新增指示有收到相同信號
						printf("%x %x %x %x %x",m_kk.P1[0],m_kk.P1[1],m_kk.P1[2],m_kk.P1[3],m_kk.P1[4]);
						
						filezie = m_kk.P1[0];
						filezie = filezie<<8;
						filezie = filezie +m_kk.P1[1];
						filezie = filezie<<8;
						filezie = filezie +m_kk.P1[2];
						filezie = filezie<<8;
						filezie = filezie +m_kk.P1[3];
						filezie = filezie<<8;
						filezie = filezie +m_kk.P1[4];

						printf("\r\n length = %ld \r\n",filezie);
						unsigned char *pdata = malloc((sizeof(unsigned char )* filezie)+1024); 
						long lenth_rev = 0;
						while(1)
						{
							uart_r_len = read(uart_fd_0,&pdata[lenth_rev],1024);
							if(uart_r_len >0)
							{
								lenth_rev = lenth_rev + uart_r_len;
								printf("\r\n recv data lenth_rev => %ld \r\n",lenth_rev);
								if(lenth_rev>=filezie)
								{
									break;
								}
							}
						}
						FILE *fp = fopen("/tmp/20201112main","w+");
						
						if(fp == NULL)
						{
							printf("file open fial\r\n");
							return;
						}
						fwrite(pdata,filezie,1,fp);
						fflush(fp);
						fclose(fp);
						system("./TESTscript");
						free(pdata);
						//fp_skipall = 1;
						Flag_get_CND_oxA4_reboot_ins=1;//旗標Flag_get_CND_oxA3_reboot_ins設1指有收到A3
					}//新增
					//printf("send!\r\n");
				}
			}
			else
			{
				unsigned char readbuff[1024] = {0};
				unsigned int uart_r_len = read(uart_fd_0,readbuff,1204);
				if(uart_r_len>0)
				{
					fwrite(readbuff,uart_r_len + 1,1,fp);
					filezie = filezie - uart_r_len;
					if(filezie<=0)
					{
						fp_skipall  = 0;
						// fclose(fp);
					}
				}
			}
		}
		x++;
		if (x>4096)
		{
			x=0;
		}
	}

}


int main(void)
{
	int y = 0;
	pthread_t thread_0;
	pthread_create(&thread_0, NULL, &uart_0_worker_thread, NULL); //開新通道

	while(1)
	{
		usleep(2000000);//2sec

		// if(Flag_get_CND_oxA1_reboot_ins == 1){//判斷旗標Flag_get_CND_oxA1_reboot_ins有收到A1
		// 	printf("\r\n got comand A1 in main loop !!! \r\n");//顯示出收到A1
		// 	system("ls /dev/ttyUSB0>/home/yu/example/20201110_practice/UARTdevice"); //在ubuntu終端輸入的內容偵測到USB0創建一個UART資料夾
		// 	Flag_get_CND_oxA1_reboot_ins == 0;
		// }
		// if(Flag_get_CND_oxA2_reboot_ins == 1){//判斷旗標Flag_get_CND_oxA1_reboot_ins有收到A2
		// 	printf("\r\n got comand A2 in main loop !!! \r\n");//顯示出收到A1
		// 	system("mkdir /home/yu/example/20201110_practice/A2");
		// 	Flag_get_CND_oxA2_reboot_ins == 0;
		// }
		if(Flag_get_CND_oxA3_reboot_ins == 1){//判斷旗標Flag_get_CND_oxA1_reboot_ins有收到A3
			printf("\r\n got comand A3 in main loop !!! \r\n");//顯示出收到A1
			system(buffertemp);
			Flag_get_CND_oxA3_reboot_ins == 0;
		}
			if(Flag_get_CND_oxA4_reboot_ins == 1){//判斷旗標Flag_get_CND_oxA1_reboot_ins有收到A3
		printf("\r\n got comand A4 in main loop !!! \r\n");//顯示出收到A1
		system(buffertemp);
		Flag_get_CND_oxA4_reboot_ins == 0;
		}

		if ((y % 5) ==0)
		{

		//	printf("hd!\r\n");
		
		}
		y++;
		if (y>4096)
		{
			y=0;
		}
	}
	return 0;
}


