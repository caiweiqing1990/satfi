#ifndef __SATFI_H__
#define __SATFI_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>   /* signal() */
#include <fcntl.h> /* file control definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <unistd.h> /* 注意此处：signal.h要先include进来 */
#include <ctype.h>
#include <sys/select.h> /* select() */
#include <netinet/tcp.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/stat.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/prctl.h>
#include <linux/sockios.h>
#include <speex/speex.h>
#include <speex/speex_preprocess.h>
#include <speex/speex_echo.h>
#include <arpa/inet.h>
#include <sys/un.h>
#include <strings.h>
#include <linux/gsmmux.h>
#include <cutils/sockets.h>
#include <cutils/log.h>     /*logging in logcat*/
#include "webrtc/modules/audio_processing/aec/include/echo_cancellation.h"
#include "webrtc/common_audio/vad/include/webrtc_vad.h"

#define GPS_DATA_FILE		"/sdcard/GpsData.txt"
#define CALL_RECORDS_FILE	"/sdcard/CallRecords.txt"
#define CONFIG_FILE			"/sdcard/config.ini"
#define SOS_FILE			"/sdcard/sos.ini"

#define DUDU_WAV			"/vendor/res/sound/dudu.wav"	//摘机嘟嘟声
#define BUSY_WAV			"/vendor/res/sound/busy.wav"	//被叫已挂断提示音
#define TONE_WAV			"/vendor/res/sound/bohao.wav"	//按#号进行拨号
#define SATBUSY_WAV			"/vendor/res/sound/satbusy.wav"	//卫星线路忙

#define SAT_LINK_DISCONNECT	1
#define SAT_LINK_NORMAL		0

#define USERID_LLEN 21 	 			//完整的用户ID长度
#define USERID_LEN  12  			//上传TSC服务器时，只需要传送用户ID的后12位，以节约流量
#define IMSI_LEN    15  			//IMEI和IMSI的长度
#define UDP_VOICE_SRCPORT	12056	//对讲语音源端口
#define UDP_VOICE_DSTPORT	12056	//对讲语音目的端口

#define NN 160
#define TAIL 2000

//3G模块状态
enum N3G_STATE {
  N3G_STATE_IDLE = -1,
  N3G_STATE_AT = 0,
  N3G_STATE_AT_W,
  N3G_STATE_IMEI,
  N3G_STATE_IMEI_W,
  N3G_STATE_IMSI,
  N3G_STATE_IMSI_W,
  N3G_STATE_CSQ,
  N3G_STATE_CSQ_W,
  N3G_STATE_DIALING,
  N3G_SIM_NOT_INSERTED,
};

//卫星模块状态
enum SAT_STATE {
  SAT_STATE_IDLE=-1,
  SAT_STATE_AT=0,
  SAT_STATE_AT_W,  //01
  SAT_STATE_IMEI,
  SAT_STATE_IMEI_W,//03
  SAT_STATE_IMSI,
  SAT_STATE_IMSI_W, //5
  SAT_STATE_GPSTRACK_START,
  SAT_STATE_GPSTRACK_START_W,//7
  SAT_STATE_GPSTRACK_STOP,
  SAT_STATE_GPSTRACK_STOP_W, //9
  SAT_STATE_CSQ,
  SAT_STATE_CSQ_W, //11
  SAT_STATE_DIALING,
  SAT_STATE_CFUN,
  SAT_STATE_CFUN_W,//14
  SAT_STATE_SIM_ACTIVE,//15
  SAT_STATE_SIM_ACTIVE_W,
  SAT_STATE_FULL_FUN,
  SAT_STATE_FULL_FUN_W, //18
  SAT_STATE_CREG,
  SAT_STATE_CREG_W,
  SAT_STATE_RESTART, //21
  SAT_STATE_RESTART_W,
  SAT_SIM_NOT_INSERTED,
  SAT_SIM_ARREARAGE,//欠费
  SAT_STATE_REGFAIL,
  SAT_STATE_CGACT,
  SAT_STATE_CGACT_W,
  SAT_STATE_CGACT_SCCUSS,
  SAT_STATE_CGEQREQ,
  SAT_STATE_CGEQREQ_OK,
};

enum SAT_STATE_PHONE {
  SAT_STATE_PHONE_IDLE = 0,		//空闲
  SAT_STATE_PHONE_CLIP,			//设置来电显示
  SAT_STATE_PHONE_CLIP_OK,		//设置来电显示成功	2
  SAT_STATE_PHONE_CLCC,   		//查询是否可进行拨号
  SAT_STATE_PHONE_CLCC_OK,		//可进行拨号
  SAT_STATE_PHONE_ATD_W,		//拨号命令已发送	5
  SAT_STATE_PHONE_ATA_W,		//应答命令已发送
  SAT_STATE_PHONE_ATH_W,		//挂断命令已发送	7
  SAT_STATE_PHONE_RING_COMING,	//电话来电
  SAT_STATE_PHONE_DIALING,		//电话拨号中 9
  SAT_STATE_PHONE_DIALING_RING,	//电话振铃中 10
  SAT_STATE_PHONE_NOANSWER,		//电话无人接听
  SAT_STATE_PHONE_ONLINE,		//电话通话中 12
  SAT_STATE_PHONE_HANGUP,		//电话已挂断
  SAT_STATE_PHONE_COMING_HANGUP,//来电电话已挂断	14
  SAT_STATE_PHONE_DIALINGFAILE,	//拨号失败
  SAT_STATE_PHONE_DIALING_SUCCESS,//接听成功	16
  SAT_STATE_PHONE_DIALING_ERROR,//拨号错误
  SAT_STATE_PHONE_DIALING_CLCC,//查询拨号情况 18
  SAT_STATE_PHONE_DIALING_ATH_W,//拨号失败挂断命令已发送
  SAT_STATE_PHONE_DIALING_FAILE_AND_ERROR,//20
};

typedef  struct _n3g
{
  int n3g_fd;                //3G模块串口文件
  int n3g_available;
  int n3g_status;            //0：command 1：online data
  enum N3G_STATE n3g_state;
  int n3g_hb_seconds;
  int n3g_baud_rate;         //波特率
  int n3g_csq_value;         //信号强度
  int n3g_csq_limit_value;   //信号阈值
  int n3g_csq_ltime;         //得到信号强度的时间
  int n3g_dialing;           //是否正在尝试拨号
  char n3g_ifname[16];
  char n3g_ifname_a[16];
  char n3g_imsi[16];
  char n3g_imei[16];
  char n3g_dev_name[32];     //3G模块串口设备
}N3G;

typedef struct _sat
{
  int sat_fd;                //SAT模块文件
  int ring;
  int qos1;					//u/a
  int qos2;					//384/64
  int qos3;					//384/64
  int active;				//是否准许激活
  int gprs_on;
  int sat_phone;
  int sat_message;
  int sat_pcmdata;
  int sat_available;		//-1|0|1|2|3 模块加载失败|未激活|已激活|正在激活|未准许激活
  int sat_status;            //0：command 1：online data
  enum SAT_STATE sat_state;
  enum SAT_STATE_PHONE sat_state_phone;
  int sat_hb_seconds;
  int sat_baud_rate;         //波特率
  int sat_csq_limit_value;   //信号阈值
  int sat_csq_value;         //信号强度
  int sat_csq_ltime;         //得到信号强的时间
  int sat_calling;           //是否正在进行呼叫
  int sat_dialing;           //是否正在尝试拨号
  int sat_msg_sending;
  int battery;
  char sat_ifname[16];
  char sat_ifname_a[16];
  char sat_imsi[16];
  char sat_imei[16];
  char sat_dev_name[32];     //SAT模块串口设备
  char sat_gps[256];         //SAT模块GPS数据
  int socket;				//主叫用户socket
  char calling_number[32];	//主叫号码
  char called_number[32];	//被叫号码
  int captain_socket;		//船长socket
  time_t start_time;		//开始通话时的时间
  time_t end_time;			//通话挂断时的时间

  char MsID[USERID_LLEN];				//终端ID
  char MsPhoneNum[11];			//船长的电话
  char DesPhoneNum[11];			//对方的电话
  unsigned long long StartTime;	//通话开始时间
  unsigned long long EndTime;	//通话结束时间
  unsigned short CallTime;		//通话时长
  unsigned int Money;			//通话费用/分
  int charge;
  
  int voice_socket_udp;
  struct sockaddr_in clientAddr1;
  struct sockaddr_in clientAddr2;

  int secondLinePhoneMode;
  int isSecondLinePickUp;
  int isSecondLineFirstKeyPress;
  int playBusyToneFlag;

  int net_status;//0-无天通卫星卡 1-正在入网 2-已入网
  int data_status;//0-天通数据未激活 1-正在激活 2-已激活
  int module_status; //0-卫星模块加载失败 1-正在开机 2-正在复位 3-工作正常

  int locak_socket_audio_cancel;
  int lte_status;//0-无4G卡 1-4G已开启 2-4G未开启
  int sim_status;
  int Upgrade1Confirm; //1：不下载 2：下载 3：不升级 4：升级
  int Upgrade2Confirm; //1：不下载 2：下载 3：不升级 4：升级

  float VolumeTrack;
  float VolumeRecord;

  int satbusy;
}SAT;

typedef struct _gps
{
  int gps_fd;               //北斗GPS模块文件
  int gps_baud_rate;        //波特率
  char gps_dev_name[32];    //北斗GPS模块串口设备
  char gps_bd[256];         //北斗GPS数据
  int Lg;					//经度
  char Lg_D;				//东西经
  int Lt;					//纬度
  char Lt_D;				//南北纬
  unsigned long long Date;	//时间戳
  int Speed;				//速度
}GPS;

typedef struct _tsc
{
  int tsc_port;              //TSC服务器端口
  int tsc_timeout;
  int keepalive_interval;
  int tsc_hb_req_ltime;
  int tsc_hb_rsp_ltime;
  char tsc_domain[256];      //TSC服务器：域名
  char tsc_addr[32];         //TSC服务器：IP
  int update_interval;		 //升级间隔
  int SizeTmp;
  int SendBufSize;
  int mss;
  int route[16];
}TSC;

typedef struct _app
{
  int app_port;              //APP监听端口
  int app_timeout;
  char app_addr[32];         //APP服务器：IP
}APP;

typedef struct _omc
{
  int omc_port;
  int omc_timeout;
  char omc_domain[256];
  char omc_addr[32];
}OMC;

typedef struct _dial
{
	int dialing;				//拨打电话中
	char calling_number[15];	//主叫号码
	char called_number[15];		//被叫号码
	int socket;					//主叫用户IP和PORT
}DIAL;

typedef struct _base
{
  N3G n3g;
  SAT sat;
  GPS gps;
  TSC tsc;
  APP app;
  OMC omc;
}BASE;

typedef struct _grpinfo
{
	char GrpID[USERID_LLEN];		//群组ID
	unsigned short Count;			//终端个数
	char msid[500][USERID_LLEN];	//一个群组最多500个终端,可修改
}GRPINFO;

typedef struct _user {
  int socketfd;						//套接字
  char userid[USERID_LLEN];        	//用户ID
  char defgrp[USERID_LLEN];			//默认群组ID
  unsigned short grpCount; 			//群组个数
  GRPINFO grpinfo[30];				//群组个数最多保存30个，可修改
  int udp_voice_packnum;			//用于语音对讲计费，统计收到的语音包数
  int famNumConut;
  char FamiliarityNumber[100][USERID_LLEN];//亲情号码最多100个
  struct _user *next;
}USER;

typedef struct _log {
  char MsID[USERID_LLEN]; 
  void *data;
  struct _log *next;
}LOG;

typedef  struct Pack{
	unsigned int Name;			//消息 语音 图片 唯一识别码
	unsigned short PackSeq; 	//包流水号
	unsigned short Packtotal; 	//语音 图片 总包数
	int offset;					//数据偏移
	char* Data;					//消息 语音 图片数据
	int isSended;
	unsigned short type;		// 0 1 2 消息 语音 图片
	struct Pack *next;
}PACK;

typedef  struct Messge{
	char MsID[USERID_LLEN]; 
	char toPhoneNum[USERID_LLEN];
	int ID;
	char Data[1024];
	struct Messge *next;
}MESSAGE;

typedef  struct GPSData{
	char* Data;
	struct GPSData *next;
}GPSDATA;

#define BUF_SIZE	1024*32
typedef struct _appsocket {
  int AppSocketFd;
  time_t Update;
  int DataSaveOffset;
  char Data[BUF_SIZE];
  char ip[16];
  struct _appsocket *next;
}APPSOCKET;

#undef LOG_TAG
#define LOG_TAG "satfi"
 
#define LOGD(fmt, arg ...) ALOGD("%s: " fmt, __FUNCTION__ , ##arg)
#define LOGW(fmt, arg ...) ALOGW("%s: " fmt, __FUNCTION__ , ##arg)
#define LOGE(fmt, arg ...) ALOGE("%s: " fmt, __FUNCTION__ , ##arg)

#define __DEBUG__  
#ifdef __DEBUG__  
//#define satfi_log(x...) printf(x)
#define satfi_log(x...) ALOGE(x)
#else
#define satfi_log(x...)
#endif

extern void base_init(void);
extern int AppCallUpRspForce(int socket, short sat_state_phone);
extern int handle_sat_data(int *satfd, char *data, int *ofs);
extern int safe_recvfrom(int fd, char* buff, int len);
extern int parseGpsData(char *buf, int len);
extern void gps_start(void);
extern int gps_stop(void);
extern int create_satfi_udp_fd(void);
extern void milliseconds_sleep(unsigned long milliseconds);
extern int safe_sendto(const char* path, const char* buff, int len);

#endif
