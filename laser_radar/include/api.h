/**********************************************************************
* COPYRIGHT NOTICE - HYPERSEN TECHNOLOGY
*
* Copyright (c) 2018, Hypersen Technology, Inc.
*
* All rights reserved.
*
*======================================================================
* \file api.h
* \brief TODO
* \author Kevin
* \email Kevin_Wang@hypersen.com
* \version 1.0.0
* \date 2018年11月13日 上午11:43:31
* \license private & classified
*---------------------------------------------------------------------
* Remark: This project is still under construction.
*======================================================================
* Change History:
*---------------------------------------------------------------------
* <Date>			| <Version>	| <Author>			| <Description>
*---------------------------------------------------------------------
* 2018年11月13日			| V1.0.0	| Kevin				| Create file
*======================================================================
* Detailed Notes:
*---------------------------------------------------------------------
* <Version>		| <Description>
*---------------------------------------------------------------------
* V1.0.0		| TODO
*---------------------------------------------------------------------

**********************************************************************/

#ifndef HPS3D_API_H_
#define HPS3D_API_H_

#ifdef __cplusplus
extern "C"  /*C++*/
{
#endif

#include <stdbool.h>
#include <string.h>


#ifdef _WIN32
	#include <windows.h>
	#include <winsock2.h>
#else
	#include <sys/socket.h>
	#include <netinet/in.h>
#endif

#define DLL_API _declspec(dllexport)

typedef signed char 	int8_t;
typedef unsigned char 	uint8_t;
typedef unsigned short 	uint16_t;
typedef short 			int16_t;
typedef unsigned int 	uint32_t;
typedef int 			int32_t;
typedef float 			float32_t;
typedef double 			float64_t;

/***********************************************************************************/
/*用户可修改，参数直接影响到SDK内存占用，详细配置参考SDK手册(HPS3D_RM002) 四、多设备支持  */
#define 	DEV_NUM 			 (10)							/*支持的设备数量*/
#define     DEV_NAME_SIZE		 (20)							/*设备名长度*/
#define 	ROI_NUM 			 (8)							/*ROI的数量*/
#define 	OBSTACLE_NUM 		 (20)							/*支持障碍物数量*/
#define 	OBSERVER_NUM  		 (10)							/*观察者数量*/
/**********************************************************************************/
/*内部使用参数请勿修改*/
/*	相机分辨率 */
#define		RES_WIDTH			 (160)
#define		RES_HEIGHT			 (60)
#define		MAX_PIX_NUM 		 (9600)   /*RES_WIDTH * RES_HEIGHT*/

/*指示当前像素点测量无效值及含义*/
#define	 	LOW_AMPLITUDE   	(65300) 						/*信号幅值低*/
#define	  	SATURATION 			(65400)     					/*饱和位饱和*/
#define	 	ADC_OVERFLOW  		(65500)   						/*ADC溢出*/
#define	 	INVALID_DATA 		(65530)    						/*无效数据*/
/**********************************************************************************/

/*函数返回的结果*/
typedef enum
{
	RET_OK 		= 0x01,
	RET_ERROR 	= 0x02,
	RET_BUSY 	= 0x03,
	RET_CONNECT_FAILED,
	RET_CREAT_PTHREAD_ERR,  /*线程创建失败*/
	RET_WRITE_ERR,
	RET_READ_ERR,
	RET_PACKET_HEAD_ERR,
	RET_PACKET_ERR,
	RET_BUFF_EMPTY,			/*缓冲区为空或不可用*/
	RET_VER_MISMATCH,  		/*相机固件版本与SDK版本不匹配*/
}RET_StatusTypeDef;

/*设备版本*/
typedef struct
{
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t major;
	uint8_t minor;
	uint8_t rev;
}Version_t;

/*运行模式*/
typedef enum
{
	MinOfRunModeType = 0,
	RUN_IDLE = 0,			/*待机或停止测量*/
	RUN_SINGLE_SHOT,		/*单次测量*/
	RUN_CONTINUOUS,			/*连续测量*/
	NumberOfRunModeType
}RunModeTypeDef;

/*设置测量数据包类型*/
typedef enum
{
	DEPTH_DATA_PACKET = 0x0,  /*深度数据包  @see DepthDataTypeDef*/
	ROI_DATA_PACKET,		  /*ROI数据包  @see FullRoiDataTypeDef  @see SimpleRoiDataTypeDef*/
	OBSTACLE_PACKE			  /*障碍物数据包  @see ObstacleDataTypedef*/
}MeasurePacketTypeDef;


/*输出数据包的类型*/
typedef enum
{
	PACKET_FULL = 0,		/*完整数据包（包含深度数据）*/
	PACKET_SIMPLE			/*简单数据包（不包含深度数据）*/
}OutPacketTypeDef;

/*ROI的阈值警报类型*/
typedef enum
{
	ROI_ALARM_DISABLE = 0,  /*关闭ROI GPIO警报*/
	ROI_ALARM_GPIO       	/*ROI警报类型为GPIO信号输出*/
}ROIAlarmTypeDef;

/*迟滞配置*/
typedef struct
{
	uint8_t threshold_id;  		    /*敏感区域阈值id*/
	uint32_t threshold_value; 		/*敏感区域阈值*/
	uint32_t hysteresis; 			/*迟滞大小*/
	bool enable;					/*迟滞使能*/
	bool positive;					/*true:正向比较，如果输入值大于阈值则返回True
									false:反向比较，如果输入值小于阈值则返回False*/
}HysteresisSingleConfTypeDef;

/*ROI的参考值类型*/
typedef enum
{
	ROI_REF_DIST_AVR = 1,
	ROI_REF_DIST_MIN,
	ROI_REF_DIST_MAX,
	ROI_REF_SAT_COUNT,
	ROI_REF_AMPLITUDE,
	ROI_REF_VAILD_AMPLITUDE,
	ROI_REF_THRESHOLD_PIX_NUM
}ROIReferenceTypeDef;

/*ROI配置的结构体*/
 typedef struct
{
	bool enable;
	uint8_t roi_id;
	uint16_t left_top_x;
	uint16_t left_top_y;
	uint16_t right_bottom_x;
	uint16_t right_bottom_y;
	HysteresisSingleConfTypeDef hysteresis_conf[3];
	ROIReferenceTypeDef ref_type[3];					/*ROI的参考值类型,与hysteresis_conf一一对应*/
	ROIAlarmTypeDef alarm_type[3];						/*ROI的阈值警报类型,与hysteresis_conf一一对应*/
	uint16_t pixel_number_threshold[3];					/*超过阈值的像素点数阈值,与hysteresis_conf一一对应*/
}ROIConfTypeDef;

/*HDR模式*/
typedef enum
{
	HDR_DISABLE = 0,				/*不建议使用*/
	AUTO_HDR,						/*不建议使用*/
	SUPER_HDR,
	SIMPLE_HDR
}HDRModeTypeDef;

/*HDR配置*/
typedef struct
{
	HDRModeTypeDef hdr_mode;
	float32_t qualtity_overexposed;				/*AUTO_HDR 曝光幅值  不建议使用该模式*/
	float32_t qualtity_overexposed_serious;		/*AUTO_HDR 过度曝光赋值 不建议使用该模式*/
	float32_t qualtity_weak;					/*AUTO_HDR 信号弱幅值 不建议使用该模式*/
	float32_t qualtity_weak_serious;			/*AUTO_HDR 信号极弱幅值 不建议使用该模式*/
	uint32_t simple_hdr_max_integration;		/*SIMPLE_HDR 最大积分时间 us*/
	uint32_t simple_hdr_min_integration;		/*SIMPLE_HDR 最小积分时间 us*/
	uint8_t super_hdr_frame_number;				/*SUPER_HDR 合成帧数*/
	uint32_t super_hdr_max_integration;			/*SUPER_HDR 最大积分时间 us*/
	uint32_t hdr_disable_integration_time;		/*HDR_DISABLE 手动积分时间 us 不建议使用该模式*/
}HDRConf;

/*平滑滤波器的类型*/
typedef enum
{
	SMOOTH_FILTER_DISABLE = 0,		/*关闭平滑滤波器*/
	SMOOTH_FILTER_AVERAGE = 1,		/*均值滤波器*/
	SMOOTH_FILTER_GAUSS				/*高斯滤波器*/
}SmoothFilterTypeDef;

/*平滑滤波器的配置*/
typedef struct
{
	SmoothFilterTypeDef type;		/*平滑滤波器类型*/
	uint32_t arg1;					/*滤波参数*/
}SmoothFilterConfTypeDef;


/*uart相关配置*/
typedef enum
{
	UART_BAUDRATE_9600 = 9600,
	UART_BAUDRATE_19200 = 19200,
	UART_BAUDRATE_38400 = 38400,
	UART_BAUDRATE_57600 = 57600,
	UART_BAUDRATE_115200 = 115200,
	UART_BAUDRATE_230400 = 230400,
	UART_BAUDRATE_460800 = 460800,
	UART_BAUDRATE_1000000 = 1000000,
	UART_BAUDRATE_1500000 = 1500000,
	UART_BAUDRATE_2000000 = 2000000,
}UartBaudrateTypeDef;

typedef enum
{
	PARITY_NONE1,
	PARITY_ODD1,
	PARITY_EVEN1,
}UartParityTypeDef;

typedef enum
{
	UART_STOP_BITS0_5,
	UART_STOP_BITS1,
	UART_STOP_BITS1_5,
	UART_STOP_BITS2,
}UartStopBitTypeDef;

/*串口参数配置*/
typedef struct
{
	UartBaudrateTypeDef baudrate;
	UartParityTypeDef parity;
	UartStopBitTypeDef stop_bits;
}UartConfTypeDef;


/*GPIO配置的相关定义*/
/*GPIO_OUT功能*/
typedef enum
{
	GPOUT_FUNC_DISABLE = 0,				/*GPIO报警关闭*/
	GPOUT_FUNC_ROI_THRESHOLD0_ALARM,	/*GPIO输出阈值0报警*/
	GPOUT_FUNC_ROI_THRESHOLD1_ALARM,	/*GPIO输出阈值1报警*/
	GPOUT_FUNC_ROI_THRESHOLD2_ALARM		/*GPIO输出阈值2报警*/
}GPOutFunctionTypeDef;

/*GPIO_IN功能*/
typedef enum
{
	GPIN_FUNC_DISABLE = 0,			/*GPIO功能关闭*/
	GPIN_FUNC_CAPTURE   			/*开启测量  注：开启测量后，则不受命令控制，只受IO输入控制*/
}GPInFunctionTypeDef;

/*GPIO极性*/
typedef enum
{
	GPIO_POLARITY_LOW = 0,
	GPIO_POLARITY_HIGH
}GPIOPolarityTypeDef;

/*GPIO引脚*/
typedef enum
{
    GPIN_1 = 1,
    GPIN_2 = 2,
    GPIN_3 = 3,
    GPOUT_1 = 10,
    GPOUT_2 = 11,
    GPOUT_3 = 12,
}GPIOTypeDef;

/*GPIO输出配置*/
typedef struct
{
	GPIOTypeDef gpio;
	GPIOPolarityTypeDef polarity;
	GPOutFunctionTypeDef function;
}GPIOOutConfTypeDef;

/*GPIO输入配置*/
typedef struct
{
	GPIOTypeDef gpio;
	GPIOPolarityTypeDef polarity;
	GPInFunctionTypeDef function;
}GPIOInConfTypeDef;


/*深度滤波器类型*/
typedef enum
{
	DISTANCE_FILTER_DISABLE = 0,
	DISTANCE_FILTER_SIMPLE_KALMAN			/*简单卡尔曼滤波*/
}DistanceFilterTypeDef;

typedef struct
{
	DistanceFilterTypeDef filter_type;		/*@see DistanceFilterTypeDef*/
	float32_t kalman_K; 					/*比例系数K 建议值0.5*/
	uint32_t kalman_threshold;				/*噪声阈值*/
	uint32_t num_check;						/*阈值检查帧数,建议值2*/
}DistanceFilterConfTypeDef;

/*安装角度变换参数,旋转坐标系使用*/
typedef struct
{
	bool enable;					/*安装角度使能*/
	uint8_t angle_vertical;     	/*垂直方向安装角度（°）:主光轴与地垂线间的夹角*/
	uint16_t height;				/*相对于地面的安装高度(mm)*/
}MountingAngleParamTypeDef;

/*返回的数据包类型*/
typedef enum
{
	NULL_PACKET = 0x00,				/*数据返回为空即无测量数据返回*/
	SIMPLE_ROI_PACKET = 0x01,		/*简单ROI数据包（不含深度图数据）@see SimpleRoiDataTypeDef*/
	FULL_ROI_PACKET,				/*完整ROI数据包（含深度图数据）@see FullRoiDataTypeDef*/
	FULL_DEPTH_PACKET,				/*完整深度数据包（含深度图数据）@see DepthDataTypeDef*/
	SIMPLE_DEPTH_PACKET,			/*简单深度数据包（不含深度图数据）@see DepthDataTypeDef*/
	KEEP_ALIVE_PACKET,				/*心跳返回包*/
	OBSTACLE_PACKET,				/*障碍物数据包  @see ObstacleDataTypedef*/
    SYSTEM_ERROR					/*系统错误*/
}RetPacketTypedef;

/*ROI简单数据包*/
typedef struct
{
	uint8_t group_id;						/*组ID*/
	uint8_t id;								/*ROI ID*/
	uint16_t amplitude;						/*平均幅值*/
	uint16_t valid_amplitude;				/*平均有效幅值*/
	uint16_t distance_average;				/*平均距离值*/
	uint16_t distance_max;					/*最大距离值*/
	uint16_t distance_min;					/*最小距离值*/
	uint16_t dist_max_x;					/*不可用*/
	uint16_t dist_max_y;					/*不可用*/
	uint16_t dist_min_x;					/*不可用*/
	uint16_t dist_min_y;					/*不可用*/
	uint16_t saturation_count;				/*饱和像素点数*/
	uint8_t threshold_state;				/*报警状态指示位,当前测量值是否超出阈值:bit0:zone0, bit1:zone1, bit2:zone2*/
	uint16_t out_of_threshold_pix_num[3];	/*[0]:超过thresold0的像素点数,[1]:...,[2]:...*/
	uint16_t frame_cnt;						/*帧计数器*/
}SimpleRoiDataTypeDef;

/*完整ROI数据包*/
typedef struct
{
	uint8_t roi_num;						/*ROI总数量*/
	uint8_t group_id;						/*组ID*/
	uint8_t id;								/*ROI ID*/
	uint16_t left_top_x;					/*左上角x坐标*/
	uint16_t left_top_y;					/*左上角y坐标*/
	uint16_t right_bottom_x;				/*右下角x坐标*/
	uint16_t right_bottom_y;				/*右下角y坐标*/
	uint32_t pixel_number;					/*ROI像素点*/
	uint16_t amplitude;						/*平均幅值*/
	uint16_t valid_amplitude;				/*平均有效幅值*/
	uint16_t distance_average;				/*平均距离值*/
	uint16_t distance_max;					/*不可用*/
	uint16_t distance_min;					/*最小距离值*/
	uint16_t saturation_count;				/*饱和像素点数*/
	uint16_t threshold_state;				/*当前测量值是否超出阈值:bit0:zone0, bit1:zone1, bit2:zone2*/
	uint16_t dist_max_x;					/*不可用*/
	uint16_t dist_max_y;					/*不可用*/
	uint16_t dist_min_x;					/*不可用*/
	uint16_t dist_min_y;					/*不可用*/
	uint32_t frame_cnt;						/*帧计数器*/
	uint16_t *distance;						/*深度数据，按顺序储存*/
}FullRoiDataTypeDef;

/*深度图数据*/
typedef struct
{
	uint16_t distance_average;				/*整个视野平均距离值*/
	uint16_t amplitude_average;				/*有效幅值,不包含无效像素点*/
	uint16_t amplitude_average_whole;		/*整个视野平均幅值*/
	uint16_t amplitude_low_count;			/*低信号像素的数量*/
	uint16_t saturation_count;				/*饱和像素点数*/
	uint16_t distance_max;					/*不可用*/
	uint16_t distance_min;					/*最小距离估算值*/
	int16_t temperature;					/*相机当前温度参考值*/
	uint16_t frame_cnt;						/*帧计数器，可用于丢帧检测*/
	uint16_t interference_num; 				/*不可用*/
	uint16_t *distance;						/*深度数据，按顺序储存，当输出数据类型为 @see PACKET_SIMPLE时不可用*/
}DepthDataTypeDef;


typedef struct
{
	float32_t x;
	float32_t y;
	float32_t z;
}PerPointCloudDataTypeDef;

/*有序点云数据*/
typedef struct
{
	PerPointCloudDataTypeDef *point_data;
	uint16_t width;
	uint16_t height;
	uint32_t points;
}PointCloudDataTypeDef;

/*障碍物配置相关参数*/
typedef struct
{
	bool enable;
	uint16_t frame_head; 			/*数据帧，帧头特征字节，例如0XEB81*/
	uint8_t number;  				/*需要提取的障碍物数量 例如3*/
	uint16_t vaild_min_dist; 		/*有效范围的最小距离值mm 例如 600*/
	uint16_t vaild_max_dist; 		/*有效范围的最大距离值mm 例如 3500*/
	uint16_t invaild_value;  		/*无效区域的固定参数值mm 例如 5000*/
	uint16_t frame_size;			/*保存当前缓冲区有效字节数*/
}ObstacleConfigTypedef;

/*障碍物数据*/
typedef struct
{
	uint8_t ObstacleNumber;								/*障碍物的总数*/
	uint8_t Id;											/*障碍物ID*/
	uint32_t FrameCount;  								/*帧计数值*/
	uint16_t PixelNumber;
	uint16_t DistanceAverage;
	PerPointCloudDataTypeDef LeftPoint;
	PerPointCloudDataTypeDef RightPoint;
	PerPointCloudDataTypeDef UpperPoint;
	PerPointCloudDataTypeDef UnderPoint;
	PerPointCloudDataTypeDef MinPoint;
	PerPointCloudDataTypeDef *PixelBuffer;   /*保存障碍物所有像素点信息*/
}ObstacleDataTypedef;

/*避障参数结构体*/
typedef struct
{
	bool avoidance_enable;
	uint32_t avoidance_width;
	uint32_t avoidance_height;
	uint32_t avoidance_minDistance;
	uint32_t avoidance_mountHeight; /*安装高度*/
	uint32_t avoidance_pixelNumber; /*像素点阈值数量*/
	bool avoidance_alarmStatues;    /*报警状态指示*/
	GPIOOutConfTypeDef GpioOutConf; /*GPIO功能*/
}AvoidanceTypeDef;

/*结构体体封装结构体 用于数据的返回*/
typedef struct
{
	SimpleRoiDataTypeDef *simple_roi_data;		/*简单ROI数据包 @see SimpleRoiDataTypeDef*/
	FullRoiDataTypeDef *full_roi_data;			/*完整ROI数据包 @see FullRoiDataTypeDef*/
	DepthDataTypeDef *simple_depth_data;		/*简单深度图数据包  @see DepthDataTypeDef*/
	DepthDataTypeDef *full_depth_data;			/*完整ROI数据包 @see DepthDataTypeDef*/
	PointCloudDataTypeDef *point_cloud_data;	/*点云数据包 @see PointCloudDataTypeDef*/
	ObstacleDataTypedef *Obstacle_data;			/*障碍物数据包 @see ObstacleDataTypedef*/
	uint8_t *Obstacle_frame_data_buff; 			/*用于存放障碍物数据包的缓冲区*/
}MeasureDataTypeDef;


typedef enum
{
	SYNC = 0x01,  						/*同步方式*/
	ASYNC = 0x02 						/*异步方式*/
}HPS3D_SynchronousTypedef;

/*传输类型*/
typedef enum
{
	TRANSPORT_USB = 0,
	TRANSPORT_CAN,
	TRANSPORT_RS232,
	TRANSPORT_RS485,
	TRANSPORT_ETHERNET
}TransportTypeDef;

/*handle*/
typedef struct
{
	char *DeviceName; 					/*R/W 设备名*/
	uint32_t DeviceFd; 					/*R   不可修改*/
	uint8_t DeviceAddr; 				/*R   存放当前连接设备的设备地址(也是帧ID)*/
	uint8_t ConnectionNumber;			/*R   不可修改*/
	HPS3D_SynchronousTypedef SyncMode;  /*R   同步或异步方式*/
	RunModeTypeDef RunMode;   			/*R/W 运行模式*/
	MeasureDataTypeDef MeasureData;     /*R   同步测量数据,当异步方式时测量结果不会保存在此(可通过观察者对其操作)*/
	RetPacketTypedef RetPacketType;     /*R   同步测量返回包类型,当异步方式时测量返回包类型结果不会保存在此(可通过观察者对其操作)*/
	OutPacketTypeDef OutputPacketType; 	/*R   输出数据包类型,不可修改*/
	bool ConnectStatus;  	 			/*R   连接状态,不可修改*/
	uint8_t RoiNumber;					/*R   保存当前设备支持的ROI数量*/
	uint8_t ThresholdNumber;			/*R   保存当前设备ROI支持的阈值数量*/
	uint8_t ViewAngleHorizontal;  		/*R   水平方向视场角,不可修改*/
	uint8_t ViewAngleVertical;			/*R   垂直方向视场角,不可修改*/
	struct sockaddr_in ServerAddr;	    /*R/W 服务器IP地址及端口号*/
	TransportTypeDef TransportType;     /*R   当前传输类型*/
}HPS3D_HandleTypeDef;

/*光学参数*/
typedef struct
{
	bool enable;						/*光学参数使能（开启后，测量的深度数据为垂直距离）*/
	uint8_t viewing_angle_horiz;    	/*水平方向可视角*/
	uint8_t viewing_angle_vertical; 	/*垂直方向可视角*/
	uint8_t illum_angle_horiz;
	uint8_t illum_angle_vertical;
}OpticalParamConfTypeDef;

/*多机干扰的配置，不可用*/
typedef struct
{
	bool enable;
	uint32_t integ_time;
	uint16_t amplitude_threshold;
	uint8_t capture_num;
	uint8_t number_check;
}InterferenceDetectConfTypeDef;


/*观察者模式详细请参考 https://www.cnblogs.com/luohanguo/p/7825656.html
 * */
/*观察者订阅事件*/
typedef enum
{
	ISubject_Event_DataRecvd = 1<<0,	/*数据接收事件*/
	ISubject_Event_DevConnect = 1<<1,	/*连接事件*/
	ISubject_Event_DevDisconnect = 1<<2 /*断开连接事件*/
}AsyncISubjectEvent;

/*观察者订阅事件结构体参数*/
typedef struct
{
	uint8_t ObserverID;
	bool NotifyEnable;
	AsyncISubjectEvent AsyncEvent; 		/*观察者订阅事件*/
	MeasureDataTypeDef MeasureData; 	/*用于存放测量结果*/
	RetPacketTypedef RetPacketType; 	/*测量返回包类型*/
}AsyncIObserver_t;

/*点云数据镜像*/
typedef enum
{
	MORROR_DISABLE = 0X0,
	MIRROR_HORIZONTAL = 0X1,
	MIRROR_VERTICLA = 0X2,
}PointCloudImageMirrorTypeDef;

/**************************************函数接口*************************************/

/***********************************1.命令函数接口***********************************/
/**
 * @brief	设置测量模式
 * @param[in]
 * @note    调用次函数前需要手动修改handle->RunMode值，
 * 			并且该函数需在设备初始化完成后才可使用，@see HPS3D_ConfigInit 后使用
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetRunMode(HPS3D_HandleTypeDef *handle);

/**
 * @brief	得到设备地址
 * @param[out]  handle->DeviceAddr  输出设备地址
 * @note    此设备地址也是帧ID
 * @retval	成功返回RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetDevAddr(HPS3D_HandleTypeDef *handle);

/**
 * @brief	设置设备地址
 * @param[in]	handle->DeviceAddr   原设备地址
 * @param[in]	new_addr     		  新设备地址
 * @note	多台设备可通过修改此参数进行区分
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetDevAddr(HPS3D_HandleTypeDef *handle, uint8_t new_addr);

/**
 * @brief	获取设备版本信息
 * @param[out]  version_t
 * @note	得到传感器固件版本信息参数
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetDeviceVersion(HPS3D_HandleTypeDef *handle, Version_t *version_t);

/**
 * @brief	设定测量数据包类型
 * @param[in]	type
 * @note	需在初始化@see HPS3D_ConfigInit 前进行该配置,默认为完整深度数据包
 * @see
 * @code
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetMeasurePacketType(MeasurePacketTypeDef type);
/**
 * @brief	获取测量数据包类型
 * @param
 * @note	默认为完整深度数据包
 * @see		MeasurePacketTypeDef
 * @code
 * @retval	返回类型枚举值
 */
extern MeasurePacketTypeDef HPS3D_GetMeasurePacketType(void);


/**
 * @brief	设定测量数据返回包类型(简单包或完整包)
 * @param[in] outPacketType
 * @note    设定输出的数据包为简单数据包或完整数据包
 * @see		OutPacketTypeDef
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetPacketType(HPS3D_HandleTypeDef *handle,OutPacketTypeDef outPacketType);

/**
 * @brief	获取数据包类型
 * @param[out]	handle->PacketType
 * @note
 * @see		OutPacketTypeDef
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetPacketType(HPS3D_HandleTypeDef *handle);


/**
 * @brief	保存通信配置
 * @note	使用该函数后通信参数将永久生效，请牢记当前配置(慎重使用)
 * @see
 * @code
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SaveTransportConf(HPS3D_HandleTypeDef *handle);

/**
 * @brief	保存到用户配置参数
 * @param
 * @note	该函数将使当前配置永久生效，可通过清除配置或回复出厂恢复默认配置
 * @see		HPS3D_ProfileClearCurrent HPS3D_ProfileRestoreFactory
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_ProfileSaveToCurrent(HPS3D_HandleTypeDef *handle);

/**
 * @brief	清除用户配置参数
 * @param
 * @note
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_ProfileClearCurrent(HPS3D_HandleTypeDef *handle);

/**
 * @brief	恢复出厂设置
 * @param
 * @note
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_ProfileRestoreFactory(HPS3D_HandleTypeDef *handle);

/**
 * @brief	获得传输类型
 * @param[out]	transport_type
 * @note
 * @see	TransportTypeDef
 * @retval	成功返回 RET_OK
 * @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_GetTransportType(HPS3D_HandleTypeDef *handle, TransportTypeDef *transport_type);

/**
 * @brief	选择ROI组
 * @param[in]	group_id
 * @note
 * @retval	成功返回 RET_OK
 * @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_SelectROIGroup(HPS3D_HandleTypeDef *handle, uint8_t group_id);

/**
 * @brief	获取当前ROI组ID
 * @param[out]  group_id
 * @note
 * @retval  成功返回 RET_OK
 * @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_GetROIGroupID(HPS3D_HandleTypeDef *handle, uint8_t *group_id);

/**
 * @brief	设置ROI的警报类型
 * @param[in]
 * @see ROIAlarmTypeDef
 * @note
 * @retval	成功返回 RET_OK @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_SetROIAlarmType(HPS3D_HandleTypeDef *handle, uint8_t roi_id, uint8_t threshold_id, ROIAlarmTypeDef roi_alarm_type);

/**
 * @brief	设置ROI的参考值类型
 * @param[in]
 * @see ROIReferenceTypeDef
 * @note
 * @retval	成功返回 RET_OK @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_SetROIReferenceType(HPS3D_HandleTypeDef *handle, uint8_t roi_id, uint8_t threshold_id, ROIReferenceTypeDef ref_type);

/**
 * @brief	设定ROI区域
 * @param[in]	roi_conf
 * @see ROIConfTypeDef
 * @note
 * @retval	成功返回 RET_OK @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_SetROIRegion(HPS3D_HandleTypeDef *handle, ROIConfTypeDef roi_conf);

/**
 * @brief	设置ROI使能
 * @param[in]	roi_id
 * @param[in]	en
 * @note
 * @retval	成功返回 RET_OK @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_SetROIEnable(HPS3D_HandleTypeDef *handle, uint32_t roi_id, bool en);

/**
 * @brief	设置ROI阈值使能
 * @param[in]	roi_id
 * @param[in]	threshold_id
 * @param[in]	en
 * @note
 * @retval	成功返回 RET_OK @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_SetROIThresholdEnable(HPS3D_HandleTypeDef *handle, uint32_t roi_id, uint32_t threshold_id, bool en);

/**
 * @brief	设置ROI阈值配置
 * @param[in]	hysteresis_conf
 * @see HysteresisSingleConfTypeDef
 * @note
 * @retval	成功返回 RET_OK @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_SetROIThresholdConf(HPS3D_HandleTypeDef *handle, uint32_t roi_id, uint32_t threshold_id, uint16_t pix_num_threshold, HysteresisSingleConfTypeDef hysteresis_conf);

/**
 * @brief	获取当前设备支持的ROI数量和阈值数量
 * @param[out]	roi_number
 * @param[out]	threshold_number
 * @note
 * @retval	成功返回 RET_OK @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_GetNumberOfROI(HPS3D_HandleTypeDef *handle, uint8_t *roi_number, uint8_t *threshold_number);

/**
 * @brief	获取指定的ROI配置
 * @param[out]	roi_conf
 * @see ROIConfTypeDef
 * @note
 * @retval	成功返回 RET_OK @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_GetROIConfById(HPS3D_HandleTypeDef *handle, uint8_t roi_id, ROIConfTypeDef *roi_conf);

/**
 * @brief	设置指定的GPIO输出端口的配置
 * @param[in]	gpio_out_conf
 * @see GPIOOutConfTypeDef
 * @note        gpio_out_conf：只能配置IO输出
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetGPIOOut(HPS3D_HandleTypeDef *handle, GPIOOutConfTypeDef gpio_out_conf);

/**
 * @brief	获取指定GPIO输出端口的配置
 * @param[in]	gpio_out_conf
 * @see GPIOOutConfTypeDef
 * @note
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetGPIOOutConf(HPS3D_HandleTypeDef *handle, GPIOOutConfTypeDef *gpio_out_conf);

/**
 * @brief	设置指定的GPIO输入端口的配置
 * @param[in]	gpio_in_conf
 * @see GPIOInConfTypeDef
 * @note	gpio_in_conf：只能配置IO输入
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetGPIOIn(HPS3D_HandleTypeDef *handle, GPIOInConfTypeDef gpio_in_conf);

/**
 * @brief	获取指定GPIO输入端口的配置
 * @param[out]	gpio_in_conf
 * @see GPIOInConfTypeDef
 * @note
 * @retval	 成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetGPIOInConf(HPS3D_HandleTypeDef *handle, GPIOInConfTypeDef *gpio_in_conf);

/**
 * @brief	设置HDR模式
 * @param[in]	hdr_mode
 * @see HDRModeTypeDef
 * @note
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetHDRMode(HPS3D_HandleTypeDef *handle, HDRModeTypeDef hdr_mode);

/**
 * @brief	设置HDR
 * @param[in] hdr_conf
 * @see 	HDRConf
 * @note
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetHDRConfig(HPS3D_HandleTypeDef *handle, HDRConf hdr_conf);

/**
 * @brief	获取HDR配置
 * @param[out] hdr_conf
 * @see		HDRConf
 * @note
 * @retval	 成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetHDRConfig(HPS3D_HandleTypeDef *handle, HDRConf *hdr_conf);

/**
 * @brief	设置距离滤波器类型
 * @param[in]	distance_filter_conf
 * @see	DistanceFilterTypeDef
 * @note
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetDistanceFilterType(HPS3D_HandleTypeDef *handle, DistanceFilterTypeDef distance_filter_conf);

/**
 * @brief	配置距离滤波器
 * @param[in]	distance_filter_conf
 * @see DistanceFilterConfTypeDef
 * @note
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetSimpleKalman(HPS3D_HandleTypeDef *handle, DistanceFilterConfTypeDef distance_filter_conf);

/**
 * @brief	获取距离滤波器配置
 * @param[out]	distance_filter_conf
 * @see		DistanceFilterConfTypeDef
 * @note
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetDistanceFilterConf(HPS3D_HandleTypeDef *handle, DistanceFilterConfTypeDef *distance_filter_conf);

/**
 * @brief	设置平滑滤波器
 * @param[in]	smooth_filter_conf
 * @see	SmoothFilterConfTypeDef
 * @note
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetSmoothFilter(HPS3D_HandleTypeDef *handle, SmoothFilterConfTypeDef smooth_filter_conf);

/**
 * @brief	获取平滑滤波器的配置
 * @param[out]	smooth_filter_conf
 * @see	SmoothFilterConfTypeDef
 * @note
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetSmoothFilterConf(HPS3D_HandleTypeDef *handle, SmoothFilterConfTypeDef *smooth_filter_conf);

/**
 * @brief	设定光学参数使能
 * @param[in]	en
 * @note	光学参数补偿是将实际光程转换为水平方向距离
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetOpticalEnable(HPS3D_HandleTypeDef *handle, bool en);

/**
 * @brief	获取光学参数
 * @param[out]	optical_param_conf
 * @see	OpticalParamConfTypeDef
 * @note
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetOpticalParamConf(HPS3D_HandleTypeDef *handle, OpticalParamConfTypeDef *optical_param_conf);

/**
* @brief	设置距离补偿
* @note
* @retval	成功返回 RET_OK
*/
extern RET_StatusTypeDef HPS3D_SetDistanceOffset(HPS3D_HandleTypeDef *handle, int16_t offset);

/**
* @brief	获得距离补偿
* @param[out]	offset
* @note
* @retval	成功返回 RET_OK
*/
extern RET_StatusTypeDef HPS3D_GetDistanceOffset(HPS3D_HandleTypeDef *handle, int16_t *offset);

/**
 * @brief	设置多机干扰检测使能
 * @param[in]	en
 * @note
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetInterferenceDetectEn(HPS3D_HandleTypeDef *handle, bool en);


/**
 * @brief	设定安装角度变换使能
 * @param[in]	en
 * @note
 * @retval	成功返回 RET_OK
 */
/*extern RET_StatusTypeDef HPS3D_SetMountingAngleEnable(HPS3D_HandleTypeDef *handle, bool en);*/ /*此接口不再使用，采用以下接口替换*/

/**
 * @brief	设定安装角度变换参数
 * @param[in]	mounting_angle_param_conf
 * @see 		MountingAngleParamTypeDef
 * @note
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetMountingAngleParamConf(HPS3D_HandleTypeDef *handle, MountingAngleParamTypeDef mounting_angle_param_conf);

/**
 * @brief	获取安装角度变换参数
 * @param[out]	mounting_angle_param_conf
 * @see 		MountingAngleParamTypeDef
 * @note
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetMountingParamConf(HPS3D_HandleTypeDef *handle, MountingAngleParamTypeDef *mounting_angle_param_conf);



/*************************************2.集成函数接口**********************************/

/**
 * @brief	获取目录下指定前缀文件(自动寻找设备)
 * @param[in]	dirPath 设备文件根目录
 * @param[in]   prefix  设备文件名前缀
 * @param[out]  fileName 用于保存当前当前目录下寻找到的设备
 * @note		例：n = HPS3D_GetDeviceList("/dev/","ttyACM",fileName);
 * @retval	返回获取成功的数量 0表示失败
 */
extern uint32_t HPS3D_GetDeviceList(char *dirPath,char *prefix,char fileName[DEV_NUM][DEV_NAME_SIZE]);

/**
  * @brief	设置串口属性
  * @param[in] uartConfig
  * @note
  * @see UartConfTypeDef
  * @code
  * @retval	成功返回 RET_OK
  */
RET_StatusTypeDef HPS3D_SetUartConfig(UartConfTypeDef uartConfig);

/**
 * @brief	设备连接
 * @param[out]	handle->ConnectStatus
 * @note
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_Connect(HPS3D_HandleTypeDef *handle);

/**
 * @brief	断开连接
 * @param[out]	handle->ConnectStatus
 * @note
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_DisConnect(HPS3D_HandleTypeDef *handle);


/**
 * @brief	设备初始化
 * @param[in]
 * @note	创建线程并得到传感器初始化参数，申请内存空间;
 * 			若使用@see HPS3D_AutoConnectAndInitConfigDevice
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_ConfigInit(HPS3D_HandleTypeDef *handle);


/**
 * @brief	设置ROI的阈值
 * @param[in]	threshold_id
 * @param[in]	roi_conf
 * @see ROIConfTypeDef
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetThreshold(HPS3D_HandleTypeDef *handle, uint8_t threshold_id, ROIConfTypeDef roi_conf);

/**
 * @brief	设置ROI参数
 * @param[in]   roi_conf
 * @see ROIConfTypeDef  GPIOOutConfTypeDef
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetSingleRoi(HPS3D_HandleTypeDef *handle, ROIConfTypeDef roi_conf, GPIOOutConfTypeDef gpio_out_conf);

/**
 * @brief	添加观察者(注册回调函数通知事件)
 * @param[in]	Observer_t
 * @see AsyncIObserver_t
 * @note 	异步模式(连续测量模式使用此模式)
 * @retval  成功返回RET_OK
 */
extern RET_StatusTypeDef HPS3D_AddObserver(void * (*fun)(HPS3D_HandleTypeDef *,AsyncIObserver_t *),HPS3D_HandleTypeDef *handle,AsyncIObserver_t *Observer_t);

/**
 * @brief	 移除观察者(注销通知事件)
 * @param[in]	Observer_t
 * @see		AsyncIObserver_t
 * @note	异步模式(连续测量模式使用此模式)
 * @retval 成功返回RET_OK
 */
RET_StatusTypeDef HPS3D_RemoveObserver(AsyncIObserver_t *Observer_t);


/**
 * @brief	设备卸载与资源回收
 * @param[in]
 * @note
 * @retval 成功返回RET_OK
 */
extern RET_StatusTypeDef HPS3D_RemoveDevice(HPS3D_HandleTypeDef *handle);

/**
 * @brief		设置debug使能
 * @param[in]	en
 * @note
 * @retval	 成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetDebugEnable(bool en);

/**
 * @brief	获得debug使能状态
 * @param
 * @note
 * @retval	返回debug使能状态值
 */
extern bool HPS3D_GetDebugEnable(void);

/**
 * @brief		接收回调函数的地址
 * @param[in]	void *Call_Back     接收回调函数地址   回调函数为void *fun(uint8_t *str, uint16_t *str_len){...}
 * @param[out]	返回给回调函数str和strlen
 * @note
 * @retval	 成功返回 RET_OK
 */
RET_StatusTypeDef HPS3D_SetDebugFunc(void (*Call_Back)(char *str));

/**
 * @brief	设置点云数据转换使能
 * @param[in]	en
 * @note
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetPointCloudEn(bool en);

/**
 * @brief	获取点云数据转换使能状态
 * @param
 * @note
 * @retval	true or false
 */
extern bool HPS3D_GetPointCloudEn(void);


/**
 * @brief		单次测量
 * @param[out]	handle->RetPacketType	返回包数据类型
 * @param[out]	handle->MeasureData		测量数据
 * @note   该方式为同步测量，即调用此函数后立即得到测量返回值
 * @see
 * @code
 * @retval	 成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SingleMeasurement(HPS3D_HandleTypeDef *handle);

/**
 * @brief		初始化障碍物提取参数
 * @param[in]
 * @note
 * @see ObstacleConfigTypedef
 * @code
 *
 * @retval	 成功返回 RET_OK
 */
extern ObstacleConfigTypedef HPS3D_GetObstacleConfigInit(void);

/**
 * @brief		获取障碍物提取配置参数
 * @param
 * @note
 * @see ObstacleConfigTypedef
 * @code
 * @retval	成功返回RET_OK
 */
extern RET_StatusTypeDef HPS3D_ObstacleConfigInit(ObstacleConfigTypedef Conf);

/**
 * @brief		设置障碍物像素点个数阈值
 * @param
 * @note  通过设定此阈值可过滤细小障碍物
 * @see
 * @code
 * @retval none
 */
extern void HPS3D_SetObstaclePixelNumberThreshold(uint32_t pixel_num_thr);

/**
 * @brief		获取障碍物像素点个数阈值
 * @param
 * @note
 * @see
 * @code
 * @retval  返回像素点阈值
 */
extern uint32_t HPS3D_GetObstaclePixelNumberThreshold(void);


/**
 * @brief	设置障碍物提取阈值偏置
 * @param
 * @note
 * @see
 * @code
 * @retval  none
 */
extern void HPS3D_SetThresholdOffset(int32_t thr_offset);

/**
 * @brief	获取障碍物提取阈值偏置
 * @param
 * @note
 * @see
 * @code
 * @retval  返回当前阈值
 */
extern int32_t HPS3D_GetThresholdOffset(void);


/**
 * @brief	获取SDK版本号
 * @param
 * @note
 * @see
 * @code
 * @retval	 版本信息
 */
extern Version_t HPS3D_GetSDKVersion(void);


/**
 * @brief		将特殊测量输出值转换为指定特殊值参数配置
 * param[in]	enable
 * param[in]    value
 * @note
 * @see
 * @code
 * @retval	 成功返回RET_OK
 */
extern RET_StatusTypeDef HPS3D_ConfigSpecialMeasurementValue(bool enable,uint16_t value);

/**
 * @brief		设置边缘噪声滤除使能
 * param[in]
 * @note
 * @see
 * @code
 * @retval	成功返回RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetEdgeDetectionEnable(bool en);

/**
 * @brief		获取边缘噪声滤除使能
 * param[in]
 * @note
 * @see
 * @code
 *
 * @retval	 返回状态值
 */
extern bool HPS3D_GetEdgeDetectionEnable(void);

/**
 * @brief		设置边缘噪声阈值
 * param[in]
 * @note
 * @see
 * @code
 *
 * @retval	 成功返回RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetEdgeDetectionValue(int32_t threshold_value);

/**
 * @brief	获取边缘噪声阈值
 * param[in]
 * @note
 * @see
 * @code
 *
 * @retval	 返回当前阈值
 */
extern int32_t HPS3D_GetEdgeDetectionValue(void);

/**
  * @brief	保存点云数据为ply格式文件
  * @param[in]
  * @param[in] point_cloud_data
  * @note
  * @see PointCloudDataTypeDef
  * @code
  * @retval 成功返回RET_OK
  */
extern RET_StatusTypeDef HPS3D_SavePlyFile(char *filename,PointCloudDataTypeDef point_cloud_data);

/**
  * @brief	提取障碍物避障方案(3IO版本通过IO输出报警值)
  * @param[in] MeasureData
  * @param[in] AvoidConf
  * @note
  * @see AvoidanceTypeDef
  * @code
  * @retval	成功返回RET_OK
  */
extern RET_StatusTypeDef HPS3D_ObstacleAvoidance(HPS3D_HandleTypeDef *handle,MeasureDataTypeDef *MeasureData,AvoidanceTypeDef *AvoidConf);

/**
  * @brief	设置点云数据镜像
  * @param[in] type 镜像方向(水平或垂直)
  * @note
  * @see PointCloudImageMirrorTypeDef
  * @code
  * @retval	成功返回RET_OK
  */
extern RET_StatusTypeDef HPS3D_SetPointCloudMirrorType(PointCloudImageMirrorTypeDef type);
/**
  * @brief	获取点云数据镜像
  * @param[in]
  * @note
  * @see PointCloudImageMirrorTypeDef
  * @code
  * @retval PointCloudImageMirrorTypeDef
  */
extern PointCloudImageMirrorTypeDef HPS3D_GetPointCloudMirrorType(void);

/**
  * @brief	设置服务器信息
  * @param[in]
  * @note eg: char *serverIP = "192.168.0.10";
  *         serverPort = 12345;  此函数需在connect之前调用
  * @see
  * @code
  * @retval	成功返回RET_OK
  */
extern RET_StatusTypeDef HPS3D_SetEthernetServerInfo(HPS3D_HandleTypeDef *handle,char *serverIP,uint16_t serverPort);

/**
  * @brief	重置服务器IP（修改设备默认IP,不建议在此修改）
  * @param[in]  serverIP
  * @param[in]  serverPort
  * @param[in]  netmask
  * @param[in]  gateway
  * @note eg:serverIP[4] = {192,168,0,10};
  *         netMask[4] = {255,255,255,0};
  *         geteway[4] = {192,168,0,1};
  *         serverPort = 12345;
  * @see
  * @code
  * @retval	成功返回RET_OK
  */
RET_StatusTypeDef HPS3D_ConfigEthernet(HPS3D_HandleTypeDef *handle,uint8_t *serverIP,uint16_t serverPort,uint8_t *netMask,uint8_t *geteway);

/**
  * @brief	多设备自动连接并初始化(连接数量受DEV_NUM限制,可修改)
  * @param[in]
  * @note
  * @see
  * @code
  * @retval	返回连接成功的数量
  */
extern uint8_t HPS3D_AutoConnectAndInitConfigDevice(HPS3D_HandleTypeDef *handle);

/**
 * @brief	设置多机编码
 * @param[in]	CameraCode	 多机编码
 * @note 多机编码仅支持固件版本为1.7.62及以上版本，且编码序号为0-15
 * @see
 * @code
 *
 * @retval	成功返回 RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetMultiCameraCode(HPS3D_HandleTypeDef *handle, uint8_t CameraCode);

/**
 * @brief	获取多机编码
 * @param[in]	handle
 * @note
 * @see
 * @code
 *
 * @retval	返回当前多机编码值
 */
extern uint8_t HPS3D_GetMultiCameraCode(HPS3D_HandleTypeDef *handle);

/**
  * @brief 设置心跳检测
  * @param[in] handle
  * @param[in] enable 心跳检测使能
  * @param[in] time_ms 心跳检测时间ms
  * @note 调用此接口后，在HPS3D_SetRunMode前需先发送一次心跳包HPS3D_SendKeepAlive
  * @see
  * @code
  * @retval	成功返回RET_OK
  */
extern RET_StatusTypeDef HPS3D_SetKeepAliveConfig(HPS3D_HandleTypeDef *handle, bool enable, uint32_t time_ms);

/**
  * @brief 发送保活指令
  * @param[in] handle
  * @note  此接口需要在设定心跳检测时间内定时发送，发送此命令后只需检测在设定时间内是否有心跳返回包即可
  * @see RetPacketTypedef 中的KEEP_ALIVE_PACKET
  * @code
  * @retval	成功返回RET_OK
  */
extern RET_StatusTypeDef HPS3D_SendKeepAlive(HPS3D_HandleTypeDef *handle);


#ifdef __cplusplus
}
#endif

#endif /* API_H_ */
