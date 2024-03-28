#ifndef _NVILIDAR_PROTOCOL_H_
#define _NVILIDAR_PROTOCOL_H_

#include <stdint.h>
#include <vector>
#include <string>

//一包最多的点数信息
#define NVILIDAR_POINT_HEADER                  0xAA55        //包头信息

//one pack points 
#define NORMAL_NO_QUALITY_PACK_MAX_POINTS      8            //normal no quality 
#define NORMAL_HAS_QUALITY_PACK_MAX_POINTS     8            //normal has quality 
#define YW_HAS_QUALITY_PACK_MAX_POINTS         12           //yw has quality 
#define FS_HAS_QUALITY_PACK_MAX_POINTS         17           //fs has quality 

//pack 
#pragma pack(push)
#pragma pack(1)

//command enum 
typedef enum{
    PROTOCOL_VP100_NORMAL_NO_QUALITY = 0x0208,              //no quality 
    PROTOCOL_VP100_NORMAL_QUALITY = 0x0308,                 //has quality 
    PROTOCOL_VP100_YW_QUALITY = 0x070C,                     //yw quality has quality 
    PROTOCOL_VP100_FS_TEST_MODEL_QUAILIY = 0x0711,          //fs quality has quality 
    PROTOCOL_VP100_ERROR_FAULT = 0x8008                     //error code 
}VP100_LidarModel_Enum;

//single point info 
typedef struct{
    uint8_t    lidar_angle_zero_flag;       //is check angle 0 
    uint16_t   lidar_quality;               //quality info 
    float      lidar_angle;                 //check angle info 
    uint16_t   lidar_distance;              //current distance 
    uint64_t   lidar_stamp;                 //timestamp 
    float      lidar_speed;                 //scan speed 
    uint32_t   lidar_point_time;            //time 
    uint8_t    lidar_index;                 //current index   
    uint8_t    lidar_error_package;         //error pack info 
}Nvilidar_Node_Info;

//lidar version info 
typedef struct{
    std::string downBoard_Model;            //lidar down model 
    std::string downBoard_HardVersion;      //lidar down hard version 
    std::string downBoard_SoftVersion;      //lidar down software version 
    std::string downBoard_ID;               //lidar down id
    std::string downBoard_Date;             //lidar down date
    std::string downBoard_UID;              //lidar down uid 
    std::string downBoard_InputVoltage;     //lidar down voltage 
    std::string downBoard_BuildTime;        //lidar down buildtime

    std::string upBoard_Model;              //lidar up model 
    std::string upBoard_HardVersion;        //lidar up hard version 
    std::string upBoard_SoftVersion;        //lidar up soft version 
    std::string upBoard_ID;                 //lidar up id 
    std::string upBoard_Date;               //lidar up date 
    std::string upBoard_UID;                //lidar up uid
    std::string upBoard_BuildTime;          //lidar up build time
    uint8_t upBoard_VBD;                    //lidar up vbd
    uint8_t upBoard_TDC;                    //lidar up tdc
    int8_t  upBoard_Temperature;            //lidar up temperature 
}VP100_Head_LidarInfo_TypeDef;

//雷达信息 接收数据
typedef struct
{
    uint8_t   package_head;                 //包头
    uint8_t   package_cmd;                  //命令字
    uint8_t   package_length;               //长度
    uint8_t   package_data[255];            //有效数据 + 最后一个字节校验
}VP100_LidarInfo_DownBoard_Node_Package;

typedef struct
{
    uint16_t  package_head;                 //包头
    uint8_t   package_cmd;                  //命令字
    uint8_t   package_length;               //长度
    uint8_t   package_data[255];            //有效数据 + 最后一个字节校验
}VP100_LidarInfo_UpBoard_Node_Package;

//======================== normal no quality 
//normal no quality 
typedef struct {
    uint16_t PakageSampleDistance;
}VP100_Normal_Protocol_PackageNode_NoQuality;

//normal no quality 
typedef struct{
    uint16_t  package_head;                 //head 
    uint8_t   package_information;          //info 
    uint8_t   package_distanceDataNumber;   //point number 
    uint16_t  package_speed;                //speed 
    uint16_t  package_firstSampleAngle;     //start angle 
    VP100_Normal_Protocol_PackageNode_NoQuality  package_Sample[NORMAL_NO_QUALITY_PACK_MAX_POINTS]; //points 
    uint16_t  package_lastSampleAngle;      //last angle 
    uint16_t  package_checkSum;             //crc 
}VP100_Normal_Node_Package_No_Quality;

//======================== normal has quality 
//normal has quality 
typedef struct{
    uint16_t PakageSampleDistance;
    uint8_t  PakageSampleQuality;
}VP100_Normal_Protocol_PackageNode_Quality;

//normal has quality 
typedef struct{
    uint16_t  package_head;                 //head
    uint8_t   package_information;          //info
    uint8_t   package_distanceDataNumber;   //point number 
    uint16_t  package_speed;                //speed
    uint16_t  package_firstSampleAngle;     //start angle 
    VP100_Normal_Protocol_PackageNode_Quality  package_Sample[NORMAL_HAS_QUALITY_PACK_MAX_POINTS]; //points 
    uint16_t  package_lastSampleAngle;      //last angle 
    uint16_t  package_checkSum;             //crc
}VP100_Normal_Node_Package_Quality;

//===================yw protocol has quality 
//yw has quality 
typedef struct{
    uint16_t PakageSampleDistance;
    uint16_t  PakageSampleQuality;
}VP100_YW_Protocol_PackageNode_Quality;
//yw has quality 
typedef struct
{
    uint16_t  package_head;                 //head
    uint8_t   package_information;          //info
    uint8_t   package_distanceDataNumber;   //point number 
    uint16_t  package_speed;                //speed
    uint16_t  package_firstSampleAngle;     //start angle 
    VP100_YW_Protocol_PackageNode_Quality  package_Sample[YW_HAS_QUALITY_PACK_MAX_POINTS]; //points 
    uint16_t  package_lastSampleAngle;      //last angle 
    uint16_t  package_checkSum;             //crc
}VP100_YW_Node_Package_Quality;

//===================fs protocol has quality 
//fs has quality 
typedef struct{
    uint16_t PakageSampleDistance;
    uint16_t PakageSampleQuality;
}VP100_FS_Test_Protocol_PackageNode_Quality;
//fs has quality 
typedef struct{
    uint16_t  package_head;                 //head
    uint8_t   package_information;          //info
    uint8_t   package_distanceDataNumber;   //point number 
    uint16_t  package_speed;                //speed
    uint16_t  package_firstSampleAngle;     //start angle 
    VP100_FS_Test_Protocol_PackageNode_Quality  package_Sample[FS_HAS_QUALITY_PACK_MAX_POINTS]; //points 
    uint16_t  package_lastSampleAngle;      //last angle 
    uint16_t  package_checkSum;             //crc
}VP100_FS_Test_Node_Package_Quality;

//====================lidar error code 
typedef struct{
    uint16_t  package_head;                 //head
    uint8_t   package_information;          //info
    uint8_t   package_distanceDataNumber;   //point number 
    uint8_t   package_constByteFirst;       //0x45 const byte
    uint8_t   package_errorCode;            //error code 
    uint8_t   package_constByteSecond;      //0x00 const byte
    uint8_t   package_checkAddSum;          //check sum 
}VP100_Error_Fault_TypeDef;


//pack info with union 
typedef union{
    VP100_Normal_Node_Package_Quality               vp100_normal_quality;       
    VP100_Normal_Node_Package_No_Quality            vp100_normal_no_quality;    
    VP100_YW_Node_Package_Quality                   vp100_yw_quality;        
    VP100_FS_Test_Node_Package_Quality              vp100_fs_test_quality;  
    VP100_Error_Fault_TypeDef                       vp100_error_fault_info;
    VP100_LidarInfo_UpBoard_Node_Package            vp100_lidar_info_upboard;   //upboard info 
    VP100_LidarInfo_DownBoard_Node_Package          vp100_lidar_info_downboard; //downboard info 
    uint8_t  buf[1024];
}VP100_Node_Package_Union;

#pragma pack(pop)

//get the protocol type size of the lidar model 
#define GET_LIDAR_DATA_SIZE(model) \
    (   \
      (model == PROTOCOL_VP100_NORMAL_NO_QUALITY) ? sizeof(VP100_Normal_Node_Package_No_Quality) : \
      (model == PROTOCOL_VP100_NORMAL_QUALITY) ? sizeof(VP100_Normal_Node_Package_Quality) : \
      (model == PROTOCOL_VP100_YW_QUALITY) ? sizeof(VP100_YW_Node_Package_Quality) : \
      (model == PROTOCOL_VP100_FS_TEST_MODEL_QUAILIY) ? sizeof(VP100_FS_Test_Node_Package_Quality) : \
      (model == PROTOCOL_VP100_ERROR_FAULT) ? sizeof(VP100_Error_Fault_TypeDef) : \
      0 \
    )


#endif
