#ifndef _NVILIDAR_PROTOCOL_H_
#define _NVILIDAR_PROTOCOL_H_

#include <stdint.h>
#include <vector>

//一包最多的点数信息
#define NVILIDAR_POINT_HEADER                  0xAA55        //包头信息

#define NVILIDAR_RESP_MEASUREMENT_CHECKBIT     (0x1)        //角度标记位是否有效
#define NVILIDAR_ANGULDAR_RESOLUTION           64           //角分辨率 即多少数表示为1度角

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
    PROTOCOL_VP100_FS_TEST_MODEL_QUAILIY = 0x0711           //fs quality has quality 
}VP100_LidarModel_Enum;


//single point info 
typedef struct{
    uint8_t    lidar_angle_zero_flag;     //is check angle 0 
    uint16_t   lidar_quality;             //quality info 
    float      lidar_angle;               //check angle info 
    uint16_t   lidar_distance;            //current distance 
    uint64_t   lidar_stamp;               //timestamp 
    float      lidar_speed;               //scan speed 
    uint32_t   lidar_point_time;          //time 
    uint8_t    lidar_index;               //current index   
    uint8_t    lidar_error_package;       //error pack info 
}Nvilidar_Node_Info;

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

//pack info with union 
typedef union{
    VP100_Normal_Node_Package_Quality               vp100_normal_quality;       
    VP100_Normal_Node_Package_No_Quality            vp100_normal_no_quality;    
    VP100_YW_Node_Package_Quality                   vp100_yw_quality;        
    VP100_FS_Test_Node_Package_Quality              vp100_fs_test_quality;   
    uint8_t  buf[0];
}VP100_Node_Package_Union;

#pragma pack(pop)

//get the protocol type size of the lidar model 
#define GET_LIDAR_DATA_SIZE(model) \
    (   \
      (model == PROTOCOL_VP100_NORMAL_NO_QUALITY) ? sizeof(VP100_Normal_Node_Package_No_Quality) : \
      (model == PROTOCOL_VP100_NORMAL_QUALITY) ? sizeof(VP100_Normal_Node_Package_Quality) : \
      (model == PROTOCOL_VP100_YW_QUALITY) ? sizeof(VP100_YW_Node_Package_Quality) : \
      (model == PROTOCOL_VP100_FS_TEST_MODEL_QUAILIY) ? sizeof(VP100_FS_Test_Node_Package_Quality) : \
      0 \
    )


#endif
