#include <stdint.h>

#define kMaxPointSize 1500
#define RAW_POINT_NUM     100
#define SINGLE_POINT_NUM  96
#define DUAL_POINT_NUM    48
#define TRIPLE_POINT_NUM  30
#define IMU_POINT_NUM     1

#pragma pack (1) /*指定按1字节对齐*/

typedef struct {
  uint8_t signature[16];    // 16 Byte
  uint8_t version[4];       // 4 Byte
  uint32_t magic_code;      // 4 Byte
} LvxFilePublicHeader;

typedef struct {
  uint32_t frame_duration;  // 4 Byte
  uint8_t device_count;     // 1 Byte
} LvxFilePrivateHeader;

typedef struct {
  uint8_t lidar_broadcast_code[16]; // 16 Byte
  uint8_t hub_broadcast_code[16];   // 16 Byte
  uint8_t device_index;             // 1 Byte
  uint8_t device_type;              // 1 Byte
  uint8_t extrinsic_enable;         // 1 Byte
  float roll;                       // 4 Byte                
  float pitch;                      // 4 Byte
  float yaw;                        // 4 Byte
  float x;                          // 4 Byte
  float y;                          // 4 Byte
  float z;                          // 4 Byte
} LvxDeviceInfo;

typedef struct {
  uint64_t current_offset;
  uint64_t next_offset;
  uint64_t frame_index;
} FrameHeader;

typedef struct {
  uint8_t device_index;
  uint8_t version;
  uint8_t port_id;
  uint8_t lidar_index;
  uint8_t rsvd;
  uint32_t error_code;
  uint8_t timestamp_type;
  uint8_t data_type;
  uint8_t timestamp[8];
  uint8_t raw_point[kMaxPointSize];
  uint32_t pack_size;
} LvxBasePackDetail;

#pragma pack()