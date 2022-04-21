#ifndef YODAENCODER_H
#define YODAENCODER_H

//17位绝对值编码器

#include "common.h"


//Checksum 校验和.参数模型是CRC-16/MODBUS 多项式是8005
//内部已执行高/低CRC字节的交换
uint16_t CRC16_MODBUS(uint8_t *_pBuf, int _usLen);

//校验编码器数据是否正确
bool checkEncoderData(uint8_t *_pBuf);

//获取编码器数据的编号
int getEncoderID(uint8_t *_pBuf);

//获取编码器数值，字符串长度必须为11位
int getEncoderAngle(uint8_t *_pBuf);


#endif