#ifndef uart_atkTOF_h
#define uart_atkTOF_h

#include "uart_comm.h"

class UartAtkTof : public UartComm
{
public:
    UartAtkTof();

public:
    // 接收数据的数据分析
	virtual int rxAnalysis(const uint8_t *const recvBuf, uint32_t length); // 数据处理函数

    // ---- 重写 CommunicationProtocol 的 dataAnalysis --------
protected:
    virtual void dataAnalysis();

private:
    void distanceData();

public:
    float longestDistance; // unit: m
};

#endif