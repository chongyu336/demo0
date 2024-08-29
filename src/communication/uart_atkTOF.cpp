#include "uart_atkTOF.h"

UartAtkTof::UartAtkTof()
{
}

int UartAtkTof::rxAnalysis(const uint8_t *const recvBuf, uint32_t length)
{
	for (uint32_t i = 0; i < length; i++)
	{
		if (_rxFrame._index > 255)
			_rxFrame._index = 0;

		_rxFrame._header[_rxFrame._index++] = recvBuf[i];
		if (recvBuf[i] == 0x0A) // 一个帧结束符
		{
			dataAnalysis();
			_rxFrame._index = 0;
		}
	}
	return 0;
}

void UartAtkTof::dataAnalysis()
{
	if (_rxFrame._index == 0)
		return;
	if (_rxFrame._header[_rxFrame._index - 2] != 0x0D) // 一个帧的结束符标记倒数第二位
		return;

	distanceData();
}

// 求取最长距离
void UartAtkTof::distanceData()
{
	if (_rxFrame._index < 3) // 数据长度不满足 1data+2end minimum
		return;

	uint32_t tmp = 0;
	uint32_t dataindex = _rxFrame._index - 3;
	int cnt = 0;
	bool isvalid = true;
	while (1)
	{
		uint8_t btmp = _rxFrame._header[dataindex];
		uint32_t tmp10 = 0;
		if ((btmp >= '0') && (btmp <= '9'))
		{
			tmp10 = btmp - '0';
			for (int i = 0; i < cnt; i++)
			{
				tmp10 *= 10;
			}
			tmp += tmp10;
			cnt++;
			if (dataindex == 0)
				break;
			dataindex--;
		}
		else
		{ // 非数字
			isvalid = false;
			break;
		}
	}
	if (isvalid)
	{
		longestDistance = tmp * 1e-3f; // 毫米转米
	}
}
