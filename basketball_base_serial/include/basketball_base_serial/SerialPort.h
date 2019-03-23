/*
 * SerialPort.h
 *
 *  Created on: 2012-4-8
 *      Author: startar
 */

#ifndef SERIALPORT_H_
#define SERIALPORT_H_

#include <ros/ros.h>
#include <inttypes.h>
#include <vector>
#include <queue>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include <iostream>


namespace serial {

// 串口通信选项
class SerialParams {
public:
    std::string serialPort; 					// 串口的设备文件
    unsigned int baudRate; 				// 波特率
    unsigned int flowControl; 			// 流控
    unsigned int parity; 				// 校验位
    unsigned int stopBits; 				// 停止位
    SerialParams() :
            serialPort("/dev/ttyUSB0"), baudRate(115200), flowControl(0), parity(0), stopBits(1)
    {
    }
    SerialParams(
            std::string _serialPort ,
            unsigned int _baudRate ,
            unsigned int _flowControl = 0 ,
            unsigned int _parity = 0 ,
            unsigned int _stopBits = 0 
            ) :
            serialPort(_serialPort),
            baudRate(_baudRate),
            flowControl(_flowControl),
            parity(_parity),
            stopBits(_stopBits)
    {
    }
};

typedef std::vector<uint8_t> ByteVector;
typedef boost::shared_ptr<ByteVector> pByteVector;

class SerialPort {
private:
    boost::shared_ptr<boost::asio::deadline_timer>	m_ptimer;		// 超时定时器
    boost::shared_ptr<boost::asio::io_service> 	m_pios;				// io_service对象
    boost::shared_ptr<boost::asio::serial_port>	m_pSerial;			// 串口对象的指针
    boost::mutex 					m_serialMutex;		// 串口对象的互斥锁. 按照boost官方文档, serial_port对象不是线程安全的. 故需要此锁

    enum {HEADER_LEN = 4};
    //enum {HEADER_LEN = 3};

    enum STATE {//使用了枚举类型讲每种工作状态都列举了出来
        WAITING_FF, WAITING_FF2, READING_HEAD, READING_DATA, READING_CHECKSUM
    } m_state;								// 程序工作状态

    SerialParams	m_serialParams; 		// 串口的配置数据

    int				m_timeOut; 				// 数据报超时时间

    int num ;
    unsigned int checksum;

    ByteVector		m_tempBuf;				// 数据读取的临时缓冲区
    //uint8_t  *current_array ;

    ByteVector 		m_currentHeader;		// 正在读取的报头(4字节)
    size_t 			m_HeaderBytesRead;		// 报头已经读取的字节数

    ByteVector 		m_currentData;			// 正在读取的报文数据
    size_t 			m_DataBytesRead;		// 数据已经读取的字节数

    std::queue<pByteVector>	m_writeQueue;		// 待发送数据的队列
    boost::mutex									m_writeQueueMutex;	// 队列的互斥锁

    boost::function<void(ByteVector, int)> m_dataCallbackFunc;		// 数据回调函数
    boost::function<void()> m_errorCallbackFunc;	// 错误回调函数

    // 跑io_service::run()的线程
    boost::thread m_thread;

    // 线程的主过程, 主要是在跑io_service::run()
    void mainRun();

    // 为了方便写的函数
    void start_a_read();
    void start_a_write();

    // async_read_some的Handler
    void readHandler(const boost::system::error_code &ec, size_t bytesTransferred);
    // async_write_some的Handler
    void writeHandler(const boost::system::error_code &ec);
    // 超时定时器的Handler
    void timeoutHandler(const boost::system::error_code &ec);

public:
    SerialPort();
    
    virtual ~SerialPort();

    void setSerialParams(const SerialParams &params);		// 设置串口参数

    void setTimeOut(int timeout);							// 设置超时时间

    bool startThread();										// 启动线程
    bool stopThread();										// 停止线程

    // 设置收到数据之后的回调函数
    void setCallbackFunc(const boost::function<void(ByteVector,int)> &func);

    // 向串口中发送一个数据报文
    //bool writeDatagram(const XM_msgs::XM_Datagram &datagram);
    // 向串口中直接写入一串数据
    bool writeRaw(const ByteVector &rawData);
};

} /* namespace XM_SerialNode */

#endif /* SERIALPORT_H_ */
