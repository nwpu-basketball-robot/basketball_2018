/*
 * SerialPort.cpp
 *
 *  Created on: 2012-4-8
 *      Author: startar
 */

#include <basketball_base_serial/SerialPort.h>
#include <vector>
#include <boost/make_shared.hpp>

//using namespace std;
//using namespace boost;

namespace serial {

SerialPort::SerialPort()
{
    std::cout << "SerialPort Object created!" << std::endl;


    m_state=WAITING_FF ;

    m_tempBuf.resize(1024, 0);

}

SerialPort::~SerialPort()
{
    m_pios->stop();
    m_thread.join();
}

void SerialPort::start_a_read()
{

    boost::mutex::scoped_lock lock(m_serialMutex);

    // 启动一次异步读
    m_pSerial->async_read_some(boost::asio::buffer(m_tempBuf), boost::bind(&SerialPort::readHandler,
            this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred
            ));
}

void SerialPort::start_a_write()
{
    boost::mutex::scoped_lock lock(m_serialMutex);

    // 启动一次异步写
    boost::asio::async_write(*m_pSerial, boost::asio::buffer(*(m_writeQueue.front())),
            bind(&SerialPort::writeHandler, this, boost::asio::placeholders::error));
}

void SerialPort::mainRun()
{
    std::cout << "SerialPort mainThread STARTED!" << std::endl;

    // 初始化工作状态
    m_state = WAITING_FF;

    // 设置Header存储区的大小为4字节
    m_currentHeader.resize(4, 0);

    // 启动一次异步读
    start_a_read();

    // 开始跑io_service::run()
    m_pios->run();

    std::cout << "SerialPort mainThread EXITED!" << std::endl;
}

void SerialPort::readHandler(const boost::system::error_code &ec, size_t bytesTransferred)
{
    if (ec)
    {
        // TODO: 报错
        std::cout << "SerialPort read error !!" << std::endl;
        return;
    }

//    unsigned int checksum = 0;
    // 处理每个读到的字符
    for (size_t i=0; i<bytesTransferred; i++)
    {
        // 新拿一个字节出来
        uint8_t byte = m_tempBuf.at(i);
        //printf("State=%d, Processing byte: 0x%X \n", m_state, byte);

        // 状态机...
        switch (m_state) {
        case WAITING_FF:
            // 如果新读到的字节是0xFF, 则转移状态至WAITING_FF2
            checksum = 0;
            if (byte==(uint8_t)0xFF)
            {
                 m_state = WAITING_FF2;
                 //std::cout << "Reading" << std::endl;

                    // 启动超时定时器
                    m_ptimer.reset(new boost::asio::deadline_timer(*m_pios,
                            boost::posix_time::milliseconds(m_timeOut)));
                    m_ptimer->async_wait(bind(
                            &SerialPort::timeoutHandler,
                            this,
                            boost::asio::placeholders::error));
            }
            break;
        case WAITING_FF2:
            // 如果新读到的字节的0xFF, 则转移状态至READING_HEAD, 否则回到WAITING_FF
            if (byte==(uint8_t)0xFF)
            {
                //std::cout << "Get a new datagram header !!" << std::endl;

                m_HeaderBytesRead = 0;
                m_state = READING_HEAD;
                //std::cout << "Reading" << std::endl;
            }
            else {
                m_state = WAITING_FF;
                //std::cout << "waiting" << std::endl;
            }
            break;
        case READING_HEAD:
            // 将新字节存入m_currentHeader
            m_currentHeader[m_HeaderBytesRead] = byte;
            checksum += byte;
            m_HeaderBytesRead++;
            //std::cout << "header:" << (int)byte << std::endl;

            if (HEADER_LEN == m_HeaderBytesRead)	// 如果4个字节都已经读出
            {
                // 读出数据报文的长度, 并将m_currentData的大小重设为该长度
                uint16_t dataLen =
                        ((uint16_t)(m_currentHeader[1]) << 8) + m_currentHeader[2];
                //std::cout << dataLen << std::endl ;
                //std::cout << "ok" << std::endl;
                if (dataLen>0&&dataLen<14)
                {
                    m_currentData.resize(dataLen-1, 0);
                    if(dataLen == 2)
                    {
                        ROS_WARN("Recv a detect msg\n") ;
                    }
                    m_DataBytesRead = 0;
                    m_state = READING_DATA;
                }
                else {
                    m_state = WAITING_FF;
                    //std::cout << "READING_HEAD ERROR" << std::endl;
                }
            }
            break;
        case READING_DATA:
            // 将新字节存入m_currentData

            //ROS_WARN("Reading Data");
            m_currentData[m_DataBytesRead++] = byte ;
            checksum += byte;
            //std::cout << "checksum is now :" << checksum << std::endl;
           // cout<<m_currentData.size()<<endl ;
//            std::cout << "data" << (int)byte << std::endl;
            if (m_currentData.size() == m_DataBytesRead)	// 如果所有字节都已经读出
            {
//                ROS_WARN("Reading Data");
                m_state = READING_CHECKSUM;
            }
            break;
        case READING_CHECKSUM:
            m_ptimer->cancel();
            m_ptimer.reset();
            // 计算校验和
            //for (size_t k=0; k<m_currentHeader.size(); k++)
                //byteSum += m_currentHeader.at(k);
            //for (size_t k=0; k<m_currentData.size(); k++)
            //    checksum += m_currentData.at(k);

            if ( (uint8_t)checksum == byte)
            {
                //std::cout << "A new datagram received !!" << std::endl;
                m_dataCallbackFunc(m_currentData,m_currentHeader[3]);
            }
            else {
                /*cout << "checksum is :" << (unsigned int)checksum << endl;
                cout << checksum%255 << endl;
                cout << (int)byte << endl;
                cout << "checksum error !!" << endl;*/
            }

            m_state = WAITING_FF;
            break;
        }
    }
    //std::cout << "Finish a read operation !!" << std::endl;
    start_a_read();
}

void SerialPort::writeHandler(const boost::system::error_code &ec)
{
    if (ec)
    {
        // TODO: 报错
    }

    {
        boost::mutex::scoped_lock lock(m_writeQueueMutex);
        m_writeQueue.pop();

        if (m_writeQueue.empty()==false)
        {
            start_a_write();
        }
    }
}

void SerialPort::timeoutHandler(const boost::system::error_code &ec)
{
    if (!ec)
    {
        std::cout << "Time Out !" << std::endl;
        m_state = WAITING_FF;
    }
}

void SerialPort::setSerialParams(const SerialParams &params)
{
    m_serialParams = params;
}

void SerialPort::setTimeOut(int timeout)
{
    m_timeOut = timeout;
}

bool SerialPort::startThread()
{
    std::cout << "SerialPort::startThread() called!" << std::endl;

    // 创建io_service对象
    m_pios = boost::make_shared<boost::asio::io_service>();

    try {
        // 创建一个serial_port对象, 替换掉原来的
        m_pSerial = boost::make_shared<boost::asio::serial_port>(boost::ref(*m_pios), m_serialParams.serialPort);

        // 设置串口通信参数
        // PS: 这里的强制转换不是什么好习惯... 我只是图省事
        m_pSerial->set_option(
                boost::asio::serial_port::baud_rate(m_serialParams.baudRate));
        m_pSerial->set_option(
                boost::asio::serial_port::flow_control((boost::asio::serial_port::flow_control::type)m_serialParams.flowControl));
        m_pSerial->set_option(
                boost::asio::serial_port::parity((boost::asio::serial_port::parity::type)m_serialParams.parity));
        m_pSerial->set_option(
                boost::asio::serial_port::stop_bits((boost::asio::serial_port::stop_bits::type)m_serialParams.stopBits));
        m_pSerial->set_option(boost::asio::serial_port::character_size(8));
    }
    catch (std::exception &e) {
        std::cout << "Failed to open serial port !" << std::endl;
        std::cout << "Error Info: " << e.what() << std::endl;
        return false;
    }

    try {
        // 创建线程
        m_thread = boost::thread(boost::bind(&SerialPort::mainRun, this));
    }
    catch (std::exception &e) {
        std::cout << "Failed to create thread !" << std::endl;
        std::cout << "Error Info: " << e.what() << std::endl;
        return false;
    }
    return true;
}

bool SerialPort::stopThread()
{
    m_pios->stop();
    return true;
}

void SerialPort::setCallbackFunc(const boost::function<void(ByteVector,int)> &func)
{
    m_dataCallbackFunc = func;
}

/*
bool SerialPort::writeDatagram(const XM_msgs::XM_Datagram &datagram)
{
    const size_t dataLen = datagram.data.size();



    // 准备数据
    ByteVector bufToSend(2 + HEADER_LEN + dataLen + 1, 0);
    bufToSend[0] = bufToSend[1] = (uint8_t)0xFF;
    bufToSend[2] = datagram.sender;
    bufToSend[3] = datagram.receiver;
    bufToSend[4] = (uint8_t)(dataLen >> 8);
    bufToSend[5] = (uint8_t)(dataLen & 0xFF);
    for (size_t i=0; i<dataLen; i++)
        bufToSend[6+i] = datagram.data.at(i);


/*
    // 准备数据
    ByteVector bufToSend(2 + HEADER_LEN-1 + dataLen + 1, 0);
    bufToSend[0] = bufToSend[1] = (uint8_t)0xFF;
    bufToSend[2] = datagram.sender;
    bufToSend[3] = datagram.receiver;
    bufToSend[4] = (uint8_t)(dataLen & 0xFF);
    for (size_t i=0; i<dataLen; i++)
        bufToSend[5+i] = datagram.data.at(i);



    // 计算校验和
    uint32_t byteSum = 0;
    for (size_t i=2; i<bufToSend.size()-1; i++)
        byteSum += bufToSend.at(i);
    bufToSend[bufToSend.size()-1] = (uint8_t)(byteSum % 255);

    cout << "Sending Bytes: " << endl;
    for (size_t i=0; i<bufToSend.size(); i++)
    {
        printf("%02X ", bufToSend.at(i));
    }
    cout << endl;

    // 调用writeRaw
    return writeRaw(bufToSend);
}
*/

bool SerialPort::writeRaw(const ByteVector &rawData)
{
    boost::mutex::scoped_lock lock(m_writeQueueMutex);	// 上锁

    bool writeIdle = m_writeQueue.empty();			// 检测当前队列是否为空
    pByteVector data(new ByteVector(rawData));
    m_writeQueue.push(data);						// 将数据拷贝一份, 并加入队列里去

    if (writeIdle) start_a_write();					// 如果没有在写数据, 则启动一次异步写过程

    return true;
}

} /* namespace XM_SerialNode */
