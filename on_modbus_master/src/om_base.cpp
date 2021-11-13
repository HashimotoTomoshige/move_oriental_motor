/*
 * Copyright (c) 2019, ORIENTAL MOTOR CO.,LTD.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the ORIENTAL MOTOR CO.,LTD. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** @file	om_base.cpp
@brief シリアル通信の処理を行う
@details
@attention
@note

履歴 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
@version	Ver.1.04 Sep.30.2020 T.Takahashi
    - transDelay関数に時間がラウンドしたときの処理を追加
    - ファンクションコードもレスポンス内容に含めるように処理を追加

@version	Ver.1.01 Jul.22.2019 T.Takahashi
    - bugfix

@version	Ver.1.00 Mar.11.2019 T.Takahashi
	- 新規作成
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
#include <thread>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include "om_modbus_master/om_base.h"

using std::string;
using std::thread;

namespace om_modbusRTU_node
{

Base::Base()
{
  isEnabled_ = false;
  ros_mes_ = nullptr;
}

Base::~Base()
{
  closeComm();
}

/*---------------------------------------------------------------------------*/
/** レスポンス長のエラーチェック

@task       レスポンス長から無応答、例外応答の確認を行う
@param[in]	ドライバからのレスポンス
@param[in]	レスポンス長
@return
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::chkResponseLength(char *pFrm, int len)
{
  ISetResponse *pMesObj = ros_mes_;
  if(0 == len)
  {
    setState(STATE_ERROR, STATE_ERROR_NORESPONSE);
    throw NO_RESPONSE;
  }
  else if(EXCEPTION_RESPONSE_NUM == len)
  {
    setState(STATE_ERROR, STATE_ERROR_EXCEPTION_RESPONSE);
    for(int i = 0; i < EXCEPTION_RESPONSE_NUM; i++)
    {
      response_msg.data[i] = pFrm[i];
    }
    response_msg.slave_id = pFrm[RESPONSE_SLAVE_ID];
	response_msg.func_code = pFrm[RESPONSE_FUNCTION_CODE];
    pMesObj->setResponse(response_msg);
    throw EXCEPTION_RESPONSE_ERROR;
  }
  else if(8 == len)
  {
    setState(STATE_ERROR, STATE_ERROR_NONE);
  }
  else
  {
    setState(STATE_ERROR, STATE_ERROR_NONE);
  }
}


/*---------------------------------------------------------------------------*/
/**  COMポートクローズ

@task
@param[in]	なし
@return		  なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::closeComm(void)
{
  close(socket_fd_);
  ROS_INFO("----------- close: ------------- ");
}

/*---------------------------------------------------------------------------*/
/** 読み込み

@task
@param[in]	int fd			ファイルディスクリプタ
@param[out]	char *rdData	読み込みデータ
@param[in]	int rdLen		データ長
@return		読み込まれたバイト数
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int Base::commRead(int fd, char *rdData, int rdLen)
{
  int ret = 0;
  int available_size;
  
  ioctl(fd, FIONREAD, &available_size);
  
  if(available_size != 0)
  {
    ret = read(fd, rdData, rdLen);
  }
  if(0 > ret)
  {
    throw READ_ERROR;
  }

  return ret;
}

/*---------------------------------------------------------------------------*/
/** 書き込み

@task
@param[in]	int fd		ファイルディスクリプタ
@param[out]	vector<char>& wrData	書き込みデータ
@return		int 書き込まれたバイト数
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int Base::commWrite(int fd, std::vector<char>& wrData)
{
  int ret = 0;
  int available_size;
  
  ret = write(fd, wrData.data(), wrData.size());
  
  if(0 > ret)
  {
    throw WRITE_ERROR;
  }
 
  return ret;
}

/*---------------------------------------------------------------------------*/
/** エラーメッセージ表示

@task
@param[in]	int error
@return		なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::dispErrorMessage(int error)
{
  switch (error)
  {
  case EXCEPTION_RESPONSE_ERROR:
    ROS_ERROR("Error: Exception response");
    break;
  case NO_RESPONSE:
    ROS_ERROR("Error: No response");
    break;
  case WRITE_ERROR:
    ROS_ERROR("Error: write() function");
    break;
  case READ_ERROR:
    ROS_ERROR("Error: read() function");
    break;
  case TIMEOUT_ERROR:
    ROS_ERROR("Error: Time out");
    break;
  case SELECT_ERROR:
    ROS_ERROR("Error: select() function");
    break;
  default:
    break;
  }
}

/*---------------------------------------------------------------------------*/
/** 通信実行

@task 上位から配信されたときに実行
@param[in]	なし
@return		なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::exeComm(void)
{
  ISetResponse *pObj = ros_mes_;
  ros::Rate r(1000);
  while(1)
  {
    if(isEnabled_)
    {
      for(int i = 0; i < MAX_DATA_NUM; i++)
      {
        response_msg.data[i] = 0;
      }

      switch (query_data.func_code)
      {
        case FUNCTION_CODE_READ:
          transRead();
          break;
        case FUNCTION_CODE_WRITE:
          transWrite();
          break;
        case FUNCTION_CODE_READ_WRITE:
          transReadAndWrite();
          break;
        default:
          break;
      }
      setState(STATE_DRIVER, STATE_DRIVER_NONE);
      setState(STATE_MES, STATE_MES_NONE);
      isEnabled_ = false;
      pObj->setCommEnabled(false);
    }
    r.sleep();
  }
}

/*---------------------------------------------------------------------------*/
/** ボーレートを返す

@task
@param[in]	int baudrate ボーレート
@return		　int(termios構造体で定義されているボーレートを返す)
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int Base::getBaudRate(int baudrate)
{
  switch (baudrate)
  {
  case 9600:
    return B9600;
  case 19200:
    return B19200;
  case 38400:
    return B38400;
  case 57600:
    return B57600;
  case 115200:
    return B115200;
  case 230400:
    return B230400;
  default:
    return -1;
  }
}

/*---------------------------------------------------------------------------*/
/** 現在時刻を返す

@param		なし
@return		long 時間[ns]
@task
@details
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
long Base::getCurrentTimeLinux(void)
{
  struct timespec tv;
  clock_gettime(CLOCK_REALTIME, &tv);

  return tv.tv_nsec;
}

/*---------------------------------------------------------------------------*/
/** 初期化

@param[int]	RosMessage *ros_mes_obj
@param[int] FirstGenModbusRTU *modbus_obj[]
@return		なし
@task
@details
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::init(RosMessage *ros_mes_obj, FirstGenModbusRTU *modbus_obj[])
{
  string baudrate;

  ros_mes_ = ros_mes_obj;
  modbus_obj_ = &modbus_obj[0];

  ros::NodeHandle private_nh("~");
  private_nh.getParam("init_com", topic_com_);
  private_nh.getParam("init_baudrate", baudrate);
  topic_baudrate_ = stoi(baudrate);

  if(false == openComm(getBaudRate(topic_baudrate_), topic_com_))
  {
    throw;
  }
  startThread();
}

/*---------------------------------------------------------------------------*/
/** COMポートオープン

@task
@param[in]	int baudrate ボーレート
@param[in]	string port COMポート名
@return		  true:成功  false:失敗
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool Base::openComm(int baudrate, string port)
{
  socket_fd_ = open(port.c_str(), O_RDWR);
  if (0 > socket_fd_)
  {
    ROS_ERROR("Error: Can't opening serial port");
    return false;
  }
  if(-1 == baudrate)
  {
    ROS_ERROR("Error: Specified Baudrate out of range");
    return false;
  }

  struct serial_struct serial_setting;
  ioctl(socket_fd_, TIOCGSERIAL, &serial_setting);
  serial_setting.flags |= ASYNC_LOW_LATENCY;
  ioctl(socket_fd_, TIOCSSERIAL, &serial_setting);

  struct termios newtio;

  bzero(&newtio, sizeof(newtio));

  newtio.c_cflag = baudrate | CS8 | CLOCAL | CREAD | PARENB;
  newtio.c_iflag = 0;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;

  newtio.c_cc[VTIME] = 5;
  newtio.c_cc[VMIN] = 0;

  cfsetispeed(&newtio, baudrate);
  cfsetospeed(&newtio, baudrate);

  tcflush(socket_fd_, TCIFLUSH);
  tcsetattr(socket_fd_, TCSANOW, &newtio);

  ROS_INFO("----------- connect: ------------- ");
  return true;
}

/*---------------------------------------------------------------------------*/
/** ステータス更新

@task
@param[in]	int type (0:state_mes 1:state_driver 2:state_error)
@param[in]	int val 書き込み値
@return		　なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::setState(int type, int val)
{
  ISetResponse *pObj = ros_mes_;
  switch (type)
  {
    case STATE_MES:
      state_msg.state_mes = val;
      break;
    case STATE_DRIVER:
      state_msg.state_driver = val;
      break;
    case STATE_ERROR:
      state_msg.state_error = val;
      break;
    default:
      break;
  }
  pObj->setState(state_msg);
}

/*---------------------------------------------------------------------------*/
/** スレッド関数の開始

@task
@param[in]	なし
@return		  なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::startThread(void)
{
  thread th(&om_modbusRTU_node::Base::exeComm, this);
  th.detach();
}

/*---------------------------------------------------------------------------*/
/** 送信関数

@task
@param[in]		int fd		ファイルディスクリプタ
@param[in]		vector<char>& pCmd	送信データ
@param[out]		char *pRes	受信データ
@param[out]		int resLen	受信データ長
@return			  int 受信サイズ[Byte]
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int Base::transComm(int fd, std::vector<char>& pCmd, char *pRes, int resLen)
{
  long delayNs[2] = {5000000, 5000000};
  return transCommEx(fd, pCmd, pRes, resLen, delayNs);
}

/*---------------------------------------------------------------------------*/
/** 送信関数拡張

@task
@param[in]		int fd			ファイルディスクリプタ
@param[in]		vector<char>& *pCmd		送信データ
@param[out]		char *pRes		受信データ
@param[in]		int resLen		受信データ長
@param[in]		int delayNs[2]	送信遅延時間
@return			  int 受信サイズ[Byte]
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int Base::transCommEx(int fd, std::vector<char>& pCmd, char *pRes, int resLen, long delayNs[2])
{
  int rcvCnt = 0;
  int rcvSize = 0;
  long diffNs = 0;
  long startTime = 0;
  long endTime = 0;
  unsigned long inf = 1000000000;

  diffNs = transDelay(delayNs[0]);
  commWrite(fd, pCmd);
  if(0 < resLen)
  {
    startTime = getCurrentTimeLinux();
    do
    {
      rcvSize = commRead(fd, pRes + rcvCnt, resLen);
      rcvCnt += rcvSize;
      endTime = getCurrentTimeLinux();
      diffNs = endTime - startTime;
      if(diffNs < 0)
      {
        diffNs = (inf - startTime) + endTime;
      }

      if(diffNs > TIMEOUT)
      {
        break;
      }
    }while( (resLen - 1) >= rcvCnt );
  }
  diffNs = transDelay(delayNs[1]);
  return rcvCnt;
}

/*---------------------------------------------------------------------------*/
/** 遅延処理

@task
@param[in]	遅延秒数[ns]
@return			遅延秒数[ns]
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
long Base::transDelay(long delayNs)
{
  long diffNs = 0;
  long startTime = 0;
  long endTime = 0;
  unsigned long inf = 1000000000;

  if(0 < delayNs)
  {
    startTime = getCurrentTimeLinux();
    while( diffNs < delayNs )
    {
      endTime = getCurrentTimeLinux();
      diffNs = endTime - startTime;      
      if(diffNs < 0)
      {
        diffNs = (inf - startTime) + endTime;
      }
    }
  }
  return diffNs;
}

/*---------------------------------------------------------------------------*/
/** 読み込み

@task
@param[in]  なし
@return     なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::transRead(void)
{
  int ret = 0;
  int len;
  std::vector<std::vector<char> > txd_data;
  txd_data.resize(MAX_QUERY_NUM);
  std::array<char, 255> rxd_data{0};

  ISetResponse *pMesObj = ros_mes_;
  IConvertQueryAndResponse *pObj = modbus_obj_[query_data.slave_id];
  pObj->setRead(query_data.slave_id, query_data.read_addr, query_data.read_num, txd_data);

  len = transComm(socket_fd_, txd_data[0], rxd_data.data(), 5 + query_data.read_num * 4);
  try
  {
    chkResponseLength(rxd_data.data(), len);
    response_msg.slave_id = rxd_data[RESPONSE_SLAVE_ID];
	response_msg.func_code = rxd_data[RESPONSE_FUNCTION_CODE];
    for(int i = 0; i < query_data.read_num; i++)
    {
      response_msg.data[i] = pObj->convertResponse(rxd_data.data(), i);
    }
    pMesObj->setResponse(response_msg);
    setState(STATE_MES, STATE_MES_NONE);
  }
  catch(int error)
  {
    dispErrorMessage(error);
    setState(STATE_MES, STATE_MES_ERROR);
  }
}

/*---------------------------------------------------------------------------*/
/** 読み込み&書き込み

@task
@param[in]  なし
@return     なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::transReadAndWrite(void)
{
  int ret = 0;
  int len;
  int query_num;
  int resLen[MAX_QUERY_NUM] = {0};
  std::vector<std::vector<char> > txd_data;
  txd_data.resize(MAX_QUERY_NUM);
  std::array<char, 255> rxd_data{0};

  ISetResponse *pMesObj = ros_mes_;
  IConvertQueryAndResponse *pObj = modbus_obj_[query_data.slave_id];
  query_num = pObj->setReadAndWrite(query_data.slave_id, query_data.read_addr, query_data.write_addr, query_data.read_num, query_data.write_num, query_data.data, txd_data, resLen);

  for(int i = 0; i < query_num; i++)
  {
    len = transComm(socket_fd_, txd_data[i], rxd_data.data(), resLen[i]);
    try
    {
      chkResponseLength(rxd_data.data(), len);
      if(8 == len)
      {
      }
      else
      {
        response_msg.slave_id = rxd_data[RESPONSE_SLAVE_ID];
        response_msg.func_code = rxd_data[RESPONSE_FUNCTION_CODE];
        for(int i = 0; i < query_data.read_num; i++)
        {
          response_msg.data[i] = pObj->convertResponse(rxd_data.data(), i);
        }
        pMesObj->setResponse(response_msg);
      }
    }
    catch(int error)
    {
      dispErrorMessage(error);
      setState(STATE_MES, STATE_MES_ERROR);
      return;
    }
  }
  setState(STATE_MES, STATE_MES_NONE);
}

/*---------------------------------------------------------------------------*/
/** 書き込み

@task
@param[in]	なし
@return		  なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::transWrite(void)
{
  int ret = 0;
  int len;
  std::vector<std::vector<char> > txd_data;
  txd_data.resize(MAX_QUERY_NUM);
  std::array<char, 255> rxd_data{0};

  ISetResponse *pMesObj = ros_mes_;
  IConvertQueryAndResponse *pObj = modbus_obj_[query_data.slave_id];
  pObj->setWrite(query_data.slave_id, query_data.write_addr, query_data.write_num, query_data.data, txd_data);

  if(0 == query_data.slave_id)
  {
    len = transComm(socket_fd_, txd_data[0], rxd_data.data(), 0);
  }
  else
  {
    len = transComm(socket_fd_, txd_data[0], rxd_data.data(), 8);
    try
    {
      chkResponseLength(rxd_data.data(), len);
      response_msg.slave_id = rxd_data[RESPONSE_SLAVE_ID];
	  response_msg.func_code = rxd_data[RESPONSE_FUNCTION_CODE];
      for(int i = 0; i < len; i++)
      {
        response_msg.data[i] = rxd_data[i];
      }
      pMesObj->setResponse(response_msg);
      setState(STATE_MES, STATE_MES_NONE);
    }
    catch(int error)
    {
      dispErrorMessage(error);
      setState(STATE_MES, STATE_MES_ERROR);
    }
  }
}

/*---------------------------------------------------------------------------*/
/** 購読したメッセージを保存(インターフェイス)

@task
@param[in]	購読メッセージ
@return		　なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::setData(const om_modbus_master::om_query msg)
{
  query_data.slave_id = msg.slave_id;
  query_data.func_code = msg.func_code;
  query_data.write_addr = msg.write_addr;
  query_data.read_addr = msg.read_addr;
  query_data.write_num = msg.write_num;
  query_data.read_num = msg.read_num;

  for(int i = 0; i < MAX_DATA_NUM; i++)
  {
    query_data.data[i] = msg.data[i];
  }

  isEnabled_ = true;
}

}
