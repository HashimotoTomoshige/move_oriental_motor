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

/** @file	om_ros_message.cpp
@brief 上位と配信、購読を行う
@details
@attention
@note

履歴 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
@version	Ver.1.00 Mar.11.2019 T.Takahashi
			 - 新規作成
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
#include "om_modbus_master/om_ros_message.h"

using std::stringstream;
using std::string;
using std::cout;
using std::endl;

namespace om_modbusRTU_node
{

RosMessage::RosMessage()
{
  isCommEnabled_ = false;
}

RosMessage::~RosMessage()
{
  timer.stop();
}

/*---------------------------------------------------------------------------*/
/** アドレスの範囲確認

@task
@param[in]		int addr
@param[in]		int id
@return			  なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::chkAddress(int addr, int id)
{
  ICheckData *pObj = modbus_obj_[id];
  if(false == pObj->chkAddress(addr))
  {
    throw ADDRESS_ERROR;
  }
}

/*---------------------------------------------------------------------------*/
/** 書き込み、読み込み数の範囲確認

@task
@param[in]		int num
@param[in]		int id
@return			  なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::chkDataNum(int num, int id)
{
  ICheckData *pObj = modbus_obj_[id];
  if(false == pObj->chkDataNum(num))
  {
    throw DATA_ERROR;
  }
}


/*---------------------------------------------------------------------------*/
/** 数値の範囲確認

@task
@param[in]		入力値
@param[in]		最小値
@param[in]		最大値
@return			  int 0:範囲内 1:範囲外
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool RosMessage::chkRange(int val, int min, int max)
{
  bool ret = true;
  if(val > max || val < min)
  {
    ret = false;
  }
  return ret;
}


/*---------------------------------------------------------------------------*/
/** 数値の範囲確認

@task
@param[in]		om_modbus_master::om_query msg
@return			  なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::chkRangeOfData(const om_modbus_master::om_query msg)
{
  switch (msg.func_code)
  {
    case FUNCTION_CODE_READ:
      chkReadSlaveID(msg.slave_id);
      chkAddress(msg.read_addr, msg.slave_id);
      chkDataNum(msg.read_num, msg.slave_id);
      break;
    case FUNCTION_CODE_WRITE:
      chkWriteSlaveID(msg.slave_id);
      chkAddress(msg.write_addr, msg.slave_id);
      chkDataNum(msg.write_num, msg.slave_id);
      break;
    case FUNCTION_CODE_READ_WRITE:
      chkReadSlaveID(msg.slave_id);
      chkAddress(msg.read_addr, msg.slave_id);
      chkAddress(msg.write_addr, msg.slave_id);
      chkDataNum(msg.read_num, msg.slave_id);
      chkDataNum(msg.write_num, msg.slave_id);
      break;
    default:
      throw FUNCTION_CODE_ERROR;
      break;
  }
}

/*---------------------------------------------------------------------------*/
/** 読み込み時のスレーブIDの範囲確認

@task
@param[in]		int スレーブid
@return			  なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::chkReadSlaveID(int id)
{
  if(false == chkRange(id, MIN_SLAVE_ID + 1, MAX_SLAVE_ID))
  {
    throw SLAVE_ID_ERROR;
  }
}

/*---------------------------------------------------------------------------*/
/** 書き込み時のスレーブIDの範囲確認

@task
@param[in]		int スレーブid
@return			  なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::chkWriteSlaveID(int id)
{
  if(false == chkRange(id, MIN_SLAVE_ID, MAX_SLAVE_ID))
  {
    throw SLAVE_ID_ERROR;
  }
}

/*---------------------------------------------------------------------------*/
/** エラーメッセージ表示

@task
@param[in]		int error
@return			  なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::dispErrorMessage(int error)
{
  switch (error)
  {
  case SLAVE_ID_ERROR:
    ROS_ERROR("Error: Invalid slave ID");
    break;
  case FUNCTION_CODE_ERROR:
    ROS_ERROR("Error: Invalid function code");
    break;
  case ADDRESS_ERROR:
    ROS_ERROR("Error: Invalid address");
    break;
  case DATA_ERROR:
    ROS_ERROR("Error: Invalid data num");
    break;
  default:
    break;
  }
}

/*---------------------------------------------------------------------------*/
/** 初期化

@task
@param[in]	Base *pObj
@param[in]	FirstGenModbusRTU *modbus_obj[]
@return		  なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::init(Base *pObj, FirstGenModbusRTU *modbus_obj[])
{
  string topic;
  string update_rate;

  base_obj_ = pObj;
  modbus_obj_ = &modbus_obj[0];

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  string topic_name;
  om_modbus_master::om_state comm_msg;

  private_nh.getParam("init_topicID", topic);
  topic_id_ = stoi(topic);
  if(false == chkRange(topic_id_, MIN_TOPIC_ID, MAX_TOPIC_ID))
  {
    ROS_ERROR("Error: Specified topic ID out of range");
    throw;
  }

  private_nh.getParam("init_update_rate", update_rate);
  topic_update_rate_ = stoi(update_rate);
  if(false == chkRange(topic_update_rate_, MIN_UPDATE_RATE, MAX_UPDATE_RATE))
  {
    ROS_ERROR("Error: Specified updateRate out of range");
    throw;
  }

  topic_name = makeTopicName(topic_id_, "om_query");
  cout << "" << endl;
  cout << "Modbus RTU Node PARAMETERS" << endl;
  cout << " * /Subscriber: " << topic_name << endl;
  query_sub = nh.subscribe(topic_name, BUF_SIZE, &RosMessage::queryCallback, this);

  topic_name = makeTopicName(topic_id_, "om_response");
  cout << " * /Publisher: " << topic_name << endl;
  response_pub = nh.advertise<om_modbus_master::om_response>(topic_name, BUF_SIZE);
  topic_name = makeTopicName(topic_id_, "om_state");
  cout << " * /Publisher: " << topic_name << endl;
  state_pub = nh.advertise<om_modbus_master::om_state>(topic_name, BUF_SIZE);

  cout << " * /update_rate: " << topic_update_rate_ << endl;
  cout << "" << endl;

  if(0 != topic_update_rate_)
  {
    timer = nh.createTimer(ros::Duration(1.0 / (double)topic_update_rate_), &RosMessage::timerCallback, this);
  }
}

/*---------------------------------------------------------------------------*/
/** トピック名の作成

@task
@param[in]	int num		数値
@param[in]	string name	文字列
@return		　string 文字列
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
string RosMessage::makeTopicName(int num, string name)
{
  string ret = name;
  stringstream ss;
  ss << num;
  ret += ss.str();
  return ret;
}

/*---------------------------------------------------------------------------*/
/** Subscribeする対象のトピックが更新されたら呼び出されるコールバック関数

@task
@param[in]	購読するメッセージ
@return		なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::queryCallback(const om_modbus_master::om_query msg)
{
  if(false == isCommEnabled_)
  {
    isCommEnabled_ = true;
    state_msg.state_driver = STATE_DRIVER_COMM;
    state_msg.state_mes = STATE_MES_REACH;
    state_msg.state_error = STATE_ERROR_NONE;
    ISetMessage *pObj = base_obj_;

    try
    {
      chkRangeOfData(msg);
      pObj->setData(msg);
    }
    catch(int error)
    {
      dispErrorMessage(error);
      isCommEnabled_ = false;
      state_msg.state_driver = STATE_DRIVER_NONE;
      state_msg.state_mes = STATE_MES_ERROR;
    }
  }
  else if(true == isCommEnabled_)
  {
    ROS_INFO("Driver is busy");
  }
}

/*---------------------------------------------------------------------------*/
/** 定期周期で呼び出されるコールバック関数

@task
@param[in]	タイマーイベント(使用しない)
@return		  なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::timerCallback(const ros::TimerEvent&)
{
  state_pub.publish(state_msg);
}

/*---------------------------------------------------------------------------*/
/** インターフェイス

@task
@param[in]	上位に配信するレスポンスデータ
@return		  なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::setResponse(const om_modbus_master::om_response res)
{
  response_pub.publish(res);
}

/*---------------------------------------------------------------------------*/
/** インターフェイス

@task
@param[in]	bool val
@return		  なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::setCommEnabled(bool val)
{
  isCommEnabled_ = val;
}

/*---------------------------------------------------------------------------*/
/** インターフェイス

@task
@param[in]	上位に配信するステータスデータ
@return		　なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::setState(const om_modbus_master::om_state state)
{
  state_msg = state;
}

}
