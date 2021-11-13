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

/** @file	om_node.cpp
@brief オブジェクト生成
@details
@attention
@note

履歴 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
@version	Ver.1.00 Mar.11.2019 T.Takahashi
			 - 新規作成
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
#include "ros/ros.h"
#include "om_modbus_master/om_node.h"
#include "om_modbus_master/om_base.h"
#include "om_modbus_master/om_ros_message.h"
#include "om_modbus_master/om_first_gen.h"
#include "om_modbus_master/om_second_gen.h"
#include "om_modbus_master/om_broadcast.h"

using std::string;
using std::cout;
using std::endl;
namespace ns = om_modbusRTU_node;

namespace om_modbusRTU_node
{

/*---------------------------------------------------------------------------*/
/** 初期化関数

@task		    オブジェクトの生成
@param[in]  vector<int>& first
@param[in]  vector<int>& second
@param[in]  Base *base_obj
@param[in]  FirstGenModbusRTU *modbus_obj[]
@param[in]  RosMessage *rosmes_obj
@return     なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void init(std::vector<int>& first, std::vector<int>& second, Base *base_obj, FirstGenModbusRTU *modbus_obj[], RosMessage *rosmes_obj)
{
  if(0 == first.size() && 0 == second.size())
  {
    ROS_ERROR("Error: No slave ID specified");
    throw;
  }
  if(MAX_SLAVE_NUM < (first.size() + second.size()))
  {
    ROS_ERROR("Error: Maximum number of connectable devices exceeded");
    throw;
  }

  cout << "" << endl;
  cout << "Connectable devices" << endl;
  modbus_obj[0] = new ns::BroadcastModbusRTU();
  ICheckData *pObj_broadcast = modbus_obj[0];

  cout << " * /First Generation Number: " << first.size() << endl;
  for(int i = 0; i < first.size(); i++)
  {
    if(MIN_SLAVE_ID > first[i] || MAX_SLAVE_ID < first[i])
    {
      ROS_ERROR("Error: Specified slave ID out of range");
      throw;
    }
    cout << " * /First Generation(ID):" << first[i] << endl;
    modbus_obj[first[i]] = new ns::FirstGenModbusRTU();

    ICheckData *pObj = modbus_obj[first[i]];
    pObj_broadcast->setMaxAddress(pObj->getMaxAddress());
    pObj_broadcast->setMaxDataNum(pObj->getMaxDataNum());
  }

  cout << " * /Second Generation Number: " << second.size() << endl;
  for(int i = 0; i < second.size(); i++)
  {
    if(MIN_SLAVE_ID > second[i] || MAX_SLAVE_ID < second[i])
    {
      ROS_ERROR("Error: Specified slave ID out of range");
      throw;
    }
    cout << " * /Second Generation(ID):" << second[i] << endl;
    modbus_obj[second[i]] = new ns::SecondGenModbusRTU();

    ICheckData *pObj = modbus_obj[second[i]];
    pObj_broadcast->setMaxAddress(pObj->getMaxAddress());
    pObj_broadcast->setMaxDataNum(pObj->getMaxDataNum());
  }

  rosmes_obj->init(base_obj, &modbus_obj[0]);
  base_obj->init(rosmes_obj, &modbus_obj[0]);
}

/*---------------------------------------------------------------------------*/
/** 終了処理関数

@task        オブジェクトの削除
@param[in]   vector<int>& first
@param[in]   vector<int>& second
@param[in]   Base *base_obj
@param[in]   FirstGenModbusRTU *modbus_obj[]
@param[in]   RosMessage *rosmes_obj
@return      なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void finalize(std::vector<int>& first, std::vector<int>& second, Base *base_obj, FirstGenModbusRTU *modbus_obj[],  RosMessage *rosmes_obj)
{
  delete rosmes_obj;
  delete base_obj;
  delete modbus_obj[0];
  for(int i = 0; i < first.size(); i++)
  {
    delete modbus_obj[first[i]];
  }

  for(int i = 0; i < second.size(); i++)
  {
    delete modbus_obj[second[i]];
  }
}

/*---------------------------------------------------------------------------*/
/** bgMain関数

@task       bgループ
@param[in]  なし
@return　　　なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void bgMain()
{
  ros::spin();
}

/*---------------------------------------------------------------------------*/
/** 文字列→数値変換

@task 文字列を数値に変換する
@param[in]　　　string& src 変換文字列
@param[in]　　　char delim  区切り文字
@return　　　　　vector<int>
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
std::vector<int> split(string& src, char delim = ',')
{
  std::vector<int> vec;
  std::istringstream iss(src);
  string tmp;

  if("\0" == src)
  {
    return vec;
  }

  chkComma(src);

  while(getline(iss, tmp, delim))
  {
    vec.push_back(stoi(tmp));
  }
  return vec;
}

/*---------------------------------------------------------------------------*/
/** 重複チェック

@task
@param[in]　　　第１世代(配列)
@param[in]　　　第2世代(配列)
@return　　　　  void
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void chkDuplication(std::vector<int>& first, std::vector<int>& second)
{
  int size;
  std::vector<int> inter;

  std::sort(first.begin(), first.end());
  std::sort(second.begin(), second.end());

  size = first.size();
  for(int i = 0; i < (size-1); i++)
  {
    if(first[i] == first[i+1])
    {
      ROS_ERROR("Error: Duplicate firstGen");
      throw;
    }
  }

  size = second.size();
  for(int i = 0; i < (size-1); i++)
  {
    if(second[i] == second[i+1])
    {
      ROS_ERROR("Error: Duplicate secondGen");
      throw;
    }
  }

  std::set_intersection(first.begin(),first.end(), second.begin(),second.end(),std::back_inserter(inter));
  if(0 < inter.size())
  {
    ROS_ERROR("Error: Duplicate firstGen and secondGen");
    throw;
  }
}

/*---------------------------------------------------------------------------*/
/** 文字列からスペースを削除

@task
@param[in]　第１世代(文字列)
@param[in]　第2世代(文字列)
@return　　　なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void deleteSpace(string &first, string &second)
{
  first.erase(remove(first.begin(), first.end(), ' '), first.end());
  first.erase(remove(first.begin(), first.end(), '\t'), first.end());

  second.erase(remove(second.begin(), second.end(), ' '), second.end());
  second.erase(remove(second.begin(), second.end(), '\t'), second.end());
}

/*---------------------------------------------------------------------------*/
/** カンマ有無チェック

@task
@param[in]　チェックする文字列
@return　　　なし
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void chkComma(string &str)
{
  if(string::npos == str.find(','))
  {
    ROS_ERROR("Error: Comma does not exist");
    throw;
  }
}

}

/*---------------------------------------------------------------------------*/
/** main関数

@task		一番最初にここが実行される
@param[in]  int argc
@param[in]  char **argv
@return　　　int
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int main(int argc, char **argv)
{
  string first_str, second_str;
  std::vector<int> first_gen, second_gen;

  ros::init(argc, argv, "om");
  ros::NodeHandle private_nh("~");

  private_nh.getParam("first_gen", first_str);
  private_nh.getParam("second_gen", second_str);

  ns::deleteSpace(first_str, second_str);

  try
  {
    first_gen = ns::split(first_str);
    second_gen = ns::split(second_str);

    ns::chkDuplication(first_gen, second_gen);

    ns::RosMessage *ros_mes = new ns::RosMessage;
    ns::Base *base = new ns::Base;
    ns::FirstGenModbusRTU *modbus_rtu[ns::MAX_SLAVE_ID+1];

    ns::init(first_gen, second_gen, base, modbus_rtu, ros_mes);
    ns::bgMain();
    ns::finalize(first_gen, second_gen, base, modbus_rtu, ros_mes);
  }
  catch(int err)
  {
  }

  return 0;
}
