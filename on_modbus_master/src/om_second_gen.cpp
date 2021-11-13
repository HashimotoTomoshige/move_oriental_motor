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

/** @file	om_second_gen.cpp
@brief Modbus RTUのクエリを生成する(第2世代)
@details
@attention
@note

履歴 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
@version	Ver.1.00 Mar.11.2019 T.Takahashi
			 - 新規作成
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
#include "om_modbus_master/om_second_gen.h"

namespace om_modbusRTU_node
{

SecondGenModbusRTU::SecondGenModbusRTU()
{
}

SecondGenModbusRTU::~SecondGenModbusRTU()
{
}

/*---------------------------------------------------------------------------*/
/** アドレスチェック

@task
@param[in]	レジスタアドレス
@return		  false:範囲外　true:範囲内
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool SecondGenModbusRTU::chkAddress(int addr)
{
  return chkRange(addr, MIN_ADDRESS, MAX_ADDRESS);
}

/*---------------------------------------------------------------------------*/
/** 読み込み、書き込み数チェック

@task
@param[in]	データ数
@return		  false:範囲外　true:範囲内
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool SecondGenModbusRTU::chkDataNum(int num)
{
  return chkRange(num, MIN_DATA_NUM, MAX_DATA_NUM);
}


/*---------------------------------------------------------------------------*/
/** 最大アドレスの取得(インターフェイス)

@task
@param[in]  なし
@return		  最大アドレス
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int SecondGenModbusRTU::getMaxAddress(void)
{
  return MAX_ADDRESS;
}

/*---------------------------------------------------------------------------*/
/** 最大データ数の取得(インターフェイス)

@task
@param[in]  なし
@return		  最大データ数
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int SecondGenModbusRTU::getMaxDataNum(void)
{
  return MAX_DATA_NUM;
}

/*---------------------------------------------------------------------------*/
/** 書き込み、読み込みデータ設定(0x17)

@task	      0x17のクエリを設定
@param[in]	int id スレーブID
@param[in]	int read_addr	読み出し起点となるレジスタアドレス
@param[in]	int write_addr	書き込み起点となるレジスタアドレス
@param[in]	int read_num	読み出すレジスタの数
@param[in]	int write_num	書き込むレジスタの数
@param[int]	int *pVal	書き込みデータ
@param[out]	vector<vector<char> >& pOut	送信データ
@return		int クエリ数
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int SecondGenModbusRTU::setQuery17h(int id, int read_addr, int write_addr, int read_num, int write_num, int *pVal, std::vector<std::vector<char> >& pOut)
{
  int ret;
  int val;
  int query_num = 0;

  pOut[query_num].push_back(id);
  pOut[query_num].push_back(FUNCTION_CODE_READ_WRITE);
  pOut[query_num].push_back((char)(0xff & (read_addr   >> 8)));
  pOut[query_num].push_back((char)(0xff & (read_addr   >> 0)));
  pOut[query_num].push_back((char)(0xff & (read_num*2  >> 8)));
  pOut[query_num].push_back((char)(0xff & (read_num*2  >> 0)));
  pOut[query_num].push_back((char)(0xff & (write_addr  >> 8)));
  pOut[query_num].push_back((char)(0xff & (write_addr  >> 0)));
  pOut[query_num].push_back((char)(0xff & (write_num*2 >> 8)));
  pOut[query_num].push_back((char)(0xff & (write_num*2 >> 0)));
  pOut[query_num].push_back(write_num * 4);

  for(int i = 0; i < write_num; i++)
  {
    val = *pVal;
    set4Byte(val, pOut[query_num]);
    pVal++;
  }

  ret = chkFrm(INIT_VALUE, (std::vector<unsigned char>&)pOut[query_num]);
  pOut[query_num].push_back((char)ret);
  pOut[query_num].push_back(ret >> 8);
  return ++query_num;
}


/*---------------------------------------------------------------------------*/
/** 書き込み、読み込みデータ設定(インターフェイス)

@task
@param[in]	int id スレーブID
@param[in]	int read_addr	読み出し起点となるレジスタアドレス
@param[in]	int write_addr	書き込み起点となるレジスタアドレス
@param[in]	int read_num	読み出すレジスタの数
@param[in]	int write_num	書き込むレジスタの数
@param[int]	int *pVal	書き込みデータ
@param[out]	vector<vector<char> >& pOut	送信データ
@param[out]	int *resLen 受信数
@return		int クエリ数
@details
@attention
@note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int SecondGenModbusRTU::setReadAndWrite(int id, int read_addr, int write_addr, int read_num, int write_num, int *pVal, std::vector<std::vector<char> >& pOut, int *resLen)
{
  resLen[0] = 5 + read_num * 4;
  return setQuery17h(id, read_addr, write_addr, read_num, write_num, pVal, pOut);
}

}
