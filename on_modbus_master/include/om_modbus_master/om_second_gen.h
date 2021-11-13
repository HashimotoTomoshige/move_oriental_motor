#ifndef OM_SECOND_GEN_H
#define OM_SECOND_GEN_H

#include "om_modbus_master/om_base.h"

namespace om_modbusRTU_node
{

class SecondGenModbusRTU :  public FirstGenModbusRTU
{
private:
  
public:
  const static int MAX_DATA_NUM = 32;
  const static int MAX_ADDRESS = 0x57FF;
  SecondGenModbusRTU();
  ~SecondGenModbusRTU();
  bool chkAddress(int addr) override;
  bool chkDataNum(int num) override;
  int getMaxAddress(void) override;
  int getMaxDataNum(void) override;
  int setQuery17h(int id, int read_addr, int write_addr, int read_num, int write_num, int *pVal, std::vector<std::vector<char> >& pOut);
  int setReadAndWrite(int id, int read_addr, int write_addr, int read_num, int write_num, int *pVal, std::vector<std::vector<char> >& pOut, int *resLen) override;
};

}

#endif
