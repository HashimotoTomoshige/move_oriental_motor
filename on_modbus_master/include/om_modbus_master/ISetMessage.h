#ifndef ISET_MESSAGE_H
#define ISET_MESSAGE_H

namespace om_modbusRTU_node
{
struct QUERY_DATA_T;

class ISetMessage
{
public:
  virtual void setData(const om_modbus_master::om_query mes)=0;
  virtual ~ISetMessage(){};
};

}
#endif
