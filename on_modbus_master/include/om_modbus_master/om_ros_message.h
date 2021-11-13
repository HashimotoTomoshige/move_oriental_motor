#ifndef OM_ROS_MESSAGE_H
#define OM_ROS_MESSAGE_H

#include "ros/ros.h"
#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_state.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_base.h"
#include "om_modbus_master/ISetResponse.h"

using std::string;

namespace om_modbusRTU_node
{
class Base;
class FirstGenModbusRTU;

class RosMessage : public ISetResponse
{
private:
  Base *base_obj_;
  FirstGenModbusRTU **modbus_obj_;
  ros::Publisher response_pub;
  ros::Publisher state_pub;
  ros::Subscriber query_sub;
  om_modbus_master::om_response response_msg;
  om_modbus_master::om_state state_msg;
  ros::Timer timer;

  bool isCommEnabled_;
  int topic_id_;
  int topic_update_rate_;

  const static int BUF_SIZE = 1;
  const static int MIN_UPDATE_RATE = 0;
  const static int MAX_UPDATE_RATE = 1000;
  const static int MAX_TOPIC_ID = 15;
  const static int MIN_TOPIC_ID = 0;
  const static int MIN_SLAVE_ID = 0;
  const static int MAX_SLAVE_ID = 31;
  const static int FUNCTION_CODE_READ = 0;
  const static int FUNCTION_CODE_WRITE = 1;
  const static int FUNCTION_CODE_READ_WRITE = 2;
  
  const static int STATE_DRIVER_NONE = 0;
  const static int STATE_DRIVER_COMM = 1;
  const static int STATE_MES_REACH = 1;
  const static int STATE_MES_ERROR = 2;
  const static int STATE_ERROR_NONE = 0;
  
  const static int SLAVE_ID_ERROR = -1;
  const static int FUNCTION_CODE_ERROR = -2;
  const static int ADDRESS_ERROR = -3;
  const static int DATA_ERROR = -4;
  
  void chkAddress(int addr, int id);
  void chkDataNum(int num, int id);
  bool chkRange(int val, int min, int max);
  void chkRangeOfData(const om_modbus_master::om_query msg);
  void chkReadSlaveID(int id);
  void chkWriteSlaveID(int id);
  void dispErrorMessage(int error);
  string makeTopicName(int num, string name);
  void queryCallback(const om_modbus_master::om_query msg);
  void timerCallback(const ros::TimerEvent&);

public:
  RosMessage();
  ~RosMessage();
  void init(Base *pObj, FirstGenModbusRTU *modbus_obj[]);
  void setResponse(const om_modbus_master::om_response res) override;
  void setState(const om_modbus_master::om_state state) override;
  void setCommEnabled(bool val) override;

};
}
#endif
