#ifndef OM_BASE_H
#define OM_BASE_H

#include "om_modbus_master/om_ros_message.h"
#include "om_modbus_master/om_first_gen.h"
#include "om_modbus_master/ISetMessage.h"
using std::string;
namespace om_modbusRTU_node
{

class RosMessage;
class FirstGenModbusRTU;

class Base : public ISetMessage
{
protected:
  RosMessage *ros_mes_;
  FirstGenModbusRTU **modbus_obj_;

  om_modbus_master::om_response response_msg;
  om_modbus_master::om_state state_msg;

  bool isEnabled_;
  int socket_fd_;
  std::string topic_com_;
  int topic_baudrate_;
  fd_set fds_;
  struct timeval tv_;
  typedef struct{
    char slave_id = 0;
    char func_code = 0;
    int write_addr = 0;
    int read_addr = 0;
    short write_num = 0;
    short read_num = 0;
    int data[32] = {0};
  }QUERY_DATA_T;
  QUERY_DATA_T query_data;

  typedef struct{
    char slave_id = 0;
    int data[32] = {0};
  }RESPONSE_DATA_T;
  RESPONSE_DATA_T response_data;


  const static long TIMEOUT = 500*1000*1000;
  const static int EXCEPTION_RESPONSE_NUM = 5;
  const static int FUNCTION_CODE_READ = 0;
  const static int FUNCTION_CODE_WRITE = 1;
  const static int FUNCTION_CODE_READ_WRITE = 2;
  const static int MAX_DATA_NUM = 32;
  const static int MAX_QUERY_NUM = 10;

  const static int EXCEPTION_RESPONSE_ERROR = -1;
  const static int NO_RESPONSE = -2;
  const static int WRITE_ERROR = -3;
  const static int READ_ERROR = -4;
  const static int TIMEOUT_ERROR = -5;
  const static int SELECT_ERROR = -6;
  

  const static int STATE_MES = 0;
  const static int STATE_DRIVER = 1;
  const static int STATE_ERROR = 2;
  
  const static int STATE_MES_NONE = 0;
  const static int STATE_MES_REACH = 1;
  const static int STATE_MES_ERROR = 2;

  const static int STATE_DRIVER_NONE = 0;
  const static int STATE_DRIVER_COMM = 1;

  const static int STATE_ERROR_NONE = 0;
  const static int STATE_ERROR_NORESPONSE = 1;
  const static int STATE_ERROR_EXCEPTION_RESPONSE = 2;

  const static int RESPONSE_SLAVE_ID = 0;
  const static int RESPONSE_FUNCTION_CODE = 1;

  void chkResponseLength(char *pFrm, int len);
  void closeComm(void);
  int commRead(int fd, char *rdData, int rdLen);
  int commWrite(int fd, std::vector<char>& wrData);
  void dispErrorMessage(int error);
  void exeComm(void);
  int getBaudRate(int);
  long getCurrentTimeLinux();
  bool openComm(int, std::string);
  void setState(int type, int val);
  void startThread(void);
  int transComm(int fd, std::vector<char>& pCmd, char *pRes, int resLen);
  int transCommEx(int fd, std::vector<char>& pCmd, char *pRes, int resLen, long delayNs[2]);
  long transDelay(long delayNs);
  void transRead(void);  
  void transReadAndWrite(void);
  void transWrite(void);

public:
  Base();
  ~Base();
  void init(RosMessage *ros_mes_obj, FirstGenModbusRTU *modbus_obj[]);
  void setData(const om_modbus_master::om_query mes) override;
};

}

#endif
