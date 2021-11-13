#ifndef OM_NODE_H
#define OM_NODE_H


using std::string;

namespace om_modbusRTU_node
{
class FirstGenModbusRTU;
class RosMessage;
class Base;

void init(std::vector<int>& first, std::vector<int>& second, Base *base_obj, FirstGenModbusRTU *modbus_obj[], RosMessage *rosmes_obj);
void finalize(std::vector<int>& first, std::vector<int>& second, Base *base_obj, FirstGenModbusRTU *modbus_obj[],  RosMessage *rosmes_obj);
void bgMain(void);
void deleteSpace(string &str);
std::vector<int> split(string& src, char delim);
void chkComma(string &str);

const static int MAX_SLAVE_NUM = 8;
const static int MIN_SLAVE_ID = 1;
const static int MAX_SLAVE_ID = 31;
}
#endif
