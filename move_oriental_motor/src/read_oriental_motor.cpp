#include "ros/ros.h"
#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <string>

/* グローバル変数 */
int gState_driver = 0; /* 通信可能フラグ変数(0:通信可能,1:通信中) */
int gMotor_pos = 0;

/*---------------------------------------------------------------------------*/
/** ステータスコールバック

@details 購読したステータスデータをグローバル変数に反映する
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

void wait(void);

void resCallback(const om_modbus_master::om_response res)
{
    gMotor_pos = res.data[0];
}

void stateCallback(const om_modbus_master::om_state msg)
{
	gState_driver = msg.state_driver;
}

void wait(void)
{
	ros::Duration(0.03).sleep();
	ros::spinOnce();

	/* ドライバの通信が終了するまでループ */
	while(gState_driver == 1)
	{
		ros::spinOnce();
	}
}

/*---------------------------------------------------------------------------*/
/** メイン関数

@details 処理内容1:運転データNo.0の位置を書き込み

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "read_oriental_motor");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<om_modbus_master::om_query>("/om_modbusRTU_1/om_query1",1); /* OMノードに送信するための定義 */
	ros::Subscriber sub = n.subscribe("/om_modbusRTU_1/om_state1",1, stateCallback);
    ros::Subscriber res_sub = n.subscribe("/om_modbusRTU_1/om_response1", 1, resCallback);

	ros::NodeHandle pnh("~");
  	std::string mode;
  	pnh.getParam("mode", mode);
	// std::cout << mode;

    ros::Duration(1.0).sleep();

    om_modbus_master::om_query msg;

    msg.slave_id = 0x02;
    msg.func_code = 0;
    msg.read_addr = 256;
    msg.read_num = 1;
    pub.publish(msg);
    wait();

    std::cout << gMotor_pos;

    ros::spinOnce();

	return 0;
}