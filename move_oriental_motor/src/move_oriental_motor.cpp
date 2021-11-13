#include "ros/ros.h"
#include "om_modbus_master/om_query.h"
#include "om_modbus_master/om_response.h"
#include "om_modbus_master/om_state.h"
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>

/* グローバル変数 */
int gState_driver = 0; /* 通信可能フラグ変数(0:通信可能,1:通信中) */
om_modbus_master::om_query msg; /* ノードで定義されたメッセージを使用 */
int incremental_motor_pos[4] = {0, 0, 0, 0}; //回転型モータRk2の相対位置
int absolute_motor_pos[4] = {0, 12313, 0, 0}; //q2の初期角度は30°なので、absolute_motor_pos[1]の初期値は12313

int incremental_linear_motor_pos[3] = {0, 0, 0}; //直動型モータAzの相対位置
int absolute_linear_motor_pos[3] = {0, 0, 0}; //直動型モータAzの相対位置
int linear_motor_velocity[2] = {0, 0}; //直動型モータAzの速度(正だと右＆下)
int linear_motor_goal_position[2] = {0, 0}; //速度制御の際の直動モータAZの速度

std_msgs::Int32MultiArray az_pos;

bool write_flag = false;

bool linear_write_flag = false;

bool velocity_mode = false;

int gMotor_pos = 0;

bool initialization_flag = false;

bool finish_flag[] = {0, 0};

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

/*回転型モータのトピックを受け取った時のコールバック関数*/
void motorPositionCallback(const std_msgs::Int32MultiArray& pos)
{
	int data_num = pos.data.size(); //pos.dataの配列のサイズを取得
	ROS_INFO("I Received %i data.", data_num);

	for(int i=0; i<data_num; i++)
	{
		incremental_motor_pos[i] = pos.data[i] - absolute_motor_pos[i];//5/29追記エラーハンドリングの必要あり
		absolute_motor_pos[i] = pos.data[i];
		write_flag = true;
		ROS_INFO("Receive pos_data[%i]", i);
		ROS_INFO("absolute_motor_pos[%i] is %i", i, absolute_motor_pos[i]);
	}
}

/*直動型モータのトピックを受け取った時のコールバック関数*/
void linearMotorPositionCallback(const geometry_msgs::Twist& twist)
{	
	if(true)
	{
		const int complementary_value = 100; //変位からモーター角度への補完値
		double linear_y = twist.linear.y * complementary_value;
		double linear_z = twist.linear.z * complementary_value;
		if(initialization_flag == true)
		{
			incremental_linear_motor_pos[0] = linear_y;
			incremental_linear_motor_pos[1] = linear_z;
			absolute_linear_motor_pos[0] = 0;
			absolute_linear_motor_pos[1] = 0;
			std::cout << "**************************initialization_flag is " << initialization_flag << std::endl;
			initialization_flag = false;
		}
		else
		{
			incremental_linear_motor_pos[0] = linear_y - absolute_linear_motor_pos[0];
			incremental_linear_motor_pos[1] = linear_z - absolute_linear_motor_pos[1];
			absolute_linear_motor_pos[0] = linear_y;
			absolute_linear_motor_pos[1] = linear_z;
			std::cout << "initialization_flag is " << initialization_flag << std::endl;
			// for(int i=0; i<2; i++)
			// {
			// 	if(incremental_linear_motor_pos[i] > 0)
			// 	{
			// 		linear_motor_velocity[i] = -300;
			// 		std::cout << "set speed + " << std::endl;
			// 	}
			// 	else if((incremental_linear_motor_pos[i] < 0))
			// 	{
			// 		linear_motor_velocity[i] = -500;
			// 		std::cout << "set speed - " << std::endl;
			// 	}
			// }
		}
	}
	linear_write_flag = true;
	velocity_mode = false;
	ROS_INFO("absolute_linear_motor_pos[0] is %i", absolute_linear_motor_pos[0]);
	ROS_INFO("absolute_linear_motor_pos[1] is %i", absolute_linear_motor_pos[1]);

	ROS_INFO("incremental_linear_motor_pos[0] is %i", incremental_linear_motor_pos[0]);
	ROS_INFO("incremental_linear_motor_pos[1] is %i", incremental_linear_motor_pos[1]);
}

/*直動モーターの位置を初期化するコールバック関数*/
void initializationCallback(const std_msgs::Bool& msg)
{
	initialization_flag = msg.data;
}

/*軌道制御の際のコールバック関数*/
void poseAndVelocityCallback(const std_msgs::Float32MultiArray& msg)
{
	std::cout << "poseAndVelocityCallback" << std::endl;
	int complementary_value = 100; //変位からモーター角度への補完値
	int linear_z = msg.data[0] * complementary_value;
	int linear_y = msg.data[1] * complementary_value;
	incremental_linear_motor_pos[0] = linear_y - absolute_linear_motor_pos[0];
	incremental_linear_motor_pos[1] = linear_z - absolute_linear_motor_pos[1];
	absolute_linear_motor_pos[0] = linear_y;
	absolute_linear_motor_pos[1] = linear_z;

	linear_motor_velocity[0] = msg.data[3] * 100;
	linear_motor_velocity[1] = msg.data[2] * 100;

	linear_motor_goal_position[0] = az_pos.data[0] + incremental_linear_motor_pos[0];
	linear_motor_goal_position[1] = az_pos.data[1] + incremental_linear_motor_pos[1];

	ROS_INFO("absolute_linear_motor_pos[0] is %i", absolute_linear_motor_pos[0]);
	ROS_INFO("absolute_linear_motor_pos[1] is %i", absolute_linear_motor_pos[1]);

	ROS_INFO("incremental_linear_motor_pos[0] is %i", incremental_linear_motor_pos[0]);
	ROS_INFO("incremental_linear_motor_pos[1] is %i", incremental_linear_motor_pos[1]);

	std::cout << "az_pos.data[0]: " << az_pos.data[0] << "az_pos.data[1]: " << az_pos.data[1] << std::endl;
	linear_write_flag = true;
	velocity_mode = true;

}

void linearVelocityCallback(const geometry_msgs::Point& msg)
{
	if(abs(linear_motor_velocity[1] - msg.z * 100) > 100)
	{
		std::cout << "[joy] z velocity: " << msg.z *100 << std::endl;
		linear_motor_velocity[1] = msg.z *100;
		linear_motor_goal_position[1] = 1000000; //目標位置はなし, 次の指令が来るまでずっとその速度
		linear_write_flag = true;
	}

	if(abs(linear_motor_velocity[0] - msg.y * 100) > 100)
	{
		std::cout << "[joy] y velocity" << msg.y * 100 << std::endl;
		linear_motor_velocity[0] = msg.y * 100;
		linear_motor_goal_position[0] = 1000000; //目標位置はなし, 次の指令が来るまでずっとその速度
		linear_write_flag = true;
	}
	velocity_mode = true;
}



/*回転型モータをセットする関数*/
void setMotorRk2(int8_t var, ros::Publisher pub)
{
	msg.slave_id = var; /* 号機選択(Hex): 1~5号機 */
	msg.func_code = 1; /* ファンクションコード選択: 1(Write) */
	msg.write_addr = 1280; /* 先頭アドレス選択(Dec): 運転データNo.0のモード */
	msg.write_num = 1; /* 書き込みデータサイズ: 1 (32bit) */
	msg.data[0] = 0; /* o INC 1 ABS */
	pub.publish(msg); /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait(); /* 処理待ち */
}

/*回転型モータを動かす関数*/
void moveMotorRk2(int8_t var, int i, ros::Publisher pub, const int& pos)
{
	msg.slave_id = var; /* 号機選択(Hex): 1~5号機 */
	msg.func_code = 1; /* ファンクションコード選択: 1(Write) */
	msg.write_addr = 1024; /* 先頭アドレス選択(Dec): 運転データNo.0の位置 */
	msg.write_num = 1; /* 書き込みデータサイズ: 1 (32bit) */
	msg.data[0] = pos; /* 位置[step] */
	pub.publish(msg); /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait(); /* 処理待ち */

	msg.slave_id = var; /* 号機選択(Hex): 1~5号機 */
	msg.func_code = 1; /* ファンクションコード選択: 1(Write) */
	msg.write_addr = 124; /* 先頭アドレス選択(Dec): 運転データNo.0の位置 */
	msg.write_num = 1; /* 書き込みデータサイズ: 1 (32bit) */
	msg.data[0] = 8; /* 位置[step] */
	pub.publish(msg); /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();

	msg.slave_id = var; /* 号機選択(Hex): 1~5号機 */
	msg.func_code = 1; /* ファンクションコード選択: 1(Write) */
	msg.write_addr = 124; /* 先頭アドレス選択(Dec): 運転データNo.0の位置 */
	msg.write_num = 1; /* 書き込みデータサイズ: 1 (32bit) */
	msg.data[0] = 0; /* 位置[step] */
	pub.publish(msg); /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();
}

/*直動型モータをセットする関数*/
void setMotorAz(int8_t var, int mode, ros::Publisher pub)
{
	msg.slave_id = var; /* 号機選択(Hex): 1号機*/
	msg.func_code = 1; /* ファンクションコード選択: 1(Write)*/
	msg.write_addr = 6144; /* 先頭アドレス選択(Dec): 運転データNo.0の方式(1800h)*/
	msg.write_num = 1; /* 書き込みデータサイズ: 6 (6x32bit)*/
	msg.data[0] = mode; /* 方式: 2:相対位置決め(指令位置基準)*/
	pub.publish(msg); /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信*/
	wait(); /* 処理待ち*/
}

void setMotorAzVelocity(int8_t var, int add_num, int vel, ros::Publisher pub)
{
	msg.slave_id = var; /* 号機選択(Hex): 1号機*/
	msg.func_code = 1; /* ファンクションコード選択: 1(Write)*/
	msg.write_addr = 6148 + 64 * add_num; /* 先頭アドレス選択(Dec): 運転データNo.0の方式(1800h)*/
	msg.write_num = 1; /* 書き込みデータサイズ: 6 (6x32bit)*/
	msg.data[0] = vel; /* 方式: 2:相対位置決め(指令位置基準)*/
	pub.publish(msg); /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信*/
	wait(); /* 処理待ち*/
}

void selectMotorAzNum(int8_t var, int add_num, ros::Publisher pub)
{
	msg.slave_id = var; /* 号機選択(Hex): 1号機*/
	msg.func_code = 1; /* ファンクションコード選択: 1(Write)*/
	msg.write_addr = 118; /* 先頭アドレス選択(Dec): 運転データNo.0の方式(1800h)*/
	msg.write_num = 1; /* 書き込みデータサイズ: 6 (6x32bit)*/
	msg.data[0] = add_num; /* 方式: 2:相対位置決め(指令位置基準)*/
	pub.publish(msg); /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信*/
	wait(); /* 処理待ち*/
}

/*直動型モータを動かす関数*/
void moveMotorAz(int8_t var, int i, ros::Publisher pub, const int& pos)
{
	msg.slave_id = var; /* 号機選択(Hex): 1号機*/
	msg.func_code = 1; /* ファンクションコード選択: 1(Write)*/
	msg.write_addr = 6146; /* 先頭アドレス選択(Dec): 運転データNo.0の方式(1800h)*/
	msg.write_num = 1; /* 書き込みデータサイズ: 6 (6x32bit)*/
	msg.data[0] = pos; /* 方式: 2:相対位置決め(指令位置基準)*/
	pub.publish(msg); /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信*/
	wait(); /* 処理待ち*/

	msg.slave_id = var; /* 号機選択(Hex): 1号機*/
	msg.func_code = 1; /* ファンクションコード選択: 1(Write)*/
	msg.write_addr = 124; /* 先頭アドレス選択(Dec): 運転データNo.0の位置*/
	msg.write_num = 1; /* 書き込みデータサイズ: 1 (32bit)*/
	msg.data[0] = 8; /* 位置[step]*/
	pub.publish(msg); /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信*/
	wait(); /* 処理待ち*/

	msg.slave_id = var; /* 号機選択(Hex): 1号機*/
	msg.func_code = 1; /* ファンクションコード選択: 1(Write)*/
	msg.write_addr = 124; /* 先頭アドレス選択(Dec): 運転データNo.0の位置*/
	msg.write_num = 1; /* 書き込みデータサイズ: 1 (32bit)*/
	msg.data[0] = 0; /* 位置[step]*/
	pub.publish(msg); /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信*/
	wait(); /* 処理待ち*/
	if(var = 0x07)
	{
		std::cout << "receive 0x07: " << pos << std::endl;
	}
}

void motorAzVelocityModeStart(int8_t var, ros::Publisher pub)
{
	msg.slave_id = var; /* 号機選択(Hex): 1号機*/
	msg.func_code = 1; /* ファンクションコード選択: 1(Write)*/
	msg.write_addr = 124; /* 先頭アドレス選択(Dec): 運転データNo.0の位置*/
	msg.write_num = 1; /* 書き込みデータサイズ: 1 (32bit)*/
	msg.data[0] = 16*16*16*4; /* 位置[step]*/
	pub.publish(msg); /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信*/
	wait(); /* 処理待ち*/
}

void motorAzVelocityModeEnd(int8_t var, ros::Publisher pub)
{
	msg.slave_id = var; /* 号機選択(Hex): 1号機*/
	msg.func_code = 1; /* ファンクションコード選択: 1(Write)*/
	msg.write_addr = 124; /* 先頭アドレス選択(Dec): 運転データNo.0の位置*/
	msg.write_num = 1; /* 書き込みデータサイズ: 1 (32bit)*/
	msg.data[0] = 0; /* 位置[step]*/
	pub.publish(msg); /* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信*/
	wait(); /* 処理待ち*/
}

void moveAzVelocityMode(int8_t var, const int &mode_num, int vel, ros::Publisher pub)
{
	if(mode_num == 0)
	{
		setMotorAzVelocity(var, 1, vel, pub);
		selectMotorAzNum(var, 1, pub);
	}
	if(mode_num == 1)
	{
		setMotorAzVelocity(var, 0, vel, pub);
		selectMotorAzNum(var, 0, pub);
	}
	
}


int readMotorRK2(int8_t var, ros::Publisher pub)
{
	om_modbus_master::om_query msg;

    msg.slave_id = var;
    msg.func_code = 0;
    msg.read_addr = 256;
    msg.read_num = 1;
    pub.publish(msg);
    wait();

	return gMotor_pos;
}

int readMotorAZ(int8_t var, ros::Publisher pub)
{
	om_modbus_master::om_query msg;

    msg.slave_id = var;
    msg.func_code = 0;
    msg.read_addr = 204;
    msg.read_num = 1;
    pub.publish(msg);
    wait();

	return gMotor_pos;
}

/*---------------------------------------------------------------------------*/
/** 処理待ちサービス関数

@details 規定時間後(30ms)、通信可能になるまでウェイトがかかるサービス
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
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
	ros::init(argc, argv, "move_oriental_motor");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<om_modbus_master::om_query>("om_query1",1); /* OMノードに送信するための定義 */
	ros::Publisher az_pos_pub = n.advertise<std_msgs::Int32MultiArray>("current_pos",1);

	ros::Subscriber sub = n.subscribe("om_state1",1, stateCallback);
	ros::Subscriber motoro_position_sub = n.subscribe("motor_position",1, motorPositionCallback);
	ros::Subscriber linear_motor_position_sub = n.subscribe("probe_control", 1, linearMotorPositionCallback);
	ros::Subscriber res_sub = n.subscribe("om_response1", 1, resCallback);
	ros::Subscriber initialize_position_sub = n.subscribe("initialization", 1, initializationCallback);
	ros::Subscriber zy_pose_and_velocity_sub = n.subscribe("linear_pose_and_velocity", 1, poseAndVelocityCallback);
	ros::Subscriber linear_velocity_sub = n.subscribe("linear_velocity", 1, linearVelocityCallback);

	ros::NodeHandle pnh("~");
  	std::string mode;
  	pnh.getParam("mode", mode);
	int az_mode_num = 0;
	// std::cout << mode;

	ros::Duration(1.0).sleep();

	if(mode == "RK2")
	{
		setMotorRk2(0x01, pub);
		setMotorRk2(0x02, pub);
		setMotorRk2(0x03, pub);
		setMotorRk2(0x04, pub);
		
	}
	if(mode == "AZ")
	{
		if(velocity_mode)
		{
		setMotorAz(0x05, 16, pub);
		setMotorAz(0x06, 16, pub);
		}
		else
		{
		setMotorAz(0x05, 2, pub);
		setMotorAz(0x06, 2, pub);
		setMotorAz(0x07, 2, pub);
		setMotorAzVelocity(0x05, 0, 850, pub); //正で右, value/100 mm/s
		setMotorAzVelocity(0x06, 0, 850, pub); //正で下
		setMotorAzVelocity(0x07, 0, 850, pub);
		}
	}

	ROS_INFO("Set");

	ros::Rate loop_rate(10);

	int velocity;

	while(ros::ok())
	{
		if(mode == "RK2")
		{
			if(write_flag)
			{
				ROS_INFO("Write RK2");
				moveMotorRk2(0x01, 0, pub, incremental_motor_pos[0]);
				moveMotorRk2(0x02, 1, pub, incremental_motor_pos[1]);
				moveMotorRk2(0x03, 2, pub, incremental_motor_pos[2]);
				moveMotorRk2(0x04, 3, pub, incremental_motor_pos[3]);
				write_flag = false;
			}
		}

		if(mode == "AZ")
		{
			az_pos.data.resize(2);
			az_pos.data[0] = readMotorAZ(0x05, pub);
			az_pos.data[1] = readMotorAZ(0x06, pub);
			// std::cout << "az_pos.data[0]: " << az_pos.data[0] << "az_pos.data[1]: " << az_pos.data[1] << std::endl;
			az_pos_pub.publish(az_pos);

			// std::cout << "Input velocity:" << std::endl;
			// std::cin >> velocity;
			// std::cout << "OK" << std::endl;
			if(linear_write_flag)
			{
				std::cout << "OK" << std::endl;
				ROS_INFO("Write AZ");
				// setMotorAzVelocity(0x05, 0, linear_motor_velocity[0], pub);
				// setMotorAzVelocity(0x06, 0, linear_motor_velocity[1], pub);
				if(velocity_mode == false)
				{
				// sleep(1);
				moveMotorAz(0x05, 0, pub, incremental_linear_motor_pos[0]); //関数の仕様上よくない。モータの回転量を引数に入れておくほうが汎用性がある
				moveMotorAz(0x06, 1, pub, incremental_linear_motor_pos[1]);
				// moveMotorAz(0x07, 1, pub, -3000);
				}

				if(velocity_mode == true)
				{
					// 一回だけ
					setMotorAzVelocity(0x05, 0, 0, pub);
					setMotorAzVelocity(0x06, 0, 0, pub);
					motorAzVelocityModeStart(0x05, pub);
					motorAzVelocityModeStart(0x06, pub);
					
					moveAzVelocityMode(0x05, az_mode_num, linear_motor_velocity[0], pub);
					moveAzVelocityMode(0x06, az_mode_num, linear_motor_velocity[1], pub);




					if(az_mode_num == 0)
					{
						az_mode_num = 1;
					}
					else if(az_mode_num == 1)
					{
						az_mode_num = 0;
					}
				}

				linear_write_flag = false;
				finish_flag[0] = 0;
				finish_flag[1] = 0;
			}

			if(write_flag)
			{
				moveMotorAz(0x07, 1, pub, incremental_motor_pos[2]);
				write_flag = false;
			}

			if(velocity_mode == true)
			{
				if(abs(linear_motor_goal_position[0] - az_pos.data[0]) < 100 && finish_flag[0] == 0)
				{
					moveAzVelocityMode(0x05, az_mode_num, 0, pub);
					finish_flag[0] = 1;
					std::cout << "y finish" << std::endl;// std::cout << "y: az_pos.data[0]: " << az_pos.data[0] << "az_pos.data[1]: " << az_pos.data[1] << std::endl;
					// motorAzVelocityModeEnd(0x05, pub);
				}
				if(abs(linear_motor_goal_position[1] - az_pos.data[1]) < 100 && finish_flag[1] == 0)
				{
					moveAzVelocityMode(0x06, az_mode_num, 0, pub);
					finish_flag[1] = 1;
					// motorAzVelocityModeEnd(0x06, pub);
					std::cout << "z finish" << std::endl;// std::cout << "z: az_pos.data[0]: " << az_pos.data[0] << "az_pos.data[1]: " << az_pos.data[1] << std::endl;
				}
			}
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	motorAzVelocityModeEnd(0x05, pub);
	motorAzVelocityModeEnd(0x06, pub);
	ros::spinOnce();

	return 0;
}
