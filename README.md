# move_oriental_motor
オリエンタルモータRK2シリーズ, AZシリーズ(位置決め機能内蔵方式)をrosで制御するプログラム

## 起動方法
### アクセス権限の設定(rk2シリーズ, AZシリーズ)
sudo chmod 777 /dev/ttyUSB-RK2 <br>
sudo chmod 777 /dev/ttyUSB-AZ <br>

### ノードの起動(rk2シリーズ, AZシリーズ)
roslaunch om_modbus_master om_modbusRTU.launch com:="/dev/ttyUSB-RK2" topicID:=1 baudrate:=115200 updateRate:=1000 firstGen:="1,2,3,4" <br>
roslaunch om_modbus_master om_modbusRTU2.launch com:="/dev/ttyUSB-AZ" topicID:=1 baudrate:=115200 updateRate:=1000 firstGen:="5,6,7" <br>

## 主なトピック名
### 検査支持部制御関係
* 4種類のRK2シリーズモータの位置制御(std_msgs::Int32MultiArray型) <br>
/robot_control/motor_position <br>
### プローブ走査機構制御関係
* xy直動モータの位置制御(geometry_msgs::Twist型) <br>
/probe_control/probe_control <br>
* エンコーダーによるxy直動モータの現在位置(std_msgs::Float32MultiArray型) <br>
/probe_control/current_pos <br>


## パッケージ詳細
### on_modbus_master
オリエンタルモータRK2シリーズ, AZシリーズ(位置決め機能内蔵方式)をmodbus通信RTU(Remote Terminal Unit)モードで制御するrosパッケージ <br>
参考URL: https://qiita.com/kazuharayama/items/4fda1ad55b8efa17e2e6 <br>
参考URL: https://www.orientalmotor.co.jp/tech/connect/ <br>

### move_oriental_motor
ロボット制御プログラムからの指令をrosのトピック通信によってsubscribeして処理し、om_modbus_masterにpublishするrosパッケージ <br>
RK2シリーズ(位置制御) <br>
AZシリーズ(位置制御, 速度制御, 位置・速度制御) <br>

