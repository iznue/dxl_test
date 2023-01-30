#include "ros/ros.h"                                // ros 기본 헤더 파일
#include "std_msgs/String.h"                        // msg 헤더 파일
#include "dynamixel_sdk/dynamixel_sdk.h"            // sdk 헤더 파일

#define ADDR_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_GOAL_POSITION          116                // TORQUE_ENABLE, GOAL_POSITION, PRESENT_POSITION 주소 값 define
#define ADDR_PRESENT_POSITION       132

#define PROTOCOL_VERSION                2.0            // 프로토콜 버전

#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        2000000             // 변조 속도 define (2000000 bps)
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller

#define TORQUE_ENABLE                   1                 // Value for enabling the torque
#define TORQUE_DISABLE                  0                 // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      -150000           // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      150000            // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b


// 모터 값 읽어오기

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dxl_test_node");         // 노드 이름 초기화

  // Dynamixel 제어를 위한 코드
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);   // PortHandler 초기화, 포트 경로 설정
  // getPortHandler() 함수는 포트 경로를 DEVICENAME으로 설정하고, dynamixel::PortHandler를 준비함

  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);   // PacketHandler 초기화, 프로토콜 버전 설정
  // getPacketHandler() 함수는 프로토콜 버전을 선택하여 패킷 구성 방법을 설정함

  int index = 0;          // index 변수 선언, Dynamixel이 회전하는 방향
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result, 패킷 통신 중에 발생하는 오류를 나타냄
  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position(목표점) 저장
  uint8_t dxl_error = 0;                          // Dynamixel error (내부 오류)
  int32_t dxl_present_position = 0;               // Present position

  // Open port (포트를 열어 다이나믹셀과 시리얼 통신을 함)
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");    // 포트를 열지 못하는 경우 예제 종료
    return 0;
  }

  // Set port baudrate (열린 포트에서 통신 속도 설정)
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    return 0;
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  // portHandler가 처리하는 포트를 통해 DXL_ID를 가진 Dynamixel에 명령을 보내 TORQUE_ENABLE 값인 1바이트를 ADDR_TORQUE_ENABLE 주소에 씀
  // 그 후 dxl_error 수신, 통신 오류가 발생하지 않은 경우 0 반환
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }

  // Write goal position
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, 500, &dxl_error);
  // portHandler가 처리하는 포트를 통해 DXL_ID를 가진 Dynamixel에 명령을 보내 4바이트 값을 ADDR_GOAL_POSITION 주소에 기록
  // 그 후 dxl_error 수신, 통신 오류가 발생하지 않은 경우 0 반환
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  // portHandler가 처리하는 포트를 통해 DXL_ID를 가진 Dynamixel에 명령을 보내 TORQUE_DISABLE 값인 1바이트를 ADDR_TORQUE_ENABLE 주소에 씀
  // 그 후 dxl_error 수신, 통신 오류가 발생하지 않는 경우 0 반환
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }

  // Close port
  portHandler->closePort();

  ros::NodeHandle nh;           // node handle 선언

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
