#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

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
#define DXL_MAXIMUM_POSITION_VALUE      150000
#define DXL_MOVING_STATUS_THRESHOLD     20                // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dxl_test_node2");

  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);     // PortHandler 초기화, 포트 경로 설정

  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);     // PacketHandler 초기화, 프로토콜 버전 설정

  int index = 0;          // index 변수 선언, Dynamixel이 회전하는 방향
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result, 패킷 통신 중에 발생하는 오류를 나타냄
  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position(목표점) 저장

  uint8_t dxl_error = 0;                          // Dynamixel error (내부 오류)
  int32_t dxl_present_position = 0;

  // Open port
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

  // Set port baudrate
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

  float T = 1000;
  float t = 0;
  int dt = 10;
  int goal_pos = 0;

  // dxl_present_positon 값 불러오기
  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }

  ros::NodeHandle nh;

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    goal_pos = dxl_present_position + (2048-dxl_present_position)*(t / T);

    if (t < T){
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, goal_pos, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->getRxPacketError(dxl_error);
      }
    }

    t = t+dt;

    ros::spinOnce();

    loop_rate.sleep();
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

  return 0;
}
