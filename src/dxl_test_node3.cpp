#include "ros/ros.h"                        // ros 기본 헤더 파일
#include "std_msgs/Int32.h"                 // msg 헤더 파일
#include "dynamixel_sdk/dynamixel_sdk.h"    // sdk 헤더 파일

#define ADDR_TORQUE_ENABLE              64      // 각각의 주소 값 define
#define ADDR_GOAL_POSITION              116
#define ADDR_PRESENT_POSITION           132

#define PROTOCOL_VERSION                2.0     // 프로토콜 버전

#define DXL_ID                          10      // Dynamixel ID : 10
#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1

#define PI 3.14159265            // 파이 값 define

// 두 위치 이동
int main(int argc, char **argv)

{
  ros::init(argc, argv, "dxl_test_node3");
  ros::NodeHandle nh;

  uint8_t dxl_error = 0;                      // 내부 오류
  //int dxl_goal_position[2] = {0,2000};
  int dxl_comm_result = COMM_TX_FAIL;         // 패킷 통신 중 발생하는 오류
  //int index = 0;

  int32_t dxl_present_position = 0;           // Present position 변수 선언
  int32_t dxl_present_position2 = 0;

  // PortHandler 초기화, 포트 경로 설정
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  // PacketHandler 초기화, 프로토콜 버전 설정
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port (포트를 열어 다이나믹셀과 시리얼 통신을 함)
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
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

/************************************************************************/

  double T = 1000.0;  //1000 ms = 1sec
  double t=0.0;
  int dt = 10;    //ms


  int dxl_goal_position[2] = {0, 2000};       // goal_position 변수 선언 (이동하고자 하는 두 위치 지정)

  int index = 0;             // index 변수 선언


  ros::Rate loop_rate(100);

  // dxl_present_position 값 불러오기
  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);

  // 두 위치 사이를 1-cos 궤적으로 왕복하는 코드
  while (ros::ok())
  {
    if (t <= T){

      int goal_pos = dxl_present_position + (dxl_goal_position[index] - dxl_present_position)*0.5*(1 - cos(PI* t/T));
      // goal_pos 변수 선언

      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, goal_pos, &dxl_error);
      // portHandler가 처리하는 포트를 통해 DXL_ID를 가진 다이나믹셀에 명령을 보내 위에서 계산된 goal_pos 값을 ADDR_GOAL_POSITION 주소에 기록

      ROS_INFO("goal_pos= %d" ,goal_pos);       // goal_pos 값 출력

      ros::spinOnce();

      loop_rate.sleep();

      t=t+dt;

    }

    else{

      if (index == 0)
      {
        index = 1;        // index를 이용해 dxl_goal_positon 값을 바꿔줌
        dxl_present_position = dxl_goal_position[0];      // 현재 위치 재설정
      }
      else
      {
        index = 0;
        dxl_present_position = dxl_goal_position[1];
      }

      t = 0;        // t 값 초기화

    }
  }

  return 0;
}
