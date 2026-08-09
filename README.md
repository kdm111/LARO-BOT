# TIL

### 1일차
agent.Dockerfile 생성
* 사용자 명령 해석
* 어떤 팔을 움직일 지 계산


로봇이 동작할 액션 인터페이스 생성
* Pick : 물건을 집는 동작

*패키지와 노드의 정확한 구분*

패키지
 * 정체성 : 빌드, 배포의 단위.
 * 정적 : 디스크에 존재한다.
 * 하는 일 : 코드를 담고, 빌드되고, 설치, 공유
 * 명령 : `colcon build`, `ros2 pkg list`

노드
 * 정체성 : 실제로 돌아가는 프로세스 하나
 * 성격 : 동적(실행 중), 메모리에 있음.
 * 하는 일 : 토픽 주고 받고, 액션 서버/클라이언트로 일한다.
 * 명령 : `ros2 run`, `ros2 node list`


*설계 시 기준*

노드 : 한 문장으로 설명 되는 역할. 이 노드는 어떤걸 구독해 어떤걸 발행한다.
1. 책임이 다른가? : 카메라(30hz 루프), 명령당 LLM 계획은 1번
2. 따로 죽거나 재시작 해도 괜찮나? : 추론이 멈춰도 LLM 에이전트는 살아야 한다.
3. 다른 머신이나 컨테이너로 갈 수 있나? : 제어(MCU), agent(PC or Jetson)

패키지 : 의존성, 공유, 언어, 환경에 따라 판단한다.
1. 여러 노드가 공유하고 있나? : 독립 패키지(인터페이스)
2. 의존성이 무겁거나 다른가? : MoveIt <> LLM 
3. 언어가 다른가? : C++(ament_cmake) <> Python(ament_python)
4. 따로 빌드하거나 재사용, 배포해야하는가?  

따라서 인터페이스를 따로 빼야 하는 이유는 agent가 타입을 사용하기 위해 motion에 의존해야 한다. 순환 의존을 방지하기 위해서라도 따로 빼야 한다. 

인터페이스 패키지 : node 0개, bringup 패키지 : node 0개


*CMAkeLists와 packages.xml이 하는 일*

이 두 파일이 존재해야, 패키지가 선언될 수 있다.

package.xml : 나는 누구이고 무엇이 필요한가? : 패키지의 선언
패키지의 이름(name)(colcon의 기준), 패키지의 의존성(depend), 빌드 타입(build-type)이 정의 된다.

CMakeLists : 나를 이렇게 만들어라. 작업 지시 파일
이름(project), 의존성(find_package), 진짜 일(rosidl_generate_interfaces). 

빌드 했는데도 이전 패키지가 남아 있을 경우
`rm -rf build/ log/ install/` 깨끗이 처음부터 다시 빌드한다.
 

*도커의 바인드 마운트*
compose.yaml의 한 줄
volumes: ./:/ws 프로젝트 폴더를 도커의 /ws로 연결한다. 


*노드 생성 시 필요한 파일 정리*
arm_agent패키지 안에 arm_agent파이썬 모듈 정의

package.xml : ros 신분증. 이 폴더를 ROS 패키지로 인식시키는 파일
읽는 파일 : colcon(빌드 순서, 이름), rosdep(의존성)

setup.py : 빌드 방법, 진입점 정의
파이썬 패키지를 어떻게 설치, 빌드하는지 정의
entry_points : 함수를 실행 파일로 만듦

setup.cfg : 실행 파일을 어디에 설치할지 정의
실행파일을 lib/arm_agent에 설치
ros2 run은 실행파일을 install/package/lib/package 안에서 찾는다.

resource/arm_agent(빈 파일) :
ament는 설치된 패키지 목록을 색인으로 관리한다. 자기 이름의 빈 파일을 색인 폴더에 만들면서 "arm_agent 패키지 있음"의 표시로 ros2 pkg list / ros2 run이 빠르게 찾는다. "존재 자체가 의미"

arm_agent/__init_-.py : 
파이썬 패키지다 표시. 파이썬 규칙으로 import가 가능하고 비어 있어도 이 파일이 모듈로 인식될수 있도록 표시



### 2일차

1. 액션 계약 확정
LLM 에이전트가 tool 스키마가 스킬 서버 인터페이스에 묶여있게 설계 되어 확정했다.
Pick/Place/MoveTo 액션의 결과를 success와 FailureReport로 통일했다. 이걸로 실패의 정보 출처를 하나로 통일하였다.
MoveTo는 SRDF에 이름 붙인 관절값으로 이동한다. 목표가 이미 관절 공간에 있으므로 IK를 풀지 않는다. 다만 현재 자세는 매번 다르므로 경로는 매 호출마다 플래너가 새로 생성한다. 
애초에 못잡는 GRASP_FAILED와 놓치는 GRIPPER_EMPTY를 분리하였다. 서로의 복구 동작을 다르게 할 계획이다.
goal에 시도횟수 attempt를 추가하였다. AI Agent가 가지고 있고, 서버는 오직 실행만 한다.

2. 로보티즈의 스택을 그대로 사용하기 위해 조사하여 의존성을 설치했다.

3. 컨테이너 구조
gazebo와 moveit을 띄우려면 ros:jazzy-ros-base에 없는 게 많다. move_group은 자주 재시작하는데 gazebo까지 죽으면 씬이 초기화된다.
sim/moveit 두 서비스가 동일한 sim 이미지를 공유한다. 

Dockerfile : 레시피 : 우분투 깔고 읽고 등등
이미지 : 다 만들어진 파일시스템 스냅샷 이를 통해 docker build가 보고 만든다.
컨테이너 : 이미지를 실행한 프로세스
서비스 : compose의 선언. 이 이미지로 이런 이름의 컨테이너를 이 볼륨과 이 환경 변수로 띄워라. 

세 컨테이너가 같은 브릿지 네트워크에 있고 ROS_DOMAIN_ID가 같다. 

4. 벤더 스택 실행
벤더 : 자기 회사 로봇을 ROS에서 쓸 수 있도록 같이 배포하는 소프트웨어 묶음. 그것이 벤더 스택
/ws에서 실행하는 소프트웨어는 출처가 세 갈래이다.

나의 코드
 * arm_interfaces # 내가 정의한 인터페이스
 * arm_agent : LLM 에이전트
벤더 스택 src/third_party
 * open_manipulator_description : URDF/xacro, 메시, 링크 치수
 * open_manipulator_bringup : launch, hardware_controller
 * open_manipulator_moveit_config : SRDF, kinematics.yaml, ompl_planning
 * dynamixel_hardware_interface : ros2_control의 hardware_interface 구현체. read()/write()를 DYNAMIXEL Protocol 2.0 패킷으로 번역. 
 * dynamixel_interfaces : 위에서 사용되는 메시지와 서비스의 정의
오픈소스 생태계 apt
 * ros2_control, joint_trajectory_controller, GripperActionController
 * MovIt2, OMPL
 * Gazebo Harmonic, gz_ros2_control, ros_gz_bridge
 * DDS(rmw)
커널 : 호스트 우분투

벤더 스택이 오픈소스 생태계에 관절은 5개이고 조인트 리밋은 이거고, 컨트로러는 이렇게 붙여라. 를 알려준다.
데이터에 가까우며, URDF, SRDF, YAML, launch

Gazebo는 sim에 설치, move_group과 Rviz는 moveit에 두 컨테이너는 파일시스템과 프로세스 공간이 격리되어 있다.

DDS가 컨테이너의 경계를 넘어 moveit의 move_group이 sim의 /joint_states를 보고, /arm_controller/follow_joint_trajectory 액션 서버를 찾아 궤적을 보낸다.

MoveIt 썼습니다에서
**MoveIt의 OMPL 플래너가 만든 궤적을 우리 스킬 서버가 액션으로 받아 실행하고, 실패 시 FailureReport로 구조화해서 에이전트에 올립니다.**

로봇 회사의 OMX 스택을 무수정으로 재사용하되, 실행환경을 도커 컴포즈의 두 컨테이너로 분리한다. 컨테이너 경계를 넘어 DDS 디스커버리가 동작함을 확인하고, RViz에서 plan & Execute한 궤적이 Gazebo의 팔을 움직이는 것으로 스택 전체가 연결됨을 검증했다. 직접 작성한 것은 Dockerfile, compose 정의, 의존성 고정(.repos)이며 로봇 설정 파일은 고치지 않았다.

5. MoveIt2를 이루는 파일들
MoveIt2는 여러 설정 파일을 조립해서 move_group이라는 노드를 만들어내는 프레임워크. omx_f_moveit.launch.py, MoveItConfigsBuilder 가하는 일이 그 조립이다.

1. URDF - 로봇의 몸
URDF는 링크(뼈)와 조인트(관절)의 트리로 이루어여 있다.
링크 이름, 관절 이름/타입/축/한계, 오프셋, 관성(mass, inertia), 충돌 형상, 시각 메시. **로봇이 물리적으로 어떻게 생겼는가?**
```
<joint name="" type=""> 
  <origin xyz="0 0 0.01">
  <limit>
</joint>
```
name이라는 회전관절이고 부모로부터 10mm 올라간 곳에 붙어 있다.
origind은 링크의 치수이다. 관절에서 나오는 **origin 링크의 치수들을 통해 해석적 IK를 유도**할 수 있다.
순기구학(FK)는 오프셋 값들을 관절각으로 회전시켜 차례로 곱해 나가는 것이고, 역기구학(IK)는 그 반대를 푸는 것이다.

2. SRDF(Semantic Robot Description Format) - URDF만으로 설명이 안되는 것들
이 로봇을 어떻게 다룰지에 대한 의미론(semantics)

그룹 : 어느 관절들을 한 덩어리로 계획할 것인가?
```
<group name="arm"> joint1~5 + end_effector_joint
<group name="gripper"> gripper_joint_1, gripper_joint_2
```
URDF는 관절 8개가 트리로 연결되어 있다는 것만 알고 있다. 팔 5개는 같이 계획하고 그리퍼는 따로다 라는 걸 쓸 자리가 없다.


명명 자세(group state) : MoveTo의 target_name
```
<group_state name="home" group="arm">
  <joint name="joint2" value="-1.57">
```
MoveTo 액션이 조회할 표.

수동 관절 
```
<passive_joint name="gripper_joint_2">
```
이 관절은 모터가 없다. 따라 움직인다. 그래서 gripper_controller가 gripoper_joint_1 하나만 잡는다.

가상 관절
```
<virtual_joint name="world_fixed" type="fixed" parent_frame="world" child_link="link0"/>
```
로봇 밑동은 세상에 고정되어 있다. 모바일 로봇이라면 floating이 된다.

충돌 무시 쌍
link2, link3은 항상 붙어 있으니 충돌 검사에서 빼라. 이게 없으면 플래너가 자기 자신과 충돌한다고 판단해서 경로를 찾을 수 없다.

3. kinematics.yaml : IK를 누가 푸는가
```
arm:
  kinematics_solver : kdl_kinematics_plugin/KDLKinematicsPlugin
  kenematics_solver_timeout: 0.005
  position_only_ik: True
```
KDLKinematicsPlugin은 수치해석적 IK이다. 해를 유도하는 것이 아니라 목표에 가까워질 때까지 조금씩 고쳐나간다. 초기 값에 따라 성공 실패가 나뉘고, 같은 목표를 두 번 풀면 다른 답이 나올 수 있다. 나의 해석적 IK가 비교당할 상대.

벤더의 `kinematics.yaml`은 `position_only_ik: True` 5축으로는 6D 포즈를 만족할 수 없어 방향 제약을 통째로 포기한 설정이다. 피킹에는 부적합하므로 해석적 IK로 대체한다.
하지만 피킹의 경우 그리퍼가 아래를 향해야 한다. 팔이 옆으로 누워서 블록 위치에 손목을 갖다대고 도착했다 라고 생각할 수 있다.

**우리는 임의의 방향 문제를 푸는것이 아니다. 그리퍼가 아래로 향하는 것과 손이 블록으로 정렬되는 것이다. 임의의 방향이 아니다**

4. ompl_planning.yaml
Open Motion Planning Library, RRT, RRT-Connect, PRM 같은 샘플링 기반 플래너들의 모음집이다.
관절 공간에 무작위 점을 뿌리고 충돌 없는 것들을 이어붙이는 것

CheckStartStateCollision : 시작 자세부터 충돌이면 계획하지 말아라
OMPL : 경로(공간상의 점)를 찾는다. 시간 개념이 없다.
AddTimeOptimalParameterization : 그 점들에 시각과 속도를 붙여 궤적으로 만든다. 이때 joint_limits.의 속도/가속도의 한계를 지킨다.

경로는 어디를 지나는가 궤적은 언제 거기 있는가 

5. joint_limits.yaml
default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1

URDF에도 limit velocity가 있지만 그건 하드웨어 최대치이고 이 파일로 덮어쓰거나 축소한다. 
데모 영상에서 팔이 느리게 움직인다면 이 숫자 때문이다.

결론
move_group은 이 전부를 파라미터로 받아서 뜨는 하나의 노드이다. 스킬 서버가 말을 걸 상대가 된다.


6. MOCK 스킬 서버

1. ROS의 파라미터 = 노드가 소유한 설정 손잡이
토픽/서비스/액션은 노드들 사이의 통신이다. 데이터가 A에서 B로 흐르지
파라미터는 노드 자신의 설정이다. 흐르는 게 아니라 거기 그냥 있는 값이다. 노드의 메모리 안에 있고 노드가 필요할 때 읽는다.
노드는 별도의 프로세스 이다. 프로세스 안의 변수를 바깥에서 바꾸려면 문을 하나 뚫어야 한다.
ROS에서 프로세스 간 주고 받는 수단은 세가지이다. 토픽, 서비스, 액션, 파라미터 읽기/쓰기는 "요청/응답" 모양으로 서비스이다.
rclpy가 노드를 기본으로 상속한다. 노드를 만들때 파라미터 서비스를 자동으로 붙인다. 

### 3일차

오랜만에 작업. 다시 전체적인 흐름 정리.
계약 + 가짜 팔 
1. 액션 계약을 먼저 못 박고 가짜 스킬 서버로 에이전트를 검증하고 있음.
arm_interfaces에 팔의 행동이 정의 되어 있음.
현재 arm_agent는 /command 를 듣고 로그만 찍는다.
목 서버 내부에는 각기 다른 액션이 하나의 함수 _execute를 공유한다.

### 4일차

목표 : 해당 테이블 위에 물체가 있을 때 각 모터를 몇 도 돌릴지 결정해서 그 물건 위에 올려놔야 한다.
모터 각도를 다 알면 손 끝이 어디 가는지 계산할 수 있다. 이를 정기구학(FK)라고 한다.
손끝을 여기에 두고 싶다고 했을 때 모터 각도를 거꾸로 알아가야 한다. 이를 역기구학(IK)라고 한다.
지금까지 에이전트를 하나 만들었고 move_to, pick, place 라는 세 가지 동작을 액션 서버(현재는 arm_mock_skills)로 보낼 수 있다. 

여기에서 스킬 서버는 네 단계로 나온다.
1. 물건이 공간 어디에 있나? 현재 하드코딩 할 예정 추후 카메라로 변환
2. 그 위치에 손끝을 놓으려면 모터를 몇 도 씩 돌려야 하나? -> **IK**
3. 그 각도로 부딛히지 않으려면 어떻게 경로를 가야하나? -> MoveIt / OMPL
4. 실제 모터를 돌린다. -> ros2_control

현재 omx의 모터는 5가지가 있다.
모터 1(베이스) : 팔 전체를 물건쪽으로 겨눈다.
모터 2,3,4(어깨,팔꿈치, 손목굽힘) : 셋 다 같은 방향으로 위아래 꺾인다. 이 과정에서 손을 그 거리와 높이에 맞추는 역할을 한다.
모터 5(손목) : 물건에서 잡기 좋은 방향으로 손목을 비틀면서 집는다.

카메라에서 본 그림은 어느 방향에 있는 점이다. 모터1은 팔 전체를 좌우로 돌리는 역할을 하므로 제일 먼저 각도가 결정된다.
모터1이 제대로 돌아가면 물건은 팔 바로 정면에 오게 된다. 2D 화면처럼 해당 거리와 높이에 따라 물건을 집는 2,3,4가 결정된다.

평면 위에 점에 닿는 방법은 크게 두 가지가 있다. 위에서 아래로 점위에 닿는 것과 아래에서 위로 닿아도 점에 도달할 수 있다.
모터 2,3 으로 물건쪽으로 가게 만들 수 있고, 모터 4로 해당 물건에 그리퍼의 방향(위 아래)을 결정할 수 있다.

모터 1 : 팔 전체를 물건 쪽으로 가게 만든다.
모터 2,3 : 물건 쪽으로 팔 전체를 뻗는다.
모터 4 : 그리퍼를 해당 물건에서 위, 아래 쪽을 보게 만들 것인지 계산한다.
모터 5 : 그리퍼를 비튼다.

**모터 1**
모터 1이 있는 곳이 원점이다. 해당 물건의 좌표를 (px, py)라고 할때 모터 1이 움직이는 각도는 atan2(py,px)가 된다.
atan2는 x축에서 몇 도 틀어져 있는지를 돌려준다. atan(py/px)를 쓰지 않는 이유는 앞,왼쪽 뒤-오른쪽을 구분하지 않는다.
atan2는 px, py 부호를 모두 따로 봐서 네 방향 모두 올바른 각을 돌려준다.

**모터 2, 3**
모터 2 -L1- 모터 3 -L2- 목표
이 관계에서 모터 2를 원점으로 세 점이 삼각형을 이룬다. 
목표가 (r,z)에 있을 때, 모터 2에서 직선 거리를 D라 하면 D = sqrt(r^2  + z^2)
D^2 = L1^2 + L2^2 - 2\*L1\*L2COS(theta)
세 변을 알면 각도가 나오게 된다. L1, L2는 로봇의 기본 스펙으로 계산되고 D는 목표가 주어졌을 때 계산이 된다.
COS(theta) = (L1^2 + L2^2 - D^2) / (2 \* L1 \* L2)
COSIGN은 같은 값일 때 각이 2개가 나온다. 두 해가 나오게 되는데 이 값이 팔꿈치를 위로 접근하는가 혹은 아래로 접근하는가를 결정한다.

**모터 2**
팔이 팔꿈치에서 꺾여 있어. L1은 목표를 똑바로 안 가리킨다. 살짝 틀어야 하는 양을 결정해야 한다.
theta_shoulder = (목표를 똑바로 겨누는 각) +- (꺾여서 틀어야 하는 보정각)
목표 거리가 두 팔을 쫙 펴야 하는 경우 D = L1 + L2이면 
cos(theta) = (L1^2 + (L1+L2)^2 - L2^2) / (2\*L1\*(L1+L2)) = 1
theta = 0이 된다.

cos의 값 범위는 -1 ~ 1사이이다. 이를 초과하면 ErrorCode의 UNREACHABLE이 된다.
계산값이 D > L1 + L2이면 cos < -1이 되고 D < |L1 - L2| 이면 cos > 1이 된다.


**모터 4**
그리퍼 방향을 결정한다.
모터 2 - L1 - 모터 3 - L2 - 모터 4 - L3 - 그리퍼 끝
방향을 먼저 정하니, 모터4의 위치가 정해진다. 그리퍼가 목표 바로 위에서 아래로 내려오니까 모터 4는 목표에서 L3만큼 바로 위에 위치한다.

**모터 5**
팔이 가리키는 축을 중심으로 그리퍼를 비튼다.
값은 물체 방향에서 오고 위치 IK에서는 자유 변수가 된다.

### 5일차
ros2 pkg create arm_kinematics --build-type ament_cmake
ik.hpp : 해당 패키지에서 사용되는 헤더 파일
ik.cpp : 역기구학을 계산하는 계산 파일
test_ik.cpp : 해당 역기구학 테스트 파일
package.xml : 이 패키지가 무엇을 필요로 하는가의 매니페스트
CMakeLists.txt : 작업 지시 파일(어떻게 만든다)


```shell
ament_uncrustify --reformat arm_kinematics
colcon test --packages-select arm_kinematics # 테스트 실행
colcon test-result --all --verbose # 판정
```

이제 실제 IK 문서로 작성

cos의 해가 없다. 이걸 어떻게 전달하나? ErrorCode.UNREACHABLE 일 경우 arm_kinematics가 arm_interface에 의존하게 된다.
이 라이브러리는 순수하게 유지해야, ROS 없이 gtest로 테스트가 가능하다.
따라서 `std:optional<double>` double 하나를 리턴하던가 아무것도 없음. 도달 가능하면 각도를 불가능하면 nullout을 반환한다.

theta 1
몸통 전용 모터값
atan2를 사용하는 이유
점 x,y가 있을 때 원점에서 그 점으로 향하는 화살표가 +x 축에서 몇 도 틀어져 있나
1,0 -> 0, 0,1 -> 90, -1,0 -> 180, 0,-1 -> -90

theta 3
팔꿈치 전용 모터값
cos_theta = (l1^2 + l2^2 - d^2)/(2\*l1\*l2)
std::fabs cmath의 double  전용 절대값. 소수가 잘리는 함정을 피하기 위함
if (|cos| > 1) return nullopt : acos에 범위 밖을 넣으면 NaN 반환

theta 2
어깨 전용 모터값
해당 하는 방향으로 옮기고, 
r : 어깨 기준 목표의 수평거리
z : 어깨 기준 목표의 높이
l1 : 위팔의 길이
l2 : 아래팔의 길이
theta2 = atan2(z, r) + beta
목표를 똑바로 겨누는 각 + 팔꿈치가 꺾인 만큼의 보정각

theta 4
손목 전용 모터값
해당 위치 위 혹은 가로로 손목이 위치할 수 있도록 하는 식
wrist_r = r - l3 * cos(phi)
wrist_z = z - l3 * sin(phi)

theta는 관절각, phi는 접근각


FK
FK를 만들어야 하는 이유. 관절각 FK 손끝 위치 계산 -> IK 관절각
즉 처음 각으로 돌아오는 지 확인해야 한다.

중요한 점 : **각 모터는 세상이 아니라 바로 앞 링크를 기준으로 돈다**

팔꿈치 모터는 위팔 끝에 볼트로 박혀 있다. 그 모터는 수평이 어디 있는지 모른다. 자기가 붙어 있는 위팔을 기준으로 아래팔을 theta3만큼 꺾을 뿐이다. theta3는 위팔 대비 상대각이 된다.

따라 아래팔이 수평에서 몇 도 인지 확인하려면 앞에서 쌓인 각들을 모두 더한다. 모두 더하는 이유는 각 링크가 모두 수평이기 때문.

그리퍼의 절대값 phi = theta2 + theta3 + theta4 의 합산이므로
그리퍼의 각도값인 theta4는 phi - theta2 - theta3가 된다.


왕복 테스트
현재 정기구학과 역기구학을 통해 팔이 왕복하는 테스트를 하려고 한다.
FK와 IK가 theta3(팔꿈치각)을 계산할 때 서로 다른 각을 재고 있다.
FK에서 a3 = theta2 + theta3로 되어 있다. 아래팔 방향을 의미한다.
팔을 쭉 피게 되면 꺽지 않아서 theta3는 0이 된다.

IK에서 theta3는 삼각형의 내각을 의미하고 코사인 법칙을 통해 구현하고 있다.
acos((l1 * l1 + l2 * l2 - d * d) / (2.0 * l1 * l2));

팔을 쫙피는 동작을 할경우 d = l1 + l2가 되고 이는 -1으로 theta3가 180도가 된다.
같은 자세인데 각도는 정반대가 된다. 

현재 왕복 테스트에서 에러를 확인하였고 theta3를 내는 get_elbow_angle의 분자를 음수로 고치고 get_shoulder_angle을 수정한다.

내각값에 마이너스를 씌우면 180도에서 뺀 각이 나오게 된다. 
관절각은 = 180 - 삼각형의 내각이된다.


### 6일차

한 손목점에 팔이 닿는 방법이 둘이다. 손끝을 고정하고 위로 세울 수도 있고 밑으로 세울 수도 잇다.

팔꿈치 위 : theta3 +acos, theta2 aim(motor1) - beta(팔꿈치에 따라 달라지는 각)
팔꿈치 아래 : tehta3 -acos, theta2 aim + beta

```
colcon test-result --all --verbose
```
Test.xml : ctest 상위 롤업
cppcheck : static 린터 테스트
lint_cmake : cmake 린터 테스트
test_ik.gtest : 내가 쓴 테스트
uncrustify : c++ 포맷 린터 테스트
xmllint : package.xml 린터 테스트

**링크**
링크는 곧은 막대가 이다. 각 링크는 굽혀 있지 않다.

joint origin은 다음과 같다.
urdf 파일 위치 : src/third_party/open_manipulator/open_manipulator_description/urdf/omx_f/
|관절|origin(부모 기준)|axis|우리 모델|
|joint2|(0,0,0.0635)|y|어깨 위치|
|joint3|(0.0415,0, 0.11315)|y|L1|
|joint4|(0.162, 0, 0)|y|L2|
|joint5|(0.0287, 0, 0)|x|L3 일부|

**기하 모델에서 산출되는 값이 항상 모터의 값과 항상 일치하지 않는다**
MoveIt에서도 IK가 남아있지만 일단 직접 계산할 수 있어야 한다.

**KDL**
Kinematics and Dynamic Library 오픈소스 C++ 기구학 라이브러리.
Moveit2가 IK를 풀 때 사용하는 것이 KDL이 된다.

KDL은 찍어가며 좁혀간다(수치해석/반복법)
1. 관절각을 아무거나 하나 찍는다.
2. FK로 그 각이면 손끝은 어디인지 계산한다.
3. 목표와 얼마나 먼지 계산한다.
4. 각을 조금 고쳐서 목표에 가까워 지게 다시 계산한다.
5. 충분히 가까울 때까지 계속 계산한다. 못 맞추면 5ms 안에 포기한다.

KDL의 강점은 범용성이다. 아무 로봇, 아무 자유도에 다 적용가능하다. 내 해석 IK 식은 바로 팔 하나에 특화되어 있다.

**모터에게 어떤 각도로 가라. 이 명령은 어떻게 실행되는가**
1. 지금 관절이 몇 도 인지 읽는다(엔코더)
2. 목표까지 어떻게 부드럽게 갈지 계산한다.
3. 모터에 명령을 쓴다.
4. 이걸 초당 100번(Hz) 반복한다.

이 과정을 반복하는 것이 바로 컨트롤러이다.

이를 ros2_control로 반복할 것이다.
controller_manager : 지휘자. 컨트롤러들을 로드/시작/정지한다. 100Hz로 돌린다.
hardware_interface : 드라이버. read()로 관절값을 읽고, write()로 명령 보냄. sim=gz_ros2_control(Gazebo), 실기=DynamixelHardwareInterface(실모터)
controllers : 실제 로직.

arm_controller - JointTrajectoryController 팔을 움직이는 놈. 궤적을 받아 100Hz로 보간하여 관절을 몬다.
joint_state_broaodcaster - 움직이는 게 아니라 관절값을 읽어 /joint_states로 발행만 한다.
gripper_controller - 그리퍼 담당

궤적을 발행하면 arm_controller가 구독해서 받고 100Hz로 관절까지 부드럽게 몬다.

```
ros2 control
```
컨트롤러를 조회,관리 하는 명령줄 도구이다. `ros2 control list_controllers` 지금 로드된 컨트롤러랑 상태(active/inactive)를 보여준다.


**gazebo로 관절 돌리기**
```
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: [joint1, joint2, joint3, joint4, joint5], points: [{positions: [0.0, 1.219, -1.219, 0.0, 0.0], time_from_start:{sec: 2}}]}"
```

고정각 : 링크 모양에 박혀서 모터가 바꾸지 못하는 각. 금속 브래킷 모양이 결정한다.

모터가 0일 때 ㄱ자로 굽는다.
theta3=0 상태에서 팔이 직선이 아니다. 위 팔에 고정각이 박혀 있다.  
(1.219(2), -1.219(3), 0(4)) 이 값을 넣으니 팔이 쫙 펴져 수평이 일어난다.

지금 사용하는 로봇은 고정각이 있기 때문에 일부 보정이 필요하다.

### 7일차

**해석해와 수치해법**
해석해 : 방정식을 손으로 풀어서 목표를 넣으면 atan2, acos를 통해서 답이 나오게 된다. 그래서 빠르다.
수치해석 : 일단 아무 관절각으로 찍는다. FK로 손끝이 어디있는지 계산하면서 야코비안을 사용해서 관절값을 조금씩 수정해 낸다.

raw orocos_kdl
작게 쪼개지고 KDL::Chain 조립을 직접하면서 체인이 뭔지 실제로 배운다. 바로 CI에 올라간다.
plugin 버전
로봇이 실제로 쓰는 솔버를 사용하면서 사용 가능함.

raw orocos_kdl을 사용하는 이유.
타이핑한 해석 IK로 수치해석의 약점을 이해하는 방향으로 간다. 
수치해석의 약점 3개는 
1. 반복법이라 느릴 수 있다.
2. local minima에 빠질 수도 있다. 수치해석은 오차를 0으로 줄이는 방식이다. 로봇 팔은 팔꿈치를 위/아래 두 가지의 해로 나올 수 있는데, 해가 팔꿈치 아래에만 있으면 풀리지 않는다. 초기값에 따라 성공/실패가 갈리고 못찾아도 타임아웃일 뿐이다. (단, 이번 벤치처럼 관절 여유자유도가 크면 local minima 위험은 잘 안 드러난다 — 실제로 KDL은 100% 성공했다.)
3. 리턴값이 해 없음이 아니라 타임아웃으로 나온다.

**KDL::Chain**
우리가 팔을 링크-조인트-링크-조인트 이렇게 그리고 있다. KDL도 똑같이 볼 수 있다. 팔 하나 Chain, 그걸 이루는 마디 Segment.
Segment = Joint + Frame 

Joint : 이 마디가 어떻게 도나, 모터의 회전축
Frame : 이 조인트에서 다음 조인트까지 얼마나, 어느 방향으로 떨어졌나, URDF의 origin offset

세그먼트 = 이 축(Joint)으로 돌고 이 다음 이 만큼 뻗어라(Frame)
이걸 5개 이어붙이면 팔이 KDL 안에 생기게 된다.

*https://docs.ros.org/en/indigo/api/orocos_kdl/html/index.html*
AI가 거짓말 하는줄 알았는데 진짜 ros document에 있다.

비교 결과
목표 5000개 채점 KDL FK로 1mm이내 도달

       성공률    평균시간    최악시간
solve_ik   90.1%     0.329 us    13.064 us
kdl_ik  100.0%    31.605 us    92.219 us

KDL이 100% 결과가 나옴. KDL은 위치 3개만 맞추면 되고 관절이 5개라서 해의 자유도가 남아돌아 LMA가 거의 항상 위치 해를 찾는다.
직접 만든 solve_ik는 한가지 계열(앞으로 뻗는)의 해만 낸다. 팔을 뒤로 접혀 손끝 수평거리(tip.r)가 음수가 되는 목표에는 reachable=true를 내면서도 KDL에선 틀린 위치에 간다. (theta1을 180도 뒤집어 푸는 blind spot). 독립 오라클 KDL로 채점했기에 이 거짓 성공을 잡아낸다.

해석 IK(solve_ik)는 반복 호출할 때 누적 이득을 가진다. KDL은 방향을 포기해 피킹에 부적합하다. 

목표 5000개 채점 KDL FK로 1mm이내 도달

       성공률    평균시간    최악시간
solve_ik   90.1%     0.331 us    11.540 us
kdl_ik  100.0%    31.724 us   563.682 us
solve_ik  공식 밖 0, 오류 494 중 tip.r <0: 494

(16일차에 덧붙임) 위 두 표의 시간은 지금 값이 아니다.
벤치가 목표를 어깨 기준으로 만들고 있었는데 solve_ik는 base_link 기준을 받는다.
9.8cm 어긋난 목표로 잰 값이었고 뒤접힘 가드도 벤치에 반영된 적이 없었다.
08-07에 고치고 다시 재니 solve_ik 90.1% / 평균 0.215us / 최악 9.380us,
kdl_ik 100% / 24.956us / 49.225us 다. 약 100배.
성공률 90.1%는 그대로인데 뜻이 바뀌었다.
7일차의 494개는 거짓 성공이었고 지금은 가드가 걸러낸 명시적 거부다. 오류는 0이다.
숫자는 같고 성질이 반대다.

재검사 결과 `tip.r < 0` 이면 팔이 뒤로 접히는 자세이고 실제 피킹 (앞으로 가서 물건을 집는 행동)은 하나도 섞여 있지 않다. 모터 제어 루프는 100Hz=10,000us 이므로 둘 다 안에서 잘 작동한다.

ROS 패키지로 빼는 값어치가 있는건 **ROS 그래프에 참여하는 노드**이다.
토픽, 서비스, 액션으로 통신하고 rclcpp를 사용해야 한다.

arm_skills 서버 생성
arm_kinematics는 ros없이 순수 lib로 유지된다.
**노드는 도구가 아니다.**
reach_once.bench_ik는 통신 안 하는 계산 도구라 arm_kinematics에 두고 액션으로 통신하는 arm_skills 서버를 생성한다.

노드는 ros 그래프에 들어가 있는 프로세스로 계속 떠 있으면서 액션으로 명령 받고 moveit에 계획 시키고 결과를 돌려준다.
arm_kinematics는 순수 라이브러리로 계산만하고 reach_once.bench_ik는 계산기로 답을 뽑아 보여준다.
skill_server는 인터폰으로 주문 받아 일하고 보고한다.
도구는 ros가 필요하지 않고 노드는 무거운 ros 의존성이 필요하다.
따라 arm_skills는 별도 패키지로 분리한다.

### 8일차

github 액션 환경에서 자동 테스트 확립을 위해 ci.yml 파일 생성

**MoveGroupInterface(MGI)**
moveit2가 준 c++ 클라이언트 라이브러리 클래스
1. 노드도 아니고 플래너도 아니다. 프로세스 안에 사는 객체이다. 계산등등은 move_group이 한다.
2. 생성할 때 하는 일 (MoveGroupInterface(node, "arm"))
* /robot_description + /robot_description_semantic : 토픽에서 모델을 받아온다.
* /joint_state를 구독해서 현재 자세를 계속 추적한다.
* move_group과 통신할 액션,서비스 클라이언트들을 세팅한다.
세 가지가 다 토픽/통신이라 노드가 spin 중이여야 했다. 
3. 메서드는 전부 요청을 포장한뒤 move_group에 전송하고 결과를 대기한다.
setNamedTarget("home"), setPoseTarget(), plan(), move() 다 나의 요청을 메시지로 포장해서 move_group에 보내고 답을 기다리는 함수이다. MGI는 그 성공/실패 코드만 받아온다.

**MoveIt**
launch 파일은 두 노드를 띄운다. move_group + rviz2_moveit. 
move_group = MoveIt 서버(두뇌), MoveItConfigsBuilder
파라미터
SRDF : 그룹, 이름자세,충돌쌍
kinematics : IK 솔버
joint_limits : 속도/가속 스케일
controllers : 궤적을 어느 컨트롤러에 보낼지(arm=FollowJointTrajectory, gripper=GripperCommand)
OMPL 파이프라인 : 샘플링 플래너

move_group은 URDF를 /robot_description 토픽에서 받는다. 따라서 가제보를 먼저 띄워야 한다.
skill_server는 move_group은 /robot_description_semantic을 발행하는걸 받아온다.


MoveIt의 제어구조
skill_server(무엇을=명령) -> move_group(IK+OMPL+시간파라미터화+궤적발사) -> ros2_control(그대로 추종)

**경로를 짜는 것이 아니라 move_group을 운전할 뿐이다.**
skill_server.cpp는 액션을 받으면 로그만 찍고 succeed하는 place 홀더이다.
MoveGroupInterface는 플래너가 아니라 리모컨이다.
진짜 일을 하는 건 벤더 launch(omx_f_moveit.launch.py)가 띄우는 move_group 노드이다.
IK + OMPL 경로 계획 + 시간 파라미터화를 다해서 컨트롤러로 궤적을 쏜다.
execute() : setNamedTarget(target) -> move() -> moveit::core::MoveItErrorCode::Success로 성공 판정

skill_server의 execute()는 그 옆에 붙어서 arm그룹이 home 자세로 가게 명령을 던진다.
우리는 경로계획은 짜지 않는다.


qnode.cpp 는 ROBOTIS OpenMANIPULATOR GUI 뒤에서 도는 ROS 노드이다. 
q = Qt, node = ROS 노드

가제보 환경 띄우기 : `ros2 launch open_manipulator_bringup omx_f_gazebo.launch.py`
moveit 띄우기 : `ros2 launch open_manipulator_moveit_config omx_f_moveit.launch.py use_sim:=true`
스킬 서버 띄우기 : `ros2 run arm_skills skill_server`

MGI -latched 토픽으로 모델을 받는다. 그래서 gazebo + moveit(use_sim:=true)를 반드시 띄워야 한다.

**MoveTo**
현재는 move가 어느 단계에서 실패했는지 구분을 하지 못한다. 정밀하게 할 경우 다시 수정해야 한다.

**Pick**
1. 그리퍼 제어 : 그리퍼는 또하나의 MoveGroup - arm이랑 똑같이 운전
2. Pick 액션 서버 골격 : Pick.action으로 서버를 하나 더 띄운뒤 MoveTo 골격을 재사용한다.
3. 접근 자세 계산 : 기존 solve_ik를 사용하여 물체 위 phi 아래 방향으로 접근각을 풀기(move_group position_only KDL이 못 하는 방향 제어)
4. 시퀀스 조립 : approach + grasp + lift + 파지판정(GripperCommand result)

```c++
std::thread{std::bind(&SkillServer::execute_move_to, this, goal_handle)}.detach();
```
함수는 detached 스레드에 실어서 단일 동작을 blocking 하지만 노든느 다른 스레드에서 계속 spin되어 서버 자체는 멈추지 않고 goal도 받고 로그도 찍을 수 있다.

이제 skill_server가 solve_ik를 부를 건데 solve_ik는 다른 패키지에 존재한다. 하지만 arm_kinematics는 라이브러리를 자기 안에서만 쓰게 만들어서 밖에서 쓰지 못한다. 
따라서 열어 놔야 한다.
컴파일된 라이브러리와 헤더가 공유공간 install/ 로 나가야 한다.

build/ : 임시 패키지별 작업 폴더 여기 있는건 그 패키지만 본다.
install/ : 공유공간. ros2와 다른 패키지가 여기에서 찾는다. 

solve_ik의 좌표 프레임은 왜 base_link 기준인가?
역기구학은 joint2 원점에서 푸는게 더 편하다. 
하지만 공개 API가 어깨 기준이면 tf2,카메라 좌표(둘다 base/world 기준)와 대조가 안된다.
내부 기하함수는 어깨 프레임을 유지하고 공개 solve_ik는 base_link 기준으로 입력을 받는다.
입구에서 어깨 오프셋을 빼서 내부 좌표로 변환한다.
joint1 origin(-0.01125, 0, 0.034), joint2(0,0,0.0635)
base_to_shoulder_x = -0.01125, base_to_shoulder_z = 0.0975
변환 px = x - X_off, pz = z - Z_off

또한 팔이 뒤집히는 자세는 도달 불가로 처리. Moveit에서는 팔이 뒤집혀도 도달가능하다고 처리할 수 있으나 팔의 특성상 뒤집히는 동작이 늘어날 수록 안됨

지금까지 한 일
**해석 IK -> base 프레임 변환 -> 모터각 변환 -> move_group 경로 계획 -> 가제보에서 움직임**

두 가지 해로 분리 위해/아래해
move() = 계획 + 즉시 실행
plan() = 계획만 실행하지 않고 성공/실패를 알려준다.

두 가지 해가 나오므로 각 가지를 plan으로 분리하고 성공하는 첫 가지를 execute로 실행. qnode.cpp 역시 plan하고 실행하는 방식을 쓴다.

그에 따라 skill_server에서 각 해를 도출하도록 설정 후 도달하지 못할 경우 다른 값으로 도달하도록 설정

파지 판정은 GripperCommand result의 stalled로 하고 있는데 MGI로는 result에 접근이 안된다. MGI는 그리퍼를 여닫을 뿐, GripperCommand 액션의 raw result(stalled/reached_goal)을 돌려주지 않는다.

그리고 가제보에서 물리 마찰이 안잡히면 stalled 자체가 안 뜰 수도 있다.(sim 그리퍼는 물체를 통과하기도 한다).


approach와 grasp는 높이만 다른 가지 선택 이동이다.
execute_pick에 인라인된 파지 함수를 헬퍼 함수 move_to_pose(x,y,z,phi,label)로 빼서 approach, grasp, lift에 높이만 바꿔서 부른다.

pick 시퀀스
1. move_gripper(open)
2. move_to_pose(물체 위 접근)
3. move_to_pose(물체 높이로 내려감)
4. move_gripper(close) 
5. move_to_pose(obj_z + 0.06) lift


place 시퀀스
1. place-approach
2. place-lower
3. open
4. place-retreat


### 9일차

에이전트 e2e 현재 까지의 흐름
/command String : agent 구독
_build_plan : 명령 문자열 파싱하여 스텝 리스트로 생성
_run_step : 스텝 하나를 액션 goal로 전송
skill_server
solve_ik 
가지선택
move_group
팔 이동
on_result 성공이면 다음 스텝, 실패면 재시도 abort

현재 agent와 실서버(skill_server) 연동 확인. 

DetectedObject 메시지
검출된 물체들과 마지막으로 검출된 시간 정리

SceneState 메시지
지금 보이는 카메라 화면에 어떤 물체들이 어디 있나의 스냅샷. 물체 여러 개를 담고 각 물체는 id + pose 마지막 관측시각


팔은 현재 gazebo 안에서 돈다. 인지는 물체 위치를 만들어서 skill_server에 전달해야 한다.
같은 가제보 안에서 가상 카메라로 하면 카메라도 블록도 팔도 가제보 안에에 있으므로집을 수 있다.

카메라와 물체를 생성해야 하므로 둘다 urdf/sdf 가제보 센서 플러그인으로 만든다. 

커스텀 패키지에 카메라를 따로 정의해야 한다.
카메라 링크 + 조인트 + 가제보 센서 플러그인

```
gz topic -e -t /camera/image_raw -n 1
```
현재 가제보 환경에서 카메라가 나오는지 확인

현재 omx 가제보 환경은 physical 만 존재하지 sensors 환경이 존재하지 않는다. 
따라서 새로운 환경을 만들어야 하고 환경을 그쪽으로 옮겨줘야 한다.

가제보 환경의 world를 명시해야 한다.
```
ros2 launch open_manipulator_bringup omx_f_gazebo.launch.py world:=$(ros2 pkg prefix arm_perception)/share/arm_perception/worlds/sensors_world
```
카메라 환경으로 보는 방법
```
ros2 run rqt_image_view rqt_image_view
```
Unified Robot Description Format : ROS의 표준 로봇 설명 언어
Simulation Description Format : 가제보 시뮬레이터의 네이티브 언어
XACRO : URDF, SDF를 프로그래밍 하게 만드는 도구

빨간 블록 gz sim에 띄우기
```
ros2 run ros_gz_sim create -world empty -file $(ros2 pkg prefix arm_perception)/share/arm_perception/models/red_block.sdf -name red_block
```
가제보 환경 모델 확인
```
gz model -m red_block -p
```


### 10일차

환경 gazebo, moveit, agent, block spawn까지 한번에 띄우는 너무 많아서 통합 laucnh 파일 생성
launch 하나에서 gazebo moveit, skill-server까지 같이 띄우게 된다.


```
ros2 launch arm_perception sim_bringup.launch.py scene:=''
```

```
ros2 run rqt_image_view rqt_image_view
```

**open-cv**
이미지를 numpy 배열로 다루는 컴퓨터 비전 라이브러리

이미지 = 숫자 배열
640*480 컬러 이미지 = shape(480,640,3) 배열 세로가 먼저 온다.
채널 순서는 BGR, 
픽셀 좌표 (u,v) = (열, 행)원점은 좌상단

**cv_bridge**
ros와 opencv 배열 통역기
frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
가제보는 rgb8로 준다. 반드시 bgr8을 명시

RGB vs HSV
H(색상) / S(Saturation) 채도 / V(Value) 명도
조명이 어두워 지면 RGB는 세 값이 모두 변하지만  HSV는 V만 떨어지고 H는 유지된다.

OpenCV의 H 범위는 8비트안에 다 넣기 위해 0~179, 0~360가 아니다. S,V는 0~255

빨강만 구간이 두 개가 된다. 
색 상환에서 빨강은 시작이자 끝이다. 0과 179 부근에 걸친다.
따라 inRange 마스크를 두 개로 만들어 OR로 합친다.

최종 목표는 색으로 걸러내고(마스크), 지저분한거 치우고(모폴로지), 덩어리를 찾고, 그 중심을 구한다.

1. 마스크 - 흑백 도장
inRange를 통해 판정한다. 범위 안이면 255(흰색), 밖이면 0(검정).
결과는 원본과 같은 크기의 흑백 이미지 한 장.
여기서부터 색 개념은 사라지고 흰 덩어리의 모양 문제가 된다.

2. 모폴로지 - 침식 후 팽창
마스크에는 잡티가 낀다. 조명 반사, 압축 노이즈로 흰 점 한두개가 튄다.
침식(erode) : 흰 영역의 테두리를 한 겹 깎는다.
팽창(dilate) : 흰 영역의 테두리를 한 겹 붙인다.
침식을 먼저하면 외톨이 점은 한 겹이라 완전히 소멸하고, 큰 덩어리는 얇아질 뿐 살아남는다.
그다음 팽창하면 살아남은 덩어리만 원래 크기로 돌아온다. 죽은 점은 되살아나지 않는다.
이 순서가 열기(MORPH_OPEN)다.
반대 순서(팽창 후 침식)는 닫기(MORPH_CLOSE). 덩어리 안의 구멍을 메울 때 쓴다.

커널 : np.ones((5,5), np.uint8)
침식과 팽창에서 말하는 "한 겹"의 크기와 모양을 정하는 값.
키우면 잡음을 잘 지우지만 작은 덩어리까지 날아간다.
임계값 손잡이는 이 커널 크기와 MIN_AREA 두 개다.

3. 윤곽선 : 덩어리의 테두리 좌표
findContours 는 흰 덩어리들의 경계선을 따라서 간 점 좌표 목록을 돌려준다.
contourArea(c)로 그 덩어리의 면적을 재서 작으면 잡음으로 처리한다.
덩어리 개수 = 목록의 길이. two_blocks 씬에서 검출 2개가 나오는 게 여기서 정해진다.

RETR_EXTERNAL : 바깥 윤곽만 가져온다. 도넛이면 안쪽 구멍은 버린다.
CHAIN_APPROX_SIMPLE : 직선 구간의 중간 점을 버리고 꼭짓점만 저장한다. 사각형이면 점 4개.
안 쓰면 테두리 픽셀을 전부 들고 있어 메모리만 먹는다.

4. 모멘트 : 평균
덩어리에 속한 흰 픽셀들의 좌표를 다 더한 값
m00 : 흰 픽셀의 개수
m10 : 모든 픽셀의 u 좌표를 다 더한 값
m01 : 모든 픽셀의 v 좌표를 다 더한 값

m10 / m00 = u 좌표들의 합 / 개수 = u의 평균
u = m10 / m00 # 평균 x -> 덩어리 중심의 열
v = m01 / m00 # 평균 y 덩어리 중심의 행

무게 중심 = 좌표의 평균 이고, 모멘트는 그걸 구하려고 미리 계산해둔 합계

**카메라는 어느 방향인지는 알지만 얼마나 먼지는 모른다.**

픽셀 하나는 점이 아니라 레이저 포인터를 쏜 방향이다. 카메라의 물체는 멀든 가깝든 전부 같은 곳에 찍힌다. 

### 11일차

*다시 정리*
블록에서 반사된 빛이 카메라로 들어온다. 앞으로 할 일은 빛의 경로를 다시 짚어 방향을 잰다. 이를 back-projection(역투영)이라고 한다.
블록은 픽셀의 선에서 어딘가에 있는 것이다.

투영은 정보를 버리는 연산이다. 3d점이 2d점으로 바뀌는 것이다. 여기에서 버려지는 것이 거리이다.

(389,323), camera_info fx=fx=554.38, cx=320, cy=240
640x480 이미지가 몇 칸인가(크기)
cx,cy 320,240 그중 정중앙 칸이 어디인가(위치)

이상적인 카메라는 렌즈가 센서의 정중앙에 있지만 실물은 미세하게 삐뚤어져 있어 캘리브레이션 하면 다른 값이 나온다.

u : 검출된 블록 중심의 열
v : 같은 중심의 행
cx : 주점(광축이 이미지를 뚫는 칸의 열)
cy : 주점의 행
fx, fy : 초점거리를 픽셀로 잰 값

u,v는 원점이 왼쪽 위다.원점은 왼쪽위 구석이고 cv2.moments로 뽑는 값이다.

cx,cy는 왜 빼는가
광축이 찍히는 칸은 왼쪽 위 구석이 아니라 주점이다. 그런데 u,v는 구석 기준으로 세어져 있다. 
u - cx, v - cy로 방향을 세운다.

fx, fy - 픽셀로 만든자
fx = width / 2 / tan(hfov/2) = 554.38 
xacro의 각도와 width 두 개에서 나온 값이다. 의미는 렌즈에서 센서까지의 거리를 픽셀 단위로 잰 것
fx가 크면 확대, 작으면 광각
fx는 우리가 계산할 일은 없다.

**카메라 프레임 > 월드 프레임**
프레임이란 축 3개와 + 원점 하나이다. 카메라(0.1245, 0.1497, 1)
원점에서 내 x축 방향으로 0.1245, 내 y축 방향으로 0.1497, z축 방향으로 1만큼 간점

camera_joint의 축과 월드에서 방향을 만들어야 한다.

perception이 하는 최종 순서
1. 역투영
2. 프레임 변환
3. 평면교차

**코드 계산**
축 3개를 손으로 계산하지 않는다. TF가 이미 그 값을 갖고 있다.
내가 하는 일은 광선 위의 점 두 개를 만들어서 TF에 넘겨 계산한다.

near = (0,0,0) 렌즈의 중심
far = (x,y,1) 깊이 1에서의 점

이 두 점을 do_transform_point로 world로 옮기면 옮겨진 두 점이 world에서 광선이 된다.

평면 교차는 나눗셈 하나이다. 
t = (PLANE_Z - near_w.z) / (far_w.z - near_w.z)
PLANE_Z는 블록 중심의 높이이다. 즉 블록 높이의 절반

far_w.z - near_w.z 가 0보다 크면 하늘을 찍은 픽셀이니 버린다.


skill_server에서 /scene_state 구독으로 하드코딩된 포즈를 대체한다.

1. 스레드가 두 갈래로 이 데이터를 만진다. MultiThreadedExecutor가 돌리는 콜백 스레드와 실행 스레드가 동시에 같은 변수를 읽고 쓸 수 있다.

2. Place.target_id는 카메라가 보는 대상이 아니다. /red_block은 /scene_state에 있다.


### 12일차

1. LLM이  실제로 하는 일
LLM은 문장을 토큰이라는 조각으로 잘라서 다룬다.
영어는 1단어 1토큰에 가깝지만 한국어는 1글자 1~2토큰이다. 같은 뜻이여도 한국어가 토큰을 2~3배 먹는다.

컨텍스트는 토큰을 컨텍스트 만큼 기억한다는 말이다.

모델이 하는 계산은 지금까지의 토큰들을 보고 다음에 토큰의 확률표를 만든다.
문장을 만드는 과정은 이 전과정을 계속 반복하는 것이다.
여기에서 temperature는 그 확률표를 얼마나 뾰족하게 만드는 것이다.
0에 가까우면 결정론적, 0.8정도면 다양하고 창의적으로 계산한다.

LLM의 특징
1. 같은걸 물어도 답이 달라질 수 있다. 확률에서 뽑으니까 skill_server가 항상 같은 입력에 같은 관절값을 내는 것과 반대이다.
2. 그럴듯 한데 틀린걸 만든다. 환각 : 확률이 높다. 자연스럽다이지 사실이 아니다.
3. 형식을 거의 지킨다. 하지만 형식이 달라질 수 있다.

**LLM 출력을 신뢰할 수 없는 외부 입력으로 간주하고 도구 스키마라는 계약과 검증 계층 폴백 사다리로 감싼다.**
이 걸 뒤집으면 하네스가 된다.

LLM은 답이 매번 다르다 -> 결정론 테스트를 LLM 바깥에 둔다.
없는 스킬을 만든다. -> 화이트리스트 검증을 통해 해당 스킬이 아니면 거부한다.
형식을 흘린다. -> 구조화 출력을 강제하고 파싱 실패시 문자열 파서를 폴백한다.

2. 무엇이 모델을 다르게 만드는가?
어떤 모델이 좋은가는 답이 없다. 질문을 바꿔야 한다. 우리 과제는 어느 축에서 높은 값을 요구하는가?가 질문이 되어야 한다.

크기는 축이다.
파라미터가 많을수록 똑똑하다는 건 대체로 맞다. 똑똑함이 값어치로 바뀌는 구간이 정해져 있다.

Qwen3.5 모델을 BFCL로 잰 값이 4B -> 9B(+15.8)까지는 되게 크지만 9B에서 27B(+2.4)로 넘어갈때는 큰 차이가 없다.

같은 조사에서 FunReason 7B는 83%가 나오지만 Llama3.3 70B 모델에서는 73%가 나온다.

정말 중요한 축은 **크기가 아니라 그 일에 맞춰졌는가이다**

축은 3갈래로 묶을 수 있다.
능력 : 할수 있는가, 도구 호출용으로 조정되었나, 크기, 기능 플래그
운영 : 메모리에 올라가는가? 답이 얼마나 빨리 오나, 얼마나 길게 기억하나
통제 : 출력 형식을 문법으로 막을 수 잇나, 같은 입력에 같은 출력을 보장하나, 어제와 오늘이 같은 모델인가. 이 통ㅊ제 축은 모델의 능력이 아니라 실행 환경이 정한다.

3. 벤치마크 읽는 법

벤치마크에는 두 종류가 있다.

범용 과목
MMLU : 57개분야 객관적 지식. : **포화** 되었다.
GPQA : 대학원 수준 과학 추론
HumanEval : 파이썬 함수 작성
LMAreana Elo : 사람이 두 답을 보고 고른 선호

포화 : 모든 응시자가 95점을 넘게 되면 그 시험은 더 이상 사람을 구분하지 못한다.

도구 호출
BFCL(Berkeley Function Calling Leaderboard) : 호출을 문법적으로 맞게 만들었나(AST), 실제로 실행되나, 호출하지 말아야 할 때 참는가?(관련성 판별)
tau-bench : 끝까지 시켜보고 최종 DB 상태로 채점. 과정이 아니라 결과
IFEval : 약 500개를 지시하고 프로그램이 기계적으로 검증

BFCL의 관련성 판별이 흥미롭다. pick도 place도 아닌 명령이 들어왔을 때 아무 스킬이나 부르지 않고 거부하는가를 측정한다.

측정값이 항상 정확한가?
도구 호출 벤치마크 12종 감사에서 평가자와 사람의 판정 불일치가 18.5%가 나온다. 
LiveMCPBench, 완전히 같은 조건으로 23회 재실행. 18.9점 오차

같은 모델을 같은 시험으로 23번 재봤더니 18.9점이 흔들렸다. 리더보드에서 모델간 격차는 대개 그보다 작다
9B에서 27B는 2.4점 정보 밖에 차이나지 않는데, 측정 노이즈가 18.9점이 나오고 잇다.

점수가 높아도 내 스키마에서는 깨진다.
벤치마크는 자기 형식으로 잰다. 우리 형식이 아니다. 형식 강제 없이 도구 호출 인자가 스키마에 맞는 비율 22% ~ 67%. 문법으로 강제시 100%
스키마가 복잡해지면 추출 성공률이 86.9% -> 70.0% 일 수 있다.

따라서 실무에서는 자기 시험지를 만든다.
케이스 20~50개정도 만들고 실제로 관측된 실패에서 뽑는다. 채점은 이진 통과, 케이스마다 여러번 시행

pass@k : k번 중 하나만 맞아도 성공
pass^k : k번 중 모두가 맞아야 성공.

코드 생성에서 @k를 사용해도 로봇에서는 ^k 의 테스트를 사용해야 한다. 로봇의 행동은 여러개를 만들어 놓고 고를 수 없다. 움직이면 끝이다.

4. 과제가 진정으로 요구하는 것

과제의 실제 모양은 어휘가 3가지 정도이다.
pick, place, move_to
(deliver는 계약에 없다. pick+place의 혼용)

출력이 굉장히 짧다. 가장 긴 계획이 2스텝이다.

파라미터는 두 종류이다. object_id(대상 물건), target_name(행동 자세 정의)
시도 횟수는 에이전트가 센다.

호출이 드물다. 인지 노드처럼 24Hz로 돌지 않는다.

실패해도 죽지 않는다. 그래서 모델이 완벽하지 않아도 된다.

운영에서 VRAM의 공존 제약이 존재한다. 현재 12GB 카드에서 gazebo-rviz-moveit이 돌아야한다. 7B에서 4.9GB가 물고 있으면 남는게 얼마 없다. 즉 우리의 수준은 그래픽 카드가 결정한다.

우리 상황에서 불필요한 것
vision, 멀티턴, 긴 컨텍ㄹ스트, thinking

**난점**
LLM은 그럴듯한 다음 조각을 뽑는데, 이 과정에서 과잉생산을 할 수 있다. 
로봇 계획에서 pick 앞에 move_to가 오는건 매우 그럴 듯하다. 이 문제는 모델을 바꿔서 푸는 문제가 아니라 프롬프트와 검증기로 푼다.
요청 안한 move_to는 스텝 상한을 걸고, target_name 화이트 리스트를 SRDF 실제 값으로 제한한다. 없는 물체는 scene_id를 통해 대조해야 한다.

우리는 통제 가능한 모델이 필요하다.


5. 로컬이나 API인가
4장에서 만든 명세가 채점표가 된다. 축마다 어느 쪽이 이기는지 보고 가장 중요한 축에서 이긴 쪽을 고른다.

지연 - 로컬이 이긴다
호스티드 프론티어 AI는 첫 글자까지 네트워크 왕복으로인해 총 소요가 2~3초대가 나온다. 로컬의 경우 0.79 ~ 1.47초가 나온다. 
로컬이 2~3배가 빠르다.

재현
로컬 temperature = 0으로 두면 확률표에서 항상 1등만 고른다. 호스티드는 통제 수단이 아예 존재하지 않는다. 앤트로픽에는 seed가 아예 없고 openAI는 temperature/top_p/top_k를 주면 400에러로 거부한다.

형식 강제 - 둘 다 된다.
양쪽 다 하드 보장을 제공한다.

양쪽의 공통 함정 셋
1. 모델은 문법의 존재를 모른다. ollama 공식 권고는 스키마를 프롬프트에도 문자열로 넣어라이다.
2. 잘린 JSON은 여전히 가능하다. 토큰 한도에 걸리거나 조기 종료하면 문법은 지켰는데 미완성이다
3. 유효 != 올바름 : blue_block을 집으라는 계획은 문법적으로 완벽하다.

비용은 예상보다 굉장히 낮기 때문에 금액으로 결정하면 안된다.
로컬이 공짜라서 로컬 사용은 틀린 이야기이다. 맞는 이유는 지연과 재현성에 있다.

로컬 제약 - VRAM 경합
VRAM 경합에서 꼬이게 된다. 로컬의 실질적 약점은 모델을 키우는게 능사가 아니다.

로봇은 실제 온보드에서 추론한다. 공장에 네트워크가 없을 수도 있고 지연을 감당하지 못할 수 있다.

6. 결론

내가 원했던건
LLM이 어떤 모델이 좋은 모델인가 판단하는 기준이 있어야 한다고 생각했다.
1. 과제를 뜯는다 : 입력/출력 모양, 어휘 크기, 호출 빈도, 실패 허용도
2. 축에 요구 강도를 매긴다. : 능력 / 운영 / 통제 불필요 목록을 반드시 쓴다.
3. 리더보드는 참고만 한다. : 측정 노이즈보다 작은 점수 차는 무시한다.
4. 제약이 후보를 자른다 : VRAM,지연 같은 하드 제약으로 후보군을 좁힌다.
5. 남은 후보는 자체 시험지로 : 20~50 케이스를 만들고 이진 채점하고 , pass^K를 선택한다.

과제 적용 결과
1. 과제 : 어휘 3개, 출력 1~2스텝, 명령당 1회, 폴백 있음
2. 요구 중요도: 통제 = 높음, VRAM = 높음, 크기 = 중, 낮음, vision,멀티턴, 긴 컨텍스트 불필요
3. 리더보드 : 4B -> 9B 1.8점 < 측정 노이즈 18.9점
4. 제약 : gazebo, moveit 과 공존 VRAM 6GB 이하
5. 시험지 : 아직 안 만듦



test_llm_planner
llm을 테스트하는 시험지 생성

테스트를 먼저 작성한 이유
시험지가 곧 검증의 명세이다. 순서를 뒤집으면 모델이 뱉는 것에 맞춰 검증기를 짜게 된다.
모델이 move_to를 끼워 넣으면 규칙은 모델의 버릇에 맞춘 규칙이지 계약이 아니다.

pytest
test_로 시작하는 파일과 함수를 자동 수집한다.
assert 한 줄이 판정이다. 실패하면 좌욱밧을 알아서 찍어준다.


LLM을 시험장에서 쫓아낸다.
plan의 세번째 인자가 그자리이다. 테스트는 FakeLLM을 끼워넣는다.
재는건 파서/검증/폴백이다. 

llm_planner
노드 != 도구
llm_planner는 rclpy를 임포트하지 않는 순수 파이썬 모듈
ROS 없이 pytest로 돌아가고, agent 노드는 이를 감싼다.


LLM 모델 선정
현재 VRAM에 맞게 6GB 이하로 설정
llama3.1:8b : BFCL에서 3등 레이트(https://llm-stats.com/benchmarks/bfcl)
exaone3.5:7.8b : exaone3.5중 가장 비슷한 크기의 모델

두 개를 골라 내가 만든 시험지로 판단한다.

ollama를 docker compose로 올려서 설치한다.

plan() 함수
프롬프트 조립
1차 호출 -> 예외 발생 -> '' 반환
검문소 4층 -> 1. JSON인가 -> 2. 개수 -> 스킬, 필수 인자 -> 4. 씬 대조 move_to 단속 -> 2차호출 -> 검문소 다시 -> 통과, 거부


mock_skill_server, llm 연결

계획 _build_plan() : LLM이 들어갈 장소라서 더욱 더 중요하다.
액션 타입이 셋이라 goal, 객체만 들고 있으면 어디로 보낼 지 모른다. 누구에게, 무엇을 한 쌍으로 묶어 다닌다.

on_command를 받아 llm_planner가 
`[{skill, object_id, target_id, target_name}]` 이런 식으로 돌려준다.


agent의 on_command의 변경
llm_planner와 연결하여 plan을 만든다.

지금 ollama와 agent를 http를 통해 연결한다.
http + json 직렬화는 0.5~0.7ms 시간이 들어간다. 모델을 적재(153ms)하고 token을 생성하는 eval은 390ms 정도가 소요된다.
ollama는 같은 컨테이너에 있어도 다른 프로세스로 돌아간다. 데몬이라서 파이썬에서 함수로 부를 방법이 없다. HTTP API 오직 하나 뿐이다. 

지금 처럼 모델과 agent를 따로 두면 agent 만 다시 시작해도 모델은 VRAM에 남는다.

```
ros2 topic pub -1 /command std_msgs/String "{data: 'pick red_block'}"
```

```
[INFO] [1785754775.304873269] [agent]: LLM 계획 채택 : [{'skill': 'pick', 'object_id': 'red_block'}]
[INFO] [1785754775.306304096] [agent]: goal 수락됨
[INFO] [1785754779.310121688] [agent]: 결과 : success=True, code=0
[INFO] [1785754779.310352695] [agent]: 시퀀스 완료
```

/command
명령
agent.py : ROS 노드
plan()
llm_planner : 순수 모듈
http
ollama

test_llm_planner.py : 검증 : llm planner의 검증
eval_llm_planner.py : 모델, 프롬프트: 모델의 검증


### 13일차

```
python3 -u -m agent.evel_llm_planner \
  -- model model_name1 model_name2
  --k 3 --tag v2 --out-dir /ws/eval_results
```
실호출 시험지 100케이스(정상 50 / 거부 50), k=3, pass^3, temperature 0.3

| 모델 | 정확도 | 특징 |
| --- | --- | --- |
| exaone3.5:7.8b | 71/100 | 총점 1위. 거부 35/50으로 유일하게 참을 줄 안다. 지연 0.37초로 제일 빠름. 대신 시키는 일(정상 36/50)은 3위 |
| gemma4:e2b | 51/100 | 정상 42/50으로 2위인데 거부 9/50. 지연 2.83초 = exaone의 7.6배 |
| qwen3.5:9b | 49/100 | 정상 44/50으로 과제 수행 1위. 그런데 거부 5/50으로 거의 안 참는다. 지연 3.81초 = 10배 |
| llama3.1:8b | 31/100 | 탈락. 정상 카테고리 4개가 50% 미만. pick 앞에 요청 안 한 move_to를 계속 끼워 넣는다 |

총점 1위와 과제 수행 1위가 다르다.
차이가 거의 전부 거부(R) 카테고리에서 나오는데, 그 실패의 원인이 모델이 아니라
프롬프트에 "못 하면 거부하라"가 없기 때문이었다. 그래서 아직 모델을 고르면 안 된다.

지연은 통과한 케이스만 골라 잰 중앙값이라 재시도 편향이 없다.
5장에서 로컬을 고른 이유가 지연이었는데 qwen과 gemma는 호스티드(2~3초)보다도 느리다.

한국어 정확도 : exaone 62% / gemma 55% / qwen 48% / llama 31%

현재 한국어 영어 비율이 2.5 : 7.5 정도로 나오고 있어. 5:5로 수정 후 다시 테스트 182개 예정



카메라가 본 좌표는 얼마나 잘 맞는가
가제보 씬에 블록을 두고 검출된 픽셀을 world계 좌표로 역투영해서 정답과 비교했다.
정답을 아는 상태로 재는 것이다. 실물에는 정답이 없어서 캘리브레이션이 필요하다.
실측시 오차가 5mm 이하로 검출되었다.
단 팔이 카메라 시야 밖에 있을 때의 값이다. 팔이 블록 위에 있으면 26mm까지 틀어진다(16일차).
실물에서는 카메라를 받아 캘리브레이션을 해야 하지만 시뮬레이션에서는 가볍게 가능하다.

개별 실측
(389, 323) -> (0.181, -0.069) 정답 (0.18, -0.07) 오차 1mm
(257, 305) -> (0.202,  0.065) 정답 (0.20, 0.06) 오차 5mm

역투영은 광선과 평면의 교차로 좌표를 구한다. 검출하는 것은 물체의 무게중심이고 무게중심은 블록의 중심에 있으므로 중심의 높이에 잡아야 한다.
오차가 7mm에서 1mm로 줄어든 것은 그러면 평면 Z값을 물체의 무게중심 만큼 내렸기 때문이다.(2.5cm -> 1.25cm)
14일차에 블록을 실물 치수(4x6x4cm)로 바꾼 뒤에는 중심 높이가 2cm라 PLANE_Z도 2cm다.

새로이 블록과업데이트 하며 

v2 — 한국어 50%로 올리고 짝을 맞춰 다시

v1의 문제는 한국어 29 : 영어 71이었고 분포도 쏠려 있었다(N1 0건, N6 0건, R5 1건).
그래서 "한국어 62%"는 한국어 능력이 아니라 한국어 케이스가 어려웠던 것을 잰 것일 수 있었다.
v2는 같은 의도를 (영어, 한국어) 한 쌍으로 정의하고 기계적으로 펼친다.
씬도 기댓값도 공유하므로 언어만 다르다. 91쌍 = 182케이스, en 91 : ko 91.

| 모델 | 정확도 | 응답속도 | 특징 |
| --- | --- | --- | --- |
| exaone3.5:7.8b | 129/182 (70.9%) | 0.36초 | 총점·속도 둘 다 1위. 거부에서 유일하게 버틴다(R2 12/14, R6 12/12) |
| gemma4:e2b | 102/182 (56.0%) | 2.82초 | 정상은 대등하거나 우세(N1 14/14). 거부가 무너진다(R1 1/18, R4 1/14). exaone의 7.8배 느림 |
| llama3.1:8b | 59/182 (32.4%) | 0.61초 | 탈락. 쉬운 영어 pick(N1)조차 0/14 |
| qwen3.5:9b | 96/182 (52.7%) | 4.63초 | 정상은 최상위(N1 14/14, N4 14/14, N7 12/12)인데 거부가 최하(R2 0/14, R3 0/16). exaone의 13배 느림 |

응답속도는 정상 통과 케이스만 골라 잰 중앙값이다. 거부 케이스는 재프롬프트로 호출이 2배가 되므로 섞으면 안 된다.

★ 한국어 격차가 사라졌다

| 모델 | 영어 | 한국어 | 짝 비교 판정 |
| --- | --- | --- | --- |
| exaone3.5:7.8b | 70.3% | 71.4% | 차이 없음 (en만 5 : ko만 6) |
| gemma4:e2b | 59.3% | 52.7% | 차이 없음 (en만 8 : ko만 2) |
| llama3.1:8b | 31.9% | 33.0% | 차이 없음 (en만 14 : ko만 15) |

v1에서 13%p였던 격차가 v2에서는 -1.1%p가 됐다. 한국어가 오히려 미세하게 높다.
v1의 격차는 언어가 아니라 케이스 난이도를 잰 것이었다.
한국어와 영어가 서로 다른 명령이었고, 한국어 쪽에 어려운 케이스가 몰려 있었다.

이건 N6에서 scene_ids로 배운 것과 같은 논리다. 통제하지 않은 비교는 없는 차이를 만들어낸다.
-> v1의 "한국어 정확도 : exaone 62% ..." 줄은 틀린 결론이다.

시험지가 안정적이라는 증거

| 모델 | v1 (100케이스) | v2 (182케이스) | 응답속도 v1 -> v2 |
| --- | --- | --- | --- |
| exaone3.5:7.8b | 71.0% | 70.9% | 0.37 -> 0.36초 |
| gemma4:e2b | 51.0% | 56.0% | 2.83 -> 2.82초 |
| llama3.1:8b | 31.0% | 32.4% | 0.64 -> 0.61초 |

케이스를 82개 늘리고 한국어 비율을 뒤집었는데 순위도 비율도 그대로다.
응답속도는 소수점 둘째 자리까지 재현됐다.
3장에서 측정 노이즈보다 작은 차이는 무시하라고 했는데, 이제 내 시험지의 노이즈가 얼마인지 알게 됐다. 거의 없다.
단 gemma의 51->56은 좋아진 게 아니다. 케이스 집합이 다르므로 절대 점수는 비교 대상이 아니고 순위만 비교된다.

★ 응답이 느리면 거부 점수를 공짜로 얻는다 (시험지의 결함)

HTTP 타임아웃이 30초다. 넘으면 예외를 먹고 빈 문자열이 되고, 파싱에 실패해서 plan()이 None을 낸다.
그런데 거부 케이스의 정답이 None이다. 즉 타임아웃이 정답 처리된다.

  타임아웃 47건 = 전부 qwen3.5:9b (exaone 0 / gemma 0 / llama 0)

느린 모델일수록 거부 점수가 부풀려진다. qwen의 R 카테고리 점수는 그대로 믿으면 안 된다.
고치려면 "검증기가 거부한 None"과 "호출이 실패한 None"을 구분해야 한다. 지금은 둘 다 None이라 구분이 없다.

예측이 맞은 것과 남은 것

N2(place)는 예상대로 전멸했다. exaone 0/12, gemma 1/12. 둘 다 [place] 대신 [pick, place]를 냈다.
무상태 플래너는 그리퍼가 뭘 들고 있는지 모르므로 place 단독은 자연어로 도달할 수 없다. 시험지 버그다.

거부는 여전히 낮다(exaone R1 10/18, R3 10/16, R4 7/14, R5 8/18).
프롬프트에 "못 하면 거부하라"가 없다는 진단 그대로이고, 아직 안 고쳤으니 당연하다.

N7(표기 견고성)은 신설했는데 문제가 아니었다.
대문자, 공백 중복, 구두점 반복, 군말에 exaone과 gemma 모두 12/12다. llama만 0/12.

★ qwen이 느린 이유 — 모델이 아니라 우리 하네스였다

호출 하나를 서버 시간까지 분해했다(웜 상태, 다른 요청 없음).

  qwen3.5:9b     load 234ms + prompt_eval 102ms + eval 181ms = 517ms
                 그런데 total(서버) 3138ms
                 -> 설명 안 되는 부분 2621ms

  exaone3.5:7.8b total(서버) 369ms
                 -> 설명 안 되는 부분 2ms

qwen의 생성 속도 자체는 78.5 tok/s로 exaone(93.8 tok/s)과 같은 급이다. 모델이 느린 게 아니다.
매 요청마다 어디에도 안 잡히는 2.5초를 쓰는데, 생성 토큰 수(14개)와 무관하게 고정이다.
5장에서 정리한 GBNF 문법 강제가 유력하다. 스키마를 문법으로 컴파일해 토큰마다 마스킹하는데
그 비용이 어휘 크기에 비례하고, qwen 계열은 어휘가 크다.

  틀린 결론 : qwen은 느린 모델이다
  맞는 결론 : format:{schema} 강제의 비용이 모델마다 다르고, qwen에서 특히 비싸다

우리가 그 스키마를 쓰므로 실용적으로는 qwen이 느린 게 맞다.
다만 원인이 모델이 아니라 내 선택에 있다는 게 중요하다. 고칠 여지가 있다는 뜻이니까.

응답속도 두 종류를 구분해야 한다

| 모델 | 정상 통과 중앙값 | 전체 평균 | 차이의 원인 |
| --- | --- | --- | --- |
| exaone3.5:7.8b | 0.36초 | 0.46초 | 거부 케이스의 재프롬프트 |
| llama3.1:8b | 0.61초 | 0.84초 | 재프롬프트 |
| gemma4:e2b | 2.82초 | 5.79초 | 재프롬프트 |
| qwen3.5:9b | 4.63초 | 14.78초 | 재프롬프트 + 타임아웃 47건(30초씩) |

전체 평균을 쓰면 느린 모델이 더 느려 보인다. 거부 케이스는 호출이 2배이고 타임아웃은 30초가 통째로 들어가기 때문이다.
그래서 모델 비교에는 정상 통과 중앙값을 쓴다.

최종 판단

exaone3.5:7.8b를 기본 모델로 한다.
정확도 1위(129/182)이면서 속도도 1위(0.36초)다. 두 축에서 동시에 이긴 유일한 모델이다.
qwen과 gemma는 정상 카테고리에서 exaone보다 낫지만 거부가 무너지고, 무엇보다 8~13배 느리다.
5장에서 로컬을 고른 이유가 지연이었는데 그 이유가 이 둘에서는 성립하지 않는다.


**llm 명령이 해석되는 경로**
agent : plan(command, scene_ids)
llm planner : scene : 씬(현재 디텍션되는 물체들)을 문자열
_call_ollama : http - > exaone으로 전달
exaone(혹은 모델) : 빨간 블록 -> 'red_block', 파란 링 -> 'blue_ring'
_validate_step : object_id가 씬 목록에 없으면 거부하고 문자열 파서로 폴백 하게 된다.

모델이 실수하더라도 검증기사 씬과 대조해 거부하므로 OpenCV가 검출하지 않은 물체가 팔까지 내려가지 않는다.

또한 target_id : 목표하는 물체를 놓는 곳. target_name(사전에 정의된 자세)은 별도의 화이트 리스트(validate_step, llm_planner)로 체크하여 모델의 실수를 방지한다.


### 14일차

블록의 yaw값을 계산하여 world 좌표계에 복원

각도를 변환하지 말고 점을 변환한뒤 각도를 다시 구한다.
cv2.minAreaRect(c)는 회전 사각형의 각도를 준다. 하지만 그것은 픽셀 평면의 각도이다. 우리에게 필요한 건 world 평면의 yaw고, 원근 투영은 각도를 보존하지 않는다. 카메라가 테이블을 정확히 수직으로 내려다보지 않는 한 픽셀각도 != world 각도이다.

따라 다른 방법을 써야 한다. 긴 축의 양 끝점 두 개를 pixel_to_world로 각각 되돌린 뒤 world에서 atan2, M3에서 만든 ray-plane 역투영을 그대로 재사용한다.

하지만 3차원에서 본 결과는 높이가 같이 cv2에 인식 되므로 긴 축을 기준으로 정확한 yaw 계산이 불가능하다. 점검이 필요하다.

현재 red_block은 x=4cm y=6cm z=4cm이다.
가장 긴 변은 카메라의 어느각도에서 봐도 뒤집히거나 하지 않는다. 4\*sqrt(2)는 6보다 항상 작을 수 밖에 없다. 가장 긴 변의 수직으로 theta5만큼 회전 시키면 물건을 집을 수 있다.

그에 따라 cv2의 minAreaRect의 긴 축을 중심으로 pose의 회전을 내보낸다.


그리고 grasp_test 라는 환경을 만들어 red_block과 blue_ring을 추가함.

그리고 perception에서 나오는 노드가 카메라의 opencv의 축값을 받아 world 좌표계의 점으로 변환한다.그리고 skill_server에서 그리퍼를 회전시킬 각을 계산한뒤 그만큼 회전시킨다.

**그리퍼에 물려있다가 떨어뜨리는 이유**
그리퍼에서 잡고 있다가 올릴 때 충돌이 나는 이유는 충돌 직전 이였던 자세가 그리퍼 두께만큼 넘어간 것이다.

up자세가 충돌 직전에 그리퍼를 닫자 넘어간다. lift가 up을 시도할 때 계획 실패, move_to_pose(down) up과 다운은 완전히 다른 자세라 팔이 통째로 재배치. 그 큰 움직임에서 블록을 놓친다.

그리퍼를 닫으면서 손가락이 움직여 여유가 사라졌다. 

theta5 = 0일 때 닫힘 축이 팔의 반경 방향이 아니라 theta1 + 90을 향한다. 근거는 URDF에 있다.
gripper_joint_1 origin(0.0295, 0.0075, 0)
gripper_joint_2 origin(0.0295, -0.0108, 0)
손가락 두 개가 link5의 y축으로 벌어져 있다.

TCP는 손가락의 끝이다. TCP를 물체의 중심에 두면 손가락이 블록 윗절반만 문다. 그 결과 시뮬레이션에서 미끄러져 블록이 세워졌다.(roll = 90). 그 결과 중심보다 1cm 낮춰서 해결했다.



### 15일차

지금까지 pick의 success 판정은 모션이 끝났다는 판정이지 쥐고 있다는 판정이 아니었다. 
move() 반환 SUCCESS : 손가락이 끝까지 닫혔다. 빈 손이다.
CONTROL_FAILED : 목표까지 못 닫혔다. 쥐고 있다.

근거
GripperActionController : 오차가 goal_tolerance 밖인 채로 stall_timeout 동안 속도가 stall_velocity_threshold 아래면 stalled=true, reached_goal=false, -> setAborted
GripperCommandControllerHandle : allow_failure_가 false면 ABORTED를 그대로 전파
따라서 MGI move()가 CONTROL_FAILED

lift 전에 분기했다. 빈손으로 들어올리기 전에 잡혔는지 확인 후 성공했는지 확인한다. 그랩을 실패하면 다시 open으로 되돌리고 잡혔으면 lift 동작으로 분기한다.

**정리**
물체를 쥔 채 놓치는 문제는 접근/후퇴가 직선이 아니라서로 의심했다. move_to_pose가 호출될 때마다 {false, true} 두가지를 처음부터 순회한다. 직전에 사용했던 해를 모른다. 원인은 경로의 모양이 아니라 가지 선택의 무기억성이다.

그에 따라 관절 상태와 가장 가까운 해를 고른다. 최근접만으로는 안되나. 파지중에 반대로 넘어가면 큰 움직임 속에서 블록을 놓친다. 그레서 grasp에 쓴 가지를 lift가 끝날 때까지 잠근다. 
실패 보고가 IK가 풀리지 않음과 plan 실패가 둘다 PLANNING_FAILED가 된다. 그래서 두가지 모두 IK가 풀리지 않으면 UNREACHABLE, 하나라도 풀렸는데 plan이 실패하면 PLANNING_FAILED, 에이전트가 UNREACHABLE에는 재시도하지 않고 중단한다. 도달 못하는 목표에 세 번 매달리지 않는다.
Planning Scene에 물체가 하나도 들어가고 있지 않다. 검출한 좌표는 어디로 갈까에만 쓰이고 충돌 검사에는 쓰이지 않는다. 지금 로봇의 충돌 회피 대상은 자기 몸뿐이다.

가지 선택을 고치면서 move_to_pose를 다시 썼는데 return MoveResult::OK를 추가했다. 그래서 반대쪽 가지 plan후 execute를 막았다. 목표에 도착한 팔이 뒤집히는 현상을 막았다.

현재 씬에 관측이 되지 않으면 프롬프트가 실패하는 상황으로 프롬프트 수정


복구 사이클
regrasp 일경우 home으로 물러나 재인지
2회가 소진되면 중단 

현재 해결할 문제 파지 결정
물체가 red_block, blue_ring의 경우 파지 되었음을 어떻게 인지 해야 할지 결정해야한다. blue_ring의 경우 그리퍼가 끝까지 부딛히고 red_block의 경우 그리퍼가 끝까지 닿지 않는다. 

잡았다라는 센서가 없어서 간접적으로 알아내야 한다.
손가락이 끝에 닿으면 잘못 집은 것이라 판단하여 REGRASP 단계로 빠지고, 못닫히면 뭔가를 물고 있는 것이다.
하지만 시뮬레이터에서는 물체를 살짝 파고들면서 목표에 도달할 수 있다. 물고 있는데도 끝까지 닫혔다가 뜬다.
나쁜건 틀렸을 때 바로 손을 펴버리게 된다.

블록은 손가락을 두께가 막지만 링의 경우 손가락을 막지 않는다. 물체마다 잡았을 때의 모습이 아예 다르다.
관절 신호로만 이를 판정할 수 없다. 힘센서를 달아도 안된다. ring의 무게는 아래로 실리는데 손가락의 축은 옆으로 돌기 때문에 무게가 거의 안걸린다.

1. 손가락이 어디에서 멈췄다. : 각도값을 읽어 손가락을 멈춘다.
2. 물건이 원래 자리에서 사라졌나? 
앞으로 이렇게 두 단계로 나누어 물체를 집었는지 판별한다. 블록은 1,2 단계, 링은 2단계만
블록을 쥔 후부터 gripper_joint_1의 각도를 한번더 읽어야 한다.

arm_skills에서 joint_states를 구독한다. MoveIt의 상태 캐시는 move() 직후 정착 전 값을 주기 때문이다.(실측 캐시 0.4887, 실제 0.00004) 그리퍼 속도가 컨트롤러의 stall_velocity_threshold(0.001) 아내로 200ms 유지되면 그때의 관절 위치를 읽고, 닫힘 목표(theta)에서 얼마나 떨어졌는지로 파지한다. 판정을 만들기 전에 씬 SDF의 물리 스텝이 5ms이다. 이름은 1ms인데 값이 달랐다. 0.001로 고치자 관통이 사라지고 close가 정상적으로 CONTROL_FAILED를 내었다. 쥐고 있어도 관절이 0인 것을 해결 하였다.쥔 상태는 0.4219~0.4400, 빈 손은 0.0001로 갈렸다.

그리퍼가 손으로 가리고 있을 때 블록이 2개로 나뉘어 보이는 것은 블록의 최종 길이를 알고 있으니, 오차 이상일 경우에는 빨간 블록으로 인식한다.

측정에 기준이 `gz model -m red_block -p`로 시뮬레이터의 실좌표를 뽑아 몇 mm가 틀렸다로 수정한다.

파지 유지는 시간이 아니라 자세이다.
lift 자세에서 41.5초를 버텼다. home으로 접으면 몇 초만에 미끄러진다. home은 팔이 접히면서 그리퍼가 뒤집힌다. 쥔채 home으로 가면 물체를 놓는다.

길이로 가림을 거를 수 있는지 체크했다.
long_axis_yaw를 통해 긴 변의 길이를 구해, 사용하려 했으나, 블록의 긴변은 6cm인데, 관측은 6.3cm ~ 8.4cm으로 나온다. 팔에의해 오염 되었을 때는 1.2cm ~ 6.7cm가 된다. minAreaaRect가 감싸는 건 윗면이 아니라 윗면과 옆면의 합집합이다. 이는 카메라의 각도에 따라 달라진다.

약한 가림은 길이로 못 막는다. 6.6cm는 정상범위인데 좌표는 26mm 트린다. 팔이 물체 위에 있는 측정은 나쁜 측정이다. 걸러낼게 아니라 그때 재지 않아야 한다. execute_pick이 시작할때 팔을 치우면 사라진다.


### 16일차

gripper_empty 감지
place의 transfer 구간에서 그리퍼 관절값이 닫힘 쪽으로 떨어졌으면 운반 중 낙하로 판정하고 gripper_empty, stage::transfer로 abort.

상시 감시가 아니라 구간 끝에서 확인한다. 
감시는 move_to_pose가 끝난 직후마다 확인한다. 즉시 멈춰야 하는 일이 생기면 그때 바꾼다.
is_holding()은 쥐었다와 못 읽었다를 구분하지 못한다. 정착 대기가 제한 시간 안에 실패하면 nullopt가 돌아오고 그것도 빈 손으로 읽힌다.
실제로는 joint_state가 안 오는 형태이다.

파지 유지가 시뮬레이션에서 52초로 나오는데 잰 것은 손가락이 물체를 파고드는 데 걸린 시간이다.

파지 실패 : GRASP_FAILED -> REGRASP
도달 불가 : UNREACHABLE -> ABORT
운반 중 낙하 : GRIPPER_EMPTY -> REPLAN -> 비어 있음 (고치기 전. 아래에서 REGRASP로 바꾼다)


regrasp : 물러나서 재인지 후에 손이 비어 있으면 다시 집는다.
pick : grasp_failed : 비어 있음 moveto(home)
place : gripper_empty : 비어 버림 move_to(home), pick


전략 수정 ErrorCode.gripper_empty : regrasp으로 수정
운반 중 낙하는 replan으로 하고 있었는데 물러나 재인지한 뒤 실패한 스텝으로 돌아간다. 그 때 손이 비어있으면 하나 더 끼운다.
복구를 계획에 끼워 넣는 스텝으로 만들면 기존 성공 경로가 알아서 실패한 스텝으로 돌아온다.

_do_recover 함수는 실패한 스텝을 다시 밟는 구조이다. place가 실패한 상황에서 place를 한번 더하면 빈손으로 놓는 시늉을 한다.

현재 pick 신호를 보내고 블록의 위치를 변경하면 어떻게 작동하는지 보는 중

파지 인식 과정
0. /scene_state에서 물체 좌표를 한 번 읽는다. -> OBJECT_NOT_FOUND
1. 물체 위 6cm로 이동 ->  UNREACHABLE, PLANNING_FAILED
2. 물체 중심보다 1cm로 내려감 -> UNREACHABLE, PLANNING_FAILED
3. 그리퍼에 닫으라고 명령 close
4. 그리퍼 관절이 멈출때까지 기다린다. -> 5초 안에 안 멈추면 판정 포기
5. 멈춘 위치를 살짝 쥐었을 때의 경계와 비교한다.
6. 쥠, 행동 고정 후 올라감. 빈손 -> GRASP_FAILED

현재 4번의 판단조건을 그리퍼 속도가 0.001 아래로 200ms 지속에서
위치가 20ms 동안 0.001 미만으로 변하는게 200ms 지속되는걸로 바꾸려고 한다.

위치는 엔코더가 직접 주는 1차 신호고 속도는 누군가 미분해서 채워주는 파생신호이다.
속도 필드가 채워지지 않아. 판정이 불가능하여 그리퍼의 위치를 기반으로 다시 작성하려 한다.

관측
900행을 재보니 그리퍼 속도가 0.001을 넘는 게 0건이었다. 최댓값이 0.00002다.
같은 순간 위치 기록에서 계산한 실제 속도는 초당 0.44다.
close 명령이 리턴하고 201ms 뒤에 판정이 찍혔다. 기다리라고 준 시간의 최솟값이다.
그때 관절값이 0.4296이었고 쥠이라고 했다. 그런데 손은 비어 있었다. 재현 2/2.

고친 뒤
빈 손이면 3.21초 기다려 0.0294를 읽고 빈 손이라고 한다.
쥐고 있으면 1.13초 만에 0.3179를 읽고 쥠이라고 한다.
물체에 막히면 관절이 빨리 멈추니까 정상 파지는 안 느려진다.
GRASP_FAILED가 뜨고 REGRASP로 물러나 새 자리에서 다시 집었다. 48.7초.



팔이 약간 가리면 검출이 얼마나 틀리나

0. 팔을 init으로 치우고 블록만 놓는다. 기준선
1. pick을 걸어 팔이 블록 위로 접근한다
2. 프레임마다 면적, 긴 변, 짧은 변, 무게중심, yaw를 기록한다
3. gz 실좌표와 대조한다

알게 된 것
가림은 blob을 짧은 쪽으로 깎는다. 남는 건 긴 방향 조각이라 긴 변은 마지막까지 안 줄어든다.
길이로는 원리적으로 못 거른다. 팔이 물체 위에 있는 동안의 측정은 나쁜 측정이고
걸러낼 게 아니라 그때 재지 않아야 한다.


place 중 낙하를 감지할 수 있나

0. 정상 place를 먼저 돌린다. 되던 게 안 깨지는지
1. 쥔 채로 52초 두고 관절값을 기록한다. 왜 내려가는지
2. 쥔 블록을 강탈하고 place를 건다. 감지되는지

관측
정상 place는 완료됐다. 다만 TRANSFER의 관절값이 0.0619까지 내려와 있었다.
살짝 쥐었다고 볼 경계 바로 위다. 조금만 더 내려갔으면 빈 손으로 읽힌다.
쥔 채로 두면 관절값이 q = -0.00484 * t + 0.886 으로 내려간다. R^2 = 0.955.
그동안 블록은 lift 높이에 매달려 6.6mm 내려앉다가 52초째에 빠진다.
강탈하면 관절값이 0.0003이 되고 code 6, stage TRANSFER로 abort된다.

알게 된 것
관절값은 계단이 아니라 직선으로 내려온다. 손가락이 물체를 초당 0.0048씩 계속 파고든다.
그동안 블록은 실제로 쥐여 있다. 다 파고들면 빠진다.
15일차에 적은 파지 유지 41.5초는 이름이 틀렸다.
잰 것은 관통이 블록 두께를 먹는 데 걸린 시간이다.
이름이 틀리니 마찰을 의심하게 됐는데 실제 대책은 미는 힘을 제한하는 것이다.


agent, skill_server, moveit들이 실제로 이어지나

0. agent를 띄운다
1. /command에 deliver red_block bin을 던진다

관측
LLM이 pick, place 2스텝으로 펼치고 완주했다. 30.2초, 오차 5.1mm.

붙이는 데 코드가 한 줄도 안 들었다. 계약이 경계 역할을 했다는 뜻이다.
mock 검증 코드를 무수정으로 실 skill_server에 붙인 것에 이어 두 번째다.


낙하 후 스스로 다시 줍는가

과정
0. deliver를 건다
1. pick 완료 직후 쥔 블록을 강탈한다
2. 자동 복구를 관찰한다

관측
운반 중 파지 확인에서 관절값 0.0245로 빈 손 판정, code 6 / TRANSFER.
REGRASP 1/2로 복구 2스텝 삽입.
move_to home, pick 재시도, place 순으로 완주. 55.9초, 오차 6.9mm.

알게 된 것
계획이 [pick, place]에서 [pick, move_to, pick, place]로 부풀고 _step이 원래 자리로 돌아온다.
복구 전용 실행 경로를 안 만들었는데 복구가 됐다.
그리고 쥠으로 보는 경계를 더 낮추려던 판단이 틀렸다.
감지 시점 관절값이 0.0245였다. 경계를 더 내렸으면 이걸 쥠으로 읽어서 못 잡았다.
낙하 후 관절값이 다 닫히는 데 시간이 걸리기 때문이다.
경계를 낮추면 잘못 잡는 건 줄지만 못 잡는 게 는다. 공짜가 아니었다.

팔이 못 닿는 곳은 포기하는가

0. 블록을 팔이 못 닿는 (0.25, -0.05)에 둔다
1. deliver를 건다
2. 재시도를 하는지 본다

관측
code 2, stage APPROACH, 전략 ABORT, 시퀀스 중단.
goal 수락에서 ABORT까지 18ms. 팔이 한 번도 안 움직였고 재시도도 안 했다.

알게 된 것
도달 불가는 몇 번을 해도 도달 불가다. ABORT 판정이 재시도보다 먼저 걸린다.


물체가 사라지면 어떻게 되나

0. 블록을 카메라 밖으로 치운다. /scene_state가 objects: [] 가 된다
1. deliver를 건다
2. 복구 도중에 블록을 원위치로 되돌린다

관측
LLM 1차 계획이 씬에 없는 물체라고 거부됐다.
거부 사유를 붙여 다시 물으니 모델이 빈 배열을 냈고 그것도 거부되어 문자열 파서로 폴백했다.
pick은 code 1 / PLAN 으로 실패하고 전략 RESCAN.
되돌려 놓지 않으면 RESCAN 1/2, 2/2를 쓰고 소진되어 중단한다. 전체 6.3초.
되돌려 놓으면 move_to home 4.7초, pick 12.1초, place 15.4초로 완주한다. 32.3초, 오차 6.7mm.

알게 된 것
폴백 사다리 세 칸이 처음으로 끝까지 밟혔다. LLM, 재프롬프트, 문자열 파서.
프롬프트에 거부 규칙이 없는데도 재프롬프트에서는 모델이 스스로 빈 배열을 냈다.
1차 응답에서만 못 거부하는 것이었다. 다만 두 번 본 것이라 단정하면 안 된다.
복구 상한이 없으면 재인지해도 좌표가 그대로라 영원히 돈다. 2회에서 끊긴다.
RESCAN과 REGRASP는 같은 함수를 쓴다. 물러나서 다시 보는 게 둘 다 같은 동작이기 때문이다.


실험 영상

media/M5_0_agent_skill_server_moveit_연결.webm
media/M5_1_파지_인식_과정.webm
media/M5_2_낙하_후_스스로_다시_줍는가.webm
media/M5_3_물체가_사라지면_재인지_복구.webm
media/M5_4_물체가_사라지면_상한_소진.webm
media/M5_5_팔이_못_닿는_곳은_포기하는가.webm

여섯 개를 한 번에 돌리는 스크립트로 5분 11초 녹화하고 시나리오별로 잘랐다.
주입 타이밍은 사람이 못 맞춘다. 파지 실패는 8초, 재인지 복구는 4.6초 안에 넣어야 한다.
그래서 로그를 감시하다가 조건이 뜨면 블록을 옮기게 했다.
녹화에 로그가 안 담겨서 자막으로 얹었다.
스크립트가 시나리오 경계를 월클락으로 남기고, 영상 시작 시각은 파일 수정시각에서 길이를 빼서 구했다.
그 둘로 로그 시각을 영상 시각으로 옮겼다.

### 17일차

**하네스 구조**
모델 주변의 모든 비모델 코드. 모델을 믿을수 없는 외부 입력으로 두고 감싸는 껍데기
1. 도구 스키마 = 계약 : TOOLS(pick, place, move_to 등 필수 인자) + TARGET_NAMEs, MAX_STEP = 2 같은 계약을 정의
2. 형식 강제 : ollama format 토큰 마스킹이라 JSON 형식이 보장됨
3. 검증 계층 : _parse_and_validate -> _validate_step. 4cmd
4. 폴백 사다리 : LLM -> _retry_prompt 1회 -> 문자열 파서. _safe_call이 예외를 먹어 ollama가 죽어도 에이전트는 죽지 않음
5. 결정론 테스트 : call_llm 의존성 주입 -> FFKLLM으로 pytest 실행. ROS가 없어도 ollama도 없이 돈다.
6. 관측성 : 계획거부(1차)

**계획 하네스**
입력 : 자연어 명령 + scend_id - 복구 : FailureReport
출력 : 스텝 배열 - 복구 : 전략 라벨 1개 
1. 스키마 : 스킬(move_to, pick, place) - 복구 : RETRY, REGRASP, RESCAN, REPLAN, ABORT
3. 검증 : 씬 대조 등 - 복구 : 라벨 화이트 리스트 + 미등록시 거부
4. 폴백 : 문자열 파서 - 복구 : 지금 쓰는 STRATEGY dict
5 테스트 : 15 케이스 - 복구 실패 주입(동영상) 6종


**규칙**
REPLAN은 스키마에서 뺐다. _do_replan 현재 로그만 찍고 멈추고 있다..
REGRASP로 풀릴 상황이 있고 REPLAN이 나오면 그냥 멈춘다.

LLM은 라벨만 고른다.
리커버는 오직 코드로 한다. 실패한 스텝이 pick이면 move_to, place면 move_to, pick
LLM은 무엇을 할 지 결정하고 코드가 어떻게를 실행한다.


