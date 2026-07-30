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

cos의 해가 없다. 이걸 어떻게 전달하나? ErrorCode.UNRECHABLE 일 경우 arm_kinematics가 arm_interface에 의존하게 된다.
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


