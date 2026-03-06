from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

# 현재 스크립트 파일이 위치한 디렉토리의 절대 경로를 가져옵니다.
# URDF 파일 위치 등을 지정할 때 기준점이 됩니다.
TEMPLATE_ASSETS_DATA_DIR = Path(__file__).resolve().parent

##
# Configuration
##

# SO_ARM101_CFG는 로봇에 대한 모든 물리적, 구조적 설정을 담는 설정(Config) 객체입니다.
# 학습 환경에 이 로봇을 스폰(Spawn)할 때 이 설정값이 통째로 전달됩니다.
SO_ARM101_CFG = ArticulationCfg(
    # [1] Spawn(생성) 설정: 모델 파일(URDF) 로드 및 기본 물리 성질 세팅
    spawn=sim_utils.UrdfFileCfg(
        # 로봇의 베이스(바닥 면)를 시뮬레이션 환경의 바닥에 고정(Fix)할지 여부입니다.
        fix_base=True,
        # 충돌(Collision) 연산 속도와 안정성을 높이기 위해 실린더 매쉬를 캡슐 형태로 바꿉니다.
        replace_cylinders_with_capsules=True,
        # 로봇 모델 정보를 담은 URDF 파일의 경로 지정
        asset_path=f"{TEMPLATE_ASSETS_DATA_DIR}/urdf/so_arm101.urdf",
        # 충돌 센서 활성화 여부.
        activate_contact_sensors=False, # set as false while waiting for capsule implementation
        # 각 강체(Rigid Body)의 물리적 속성 설정
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            # 환경에 중력을 적용시킬 것인지 여부
            disable_gravity=False,
            # 물체가 서로 겹쳤을 때(Penetration) 튕겨져 나오는 최대 속도를 제한하여 시뮬레이션 폭발 방지
            max_depenetration_velocity=5.0,
        ),
        # 관절로 연결된 모델 전체(Articulation)의 최상위 속성 설정
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            # 로봇 관절끼리 서로 부딪히는 자가 충돌(Self-Collision) 연산 여부
            enabled_self_collisions=True,
            # 물리 엔진이 위치를 계산할 때 반복하는 횟수 (크면 정밀도 상승)
            solver_position_iteration_count=8,
            # 물리 엔진이 속도를 계산할 때 반복하는 횟수
            solver_velocity_iteration_count=0,
        ),
        # URDF 로드 시 기본 관절 구동기(Joint Drive) 세팅 (아래 actuators 설정에서 덮어쓰기 위해 0으로 초기화)
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    # [2] 초기 상태(Initial State) 설정: 에피소드 시작/초기화 시 로봇의 포즈
    init_state=ArticulationCfg.InitialStateCfg(
        # 베이스의 회전 상태 (Quaternion: W, X, Y, Z)
        rot=(1.0, 0.0, 0.0, 0.0),
        # 각 관절(Joint)이 가리키는 초기 각도(Radian)
        joint_pos={
            "shoulder_pan": 0.0,    # 어깨 좌우 회전
            "shoulder_lift": 0.0,   # 어깨 상하 이동
            "elbow_flex": -0.0,     # 팔꿈치 굽힘
            "wrist_flex": 1.57,     # 손목 굽힘 (약 90도 굽힌 상태로 초기화)
            "wrist_roll": -0.0,     # 손목 회전
            "gripper": 0.0,         # 그리퍼 초기 상태
        },
        # 초기 관절 속도를 전부 0(정지 상태)으로 설정
        joint_vel={".*": 0.0},
    ),
    # [3] Actuators(구동기, 즉 모터) 설정: 어떤 제어 모델을 사용할지, 관절별 PD 게인(Gain)과 힘계산
    actuators={
        # 아래 주석은 각 관절이 감당해야 하는 로봇 부위의 무게입니다.
        # 무거운 부위를 통째로 움직여야 하는 모터일수록 더 큰 힘(Stiffness)이 필요합니다.
        # Shoulder Pan      moves: ALL masses                   (~0.8kg total)
        # Shoulder Lift     moves: Everything except base       (~0.65kg)
        # Elbow             moves: Lower arm, wrist, gripper    (~0.38kg)
        # Wrist Pitch       moves: Wrist and gripper            (~0.24kg)
        # Wrist Roll        moves: Gripper assembly             (~0.14kg)
        # Jaw               moves: Only moving jaw              (~0.034kg)
        "arm": ImplicitActuatorCfg(
            # 정규표현식으로 팔뚝 역할을 하는 관절들(어깨, 팔꿈치, 손목)을 묶어서 "arm" 그룹으로 지정
            joint_names_expr=["shoulder_.*", "elbow_flex", "wrist_.*"],
            # 모터가 낼 수 있는 최대 토크(Torque) 제한
            effort_limit_sim=1.9,
            # 모터의 최대 관절 회전 속도 제한
            velocity_limit_sim=1.5,
            # PD 제어기의 P(Proportional) 게인에 해당하는 강성(Stiffness)
            stiffness={
                "shoulder_pan": 200.0,   # 가장 무거운 전체를 들어야 하므로 제일 큼
                "shoulder_lift": 170.0,  # 그 다음으로 무거움
                "elbow_flex": 120.0,     # 감당 하중이 적어질수록 값이 감소
                "wrist_flex": 80.0,  
                "wrist_roll": 50.0,      # 가장 끝단이라 제일 가벼움
            },
            # PD 제어기의 D(Derivative) 게인에 해당하는 감쇠(Damping) - 진동 방지용
            damping={
                "shoulder_pan": 80.0,
                "shoulder_lift": 65.0,
                "elbow_flex": 45.0,
                "wrist_flex": 30.0,
                "wrist_roll": 20.0,
            },
        ),
        # 그리퍼(Gripper) 전용 액추에이터 그룹 설정
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["gripper"],
            # 물건을 꽉 쥐기 위해 arm 부분보다 토크 한계를 더 높임 (강한 쥐기 힘)
            effort_limit_sim=2.5,  # Increased from 1.9 to 2.5 for stronger grip
            velocity_limit_sim=1.5,
            # 그리퍼가 안정적으로 닫히도록 Stiffness를 arm 말단부보다 높게 세팅
            stiffness=60.0,  # Increased from 25.0 to 60.0 for more reliable closing
            # 덜덜 떨리는 것을 막기 위한 댐핑 증가
            damping=20.0,  # Increased from 10.0 to 20.0 for stability
        ),
    },
    # 관절의 물리적 최대/최소 회전 한계를 100% 다 쓰지 않고 90%(0.9) 지점에서 소프트하게 제한 걸기
    soft_joint_pos_limit_factor=0.9,
)
