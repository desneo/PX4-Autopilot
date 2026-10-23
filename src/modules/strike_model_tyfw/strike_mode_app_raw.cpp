/****************************************************************************
 * @file strike_mode_app.cpp
 *打击飞行管理和制导控制率模块
 *
 * @author grj <488596609@qq.com>
 ****************************************************************************/
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <parameters/param.h>
#include"strike_mode_app.hpp"
extern "C" {
#include "Intercepe_Strike_Model_Without_Rader.h"
}

using namespace time_literals;

StrikeModeAPP::StrikeModeAPP() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": cycle");

	// ---- 查找打击模式参数 ----
	_param_bool_search = param_find("STRIKE_SEARCH");
	_param_bool_request = param_find("STRIKE_REQUEST");
	_param_strike_roll_kp     = param_find("STRIKE_ROLL_KP");
	_param_strike_roll_ki     = param_find("STRIKE_ROLL_KI");
	_param_strike_pitch_kp    = param_find("STRIKE_PITCH_KP");
	_param_strike_pitch_ki    = param_find("STRIKE_PITCH_KI");

	if (_param_bool_search == PARAM_INVALID) {
		PX4_WARN("param STRIKE_SEARCH not found");
	}

	if (_param_bool_request == PARAM_INVALID) {
		PX4_WARN("param STRIKE_REQUEST not found");
	}

	if (_param_strike_roll_kp == PARAM_INVALID) {
		PX4_WARN("param STRIKE_ROLL_KP not found");
	}

	if (_param_strike_roll_ki == PARAM_INVALID) {
		PX4_WARN("param STRIKE_ROLL_KI not found");
	}

	if (_param_strike_pitch_kp == PARAM_INVALID) {
		PX4_WARN("param STRIKE_PITCH_KP not found");
	}

	if (_param_strike_pitch_ki == PARAM_INVALID) {
		PX4_WARN("param STRIKE_PITCH_KI not found");
	}

}

StrikeModeAPP::~StrikeModeAPP()
{
	PX4_INFO("打击制导结束!");

	perf_free(_loop_perf);
	Intercepe_Strike_Model_Without_Rader_terminate();
}


bool StrikeModeAPP::init()
{
	PX4_INFO("打击制导初始化!");
	// 调度 Run() 以 10ms（100Hz）的间隔执行
	ScheduleOnInterval(10000_us);

	Intercepe_Strike_Model_Without_Rader_initialize();
	return true;
}

// ---- 欧拉角 → 四元数 (w, x, y, z)，输入弧度 ----
static void euler_to_quat(float roll, float pitch, float yaw, float q[4])
{
	const float cr = cosf(roll  * 0.5f);
	const float sr = sinf(roll  * 0.5f);
	const float cp = cosf(pitch * 0.5f);
	const float sp = sinf(pitch * 0.5f);
	const float cy = cosf(yaw   * 0.5f);
	const float sy = sinf(yaw   * 0.5f);

	q[0] = cr * cp * cy + sr * sp * sy;   // w
	q[1] = sr * cp * cy - cr * sp * sy;   // x
	q[2] = cr * sp * cy + sr * cp * sy;   // y
	q[3] = cr * cp * sy - sr * sp * cy;   // z
}

// ---- 四元数 → 欧拉角 (roll, pitch, yaw)，输出弧度 ----
static void quat_to_euler(const float q[4], float &roll, float &pitch, float &yaw)
{
	const float w = q[0];
	const float x = q[1];
	const float y = q[2];
	const float z = q[3];

	// roll (x-axis rotation)
	const float sinr_cosp = 2.0f * (w * x + y * z);
	const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
	roll = atan2f(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	const float sinp = 2.0f * (w * y - z * x);

	if (fabsf(sinp) >= 1.0f) {
		pitch = copysignf(M_PI_F / 2.0f, sinp);   // 万向锁

	} else {
		pitch = asinf(sinp);
	}

	// yaw (z-axis rotation)
	const float siny_cosp = 2.0f * (w * z + x * y);
	const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
	yaw = atan2f(siny_cosp, cosy_cosp);
}

// NED速度 vn,ve,vd 转换到机体坐标系 vb[0]=机体X(前), vb[1]=机体Y(右), vb[2]=机体Z(下)
static void ned_vel_to_body_vel(const float q[4], float vn, float ve, float vd, float vb[3])
{
	const float w = q[0];
	const float x = q[1];
	const float y = q[2];
	const float z = q[3];

	// 方向余弦矩阵 R_nb : NED -> Body
	const float r00 = w*w + x*x - y*y - z*z;
	const float r01 = 2.f*(x*y + w*z);
	const float r02 = 2.f*(x*z - w*y);

	const float r10 = 2.f*(x*y - w*z);
	const float r11 = w*w - x*x + y*y - z*z;
	const float r12 = 2.f*(y*z + w*x);

	const float r20 = 2.f*(x*z + w*y);
	const float r21 = 2.f*(y*z - w*x);
	const float r22 = w*w - x*x - y*y + z*z;

	vb[0] = r00 * vn + r01 * ve + r02 * vd;
	vb[1] = r10 * vn + r11 * ve + r12 * vd;
	vb[2] = r20 * vn + r21 * ve + r22 * vd;
}

void StrikeModeAPP::Run()
{
	perf_begin(_loop_perf);
	_vehicle_status_sub.update(&_vehicle_status);
	_vision_detection_sub.update(&_vision_detection);
	_vehicle_attitude_sub.update(&_vehicle_attitude);
	_vehicle_local_position_sub.update(&_vehicle_local_position);
	_vehicle_thrust_sp_sub.update(&_vehicle_thrust_sp);

	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime age = now - _vision_detection.timestamp;

	// 数据新鲜（100ms 内）→ 更新 SeekerData
	if (age < 100_ms) {
		Intercepe_Strike_Model_Withou_U.SeekerData.timestamp = now;
		Intercepe_Strike_Model_Withou_U.SeekerData.target_x = _vision_detection.x;
		Intercepe_Strike_Model_Withou_U.SeekerData.target_y = _vision_detection.y;
		Intercepe_Strike_Model_Withou_U.SeekerData.target_width = _vision_detection.w;
		Intercepe_Strike_Model_Withou_U.SeekerData.target_height = _vision_detection.h;

	} else {
		// 数据超时 → 标记目标无效
		Intercepe_Strike_Model_Withou_U.SeekerData.timestamp = now;
		Intercepe_Strike_Model_Withou_U.SeekerData.target_x = -1;
		Intercepe_Strike_Model_Withou_U.SeekerData.target_y = -1;
		Intercepe_Strike_Model_Withou_U.SeekerData.target_width = -1;
		Intercepe_Strike_Model_Withou_U.SeekerData.target_height = -1;
	}

	Intercepe_Strike_Model_Withou_U.arming_state = _vehicle_status.arming_state;
	// ---- 2. 读取当前姿态（欧拉角）----
	float cur_roll, cur_pitch, cur_yaw;
	quat_to_euler(_vehicle_attitude.q, cur_roll, cur_pitch, cur_yaw);
	Intercepe_Strike_Model_Withou_U.current_roll = cur_roll;
	Intercepe_Strike_Model_Withou_U.current_pitch = cur_pitch;
	Intercepe_Strike_Model_Withou_U.current_yaw = cur_yaw;
	Intercepe_Strike_Model_Withou_U.current_thr = -_vehicle_thrust_sp.xyz[2];
	// 读取NED坐标系速度
	float vn = _vehicle_local_position.vx;
	float ve = _vehicle_local_position.vy;
	float vd = _vehicle_local_position.vz;

	float vb[3];
	ned_vel_to_body_vel(_vehicle_attitude.q, vn, ve, vd, vb);

	// vb[2] 机体Z轴速度（机体向下为正）
	Intercepe_Strike_Model_Withou_U.current_vz = vb[2];

	// ---- 读取打击模式参数 ----
	if (_param_bool_search != PARAM_INVALID) {
		param_get(_param_bool_search, &_bool_search);
	}

	if (_param_bool_request != PARAM_INVALID) {
		param_get(_param_bool_request, &_bool_request);
	}

	if (_param_strike_roll_kp != PARAM_INVALID) {
		param_get(_param_strike_roll_kp, &_strike_roll_kp);
	}
	if (_param_strike_roll_ki != PARAM_INVALID) {
		param_get(_param_strike_roll_ki, &_strike_roll_ki);
	}
	if (_param_strike_pitch_kp != PARAM_INVALID) {
		param_get(_param_strike_pitch_kp, &_strike_pitch_kp);
	}
	if (_param_strike_pitch_ki != PARAM_INVALID) {
		param_get(_param_strike_pitch_ki, &_strike_pitch_ki);
	}

	// ---- 打击模式开关 ----
	Intercepe_Strike_Model_Withou_U.bool_search = (_bool_search != 0) ? 1 : 0;
	Intercepe_Strike_Model_Withou_U.bool_request = (_bool_request != 0) ? 1 : 0;

	// -------- 把类成员拷贝给Simulink导出全局结构体 --------
	STRIKE_PARAM.STRIKE_ROLL_KP    = _strike_roll_kp;
	STRIKE_PARAM.STRIKE_ROLL_KI    = _strike_roll_ki;
	STRIKE_PARAM.STRIKE_PITCH_KP   = _strike_pitch_kp;
	STRIKE_PARAM.STRIKE_PITCH_KI   = _strike_pitch_ki;

	Intercepe_Strike_Model_Without_Rader_step();
	strike_mode_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.arming_state = Intercepe_Strike_Model_Withou_U.arming_state;

	// ---- SeekerData.timestamp 是 uint64_T，直接赋值 ----
	status.timestamp_seekerdata = Intercepe_Strike_Model_Withou_U.SeekerData.timestamp;
	status.bool_search = Intercepe_Strike_Model_Withou_U.bool_search;
	status.bool_request = Intercepe_Strike_Model_Withou_U.bool_request;
	status.target_x = Intercepe_Strike_Model_Withou_U.SeekerData.target_x;
	status.target_y = Intercepe_Strike_Model_Withou_U.SeekerData.target_y;
	status.target_width = Intercepe_Strike_Model_Withou_U.SeekerData.target_width;
	status.target_height = Intercepe_Strike_Model_Withou_U.SeekerData.target_height;
	status.guided_state = Intercepe_Strike_Model_Withou_B.guided_state;
	status.time_gate = Intercepe_Strike_Model_Witho_DW.time_gate;
	_strike_mode_status_pub.publish(status);

	const VehicleCommandBus &sim_cmd = Intercepe_Strike_Model_Withou_Y.VehicleCommand;

	// ==================== 发布 attitude + thrust setpoint ====================
	// guided_state == 3/4/5：进入 Offboard，使用姿态四元数 + 归一化推力
	if (status.guided_state == 3|| status.guided_state == 5)
	{
		// ---- 1. 发布 OffboardControlMode：只开 attitude ----
		offboard_control_mode_s ocm{};
		ocm.timestamp    = hrt_absolute_time();
		ocm.position     = false;
		ocm.velocity     = false;
		ocm.acceleration = false;
		ocm.attitude     = true;    // 关键：只用姿态层
		ocm.body_rate    = false;
		_offboard_mode_pub.publish(ocm);

		// ---- 2. 姿态角 → 四元数 ----
		// 注意：如果 Simulink 输出是角度而不是弧度，需要先 deg2rad
		float roll  = sim_cmd.param2;
		float pitch = sim_cmd.param3;
		float yaw   = sim_cmd.param4;

		float q[4];
		euler_to_quat(roll, pitch, yaw, q);

		// ---- 3. 归一化推力，限幅 ----
		float thrust = sim_cmd.param5;

		// ---- 4. 发布 vehicle_attitude_setpoint ----
		vehicle_attitude_setpoint_s att_sp{};
		att_sp.timestamp        = hrt_absolute_time();
		att_sp.q_d[0]           = q[0];
		att_sp.q_d[1]           = q[1];
		att_sp.q_d[2]           = q[2];
		att_sp.q_d[3]           = q[3];
		att_sp.thrust_body[0]   = 0.0f;
		att_sp.thrust_body[1]   = 0.0f;
		att_sp.thrust_body[2]   = -thrust;   // NED 下向上为负
		att_sp.yaw_sp_move_rate = NAN;
		_att_sp_pub.publish(att_sp);
	}

	// ==================== 发布 trajectory_setpoint_s setpoint ====================
	// guided_state == 4：进入 Offboard，使用三维位置 + 偏航角速率
	if (status.guided_state == 4)
	{
		offboard_control_mode_s ocm{};
		ocm.timestamp    = hrt_absolute_time();
		ocm.position     = true;     // 使用位置控制
		ocm.velocity     = false;
		ocm.acceleration = false;
		ocm.attitude     = false;    // 不再使用姿态
		ocm.body_rate    = false;
		_offboard_mode_pub.publish(ocm);

		float pos_x = sim_cmd.param3;
		float pos_y = sim_cmd.param4;
		float pos_z = sim_cmd.param5;
		float yawspeed = sim_cmd.param2;  // 期望偏航角速率

		trajectory_setpoint_s traj_sp{};
		traj_sp.timestamp = hrt_absolute_time();

		// 三维位置 NED
		traj_sp.position[0] = pos_x;
		traj_sp.position[1] = pos_y;
		traj_sp.position[2] = pos_z;

		// 未控制的速度和加速度设为 NAN
		traj_sp.velocity[0] = NAN;
		traj_sp.velocity[1] = NAN;
		traj_sp.velocity[2] = NAN;
		traj_sp.acceleration[0] = NAN;
		traj_sp.acceleration[1] = NAN;
		traj_sp.acceleration[2] = NAN;

		// 偏航角设为 NAN（仅控制偏航速率），偏航速率填入 yawspeed
		traj_sp.yaw      = NAN;
		traj_sp.yawspeed = yawspeed;

		_trajectory_sp_pub.publish(traj_sp);
	}

	// guided_state == 1/2/6：发 Simulink 给的命令
	if (sim_cmd.cmd_valid && (status.guided_state == 1 || status.guided_state == 2))
	{
		vehicle_command_s cmd{};
		cmd.timestamp = sim_cmd.timestamp;
		cmd.command   = sim_cmd.command;
		cmd.param1    = sim_cmd.param1;
		cmd.param2    = sim_cmd.param2;
		cmd.param3    = sim_cmd.param3;
		cmd.param4    = sim_cmd.param4;
		cmd.param5    = sim_cmd.param5;
		cmd.param6    = sim_cmd.param6;
		cmd.param7    = sim_cmd.param7;

		cmd.target_system    = _vehicle_status.system_id;
		cmd.target_component = _vehicle_status.component_id;
		cmd.source_system    = _vehicle_status.system_id;
		cmd.source_component = _vehicle_status.component_id;
		cmd.from_external    = false;
		cmd.confirmation     = 0;

		_vehicle_command_pub.publish(cmd);

		PX4_INFO("send VEHICLE_CMD_SET_NAV_STATE and Guided_State, command=%lu, param1=%.0f, guided_state=%lu",
			(unsigned long)cmd.command,
			(double)cmd.param1,
			(unsigned long)status.guided_state);
	}

	// guided_state == 3/4/5：进入 Offboard 模式
	if (sim_cmd.cmd_valid && (status.guided_state == 3 || status.guided_state == 4 || status.guided_state == 5))
	{
		vehicle_command_s cmd{};
		cmd.timestamp = hrt_absolute_time();

		// 标准 PX4 切 Offboard 的方式
		cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
		cmd.param1  = 1;   // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
		cmd.param2  = 6;   // PX4_CUSTOM_MAIN_MODE_OFFBOARD
		cmd.param3  = 0;
		cmd.param4  = 0;
		cmd.param5  = 0;
		cmd.param6  = 0;
		cmd.param7  = 0;

		cmd.target_system    = _vehicle_status.system_id;
		cmd.target_component = _vehicle_status.component_id;
		cmd.source_system    = _vehicle_status.system_id;
		cmd.source_component = _vehicle_status.component_id;
		cmd.from_external    = false;
		cmd.confirmation     = 0;

		_vehicle_command_pub.publish(cmd);

		PX4_INFO("send OFFBOARD, guided_state=%lu",
			(unsigned long)status.guided_state);
	}
	perf_end(_loop_perf);
}

int StrikeModeAPP::task_spawn(int argc, char *argv[])
{
	StrikeModeAPP *instance = new StrikeModeAPP();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int StrikeModeAPP::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int StrikeModeAPP::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("strkemode", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int strike_mode_tyfw_main(int argc, char *argv[])
{
	return StrikeModeAPP::main(argc, argv);
}
