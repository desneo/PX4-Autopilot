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
#include "Intercepe_Strike_Model_AS.h"
}

using namespace time_literals;

StrikeModeAPP::StrikeModeAPP() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": cycle");

	// ---- 查找打击模式参数 ----
	_param_bool_search        = param_find("STRIKE_SEARCH");
	_param_bool_request       = param_find("STRIKE_REQUEST");
	_param_strike_roll_kp     = param_find("STRIKE_ROLL_KP");
	_param_strike_roll_ki     = param_find("STRIKE_ROLL_KI");
	_param_strike_pitch_kp    = param_find("STRIKE_PITCH_KP");
	_param_strike_pitch_ki    = param_find("STRIKE_PITCH_KI");
	_param_strike_lowpass_hz  = param_find("STRIKE_LP_HZ");
	_param_param1             = param_find("STRIKE_P1");
	_param_param2             = param_find("STRIKE_P2");
	_param_param3             = param_find("STRIKE_P3");
	_param_param4             = param_find("STRIKE_P4");
	_param_param5             = param_find("STRIKE_P5");

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

	if (_param_strike_lowpass_hz == PARAM_INVALID) {
		PX4_WARN("param STRIKE_LP_HZ not found");
	}

	if (_param_param1 == PARAM_INVALID) {
		PX4_WARN("param STRIKE_P1  not found");
	}

	if (_param_param2 == PARAM_INVALID) {
		PX4_WARN("param STRIKE_P2  not found");
	}

	if (_param_param3 == PARAM_INVALID) {
		PX4_WARN("param STRIKE_P3  not found");
	}

	if (_param_param4 == PARAM_INVALID) {
		PX4_WARN("param STRIKE_P4  not found");
	}

	if (_param_param5 == PARAM_INVALID) {
		PX4_WARN("param STRIKE_P5  not found");
	}
}

StrikeModeAPP::~StrikeModeAPP()
{
	PX4_INFO("打击制导结束!");

	perf_free(_loop_perf);
	Intercepe_Strike_Model_AS_terminate();
}


bool StrikeModeAPP::init()
{
	PX4_INFO("打击制导初始化!");
	// 调度 Run() 以 10ms（100Hz）的间隔执行
    	ScheduleOnInterval(10000_us);

	Intercepe_Strike_Model_AS_initialize();
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


void StrikeModeAPP::Run()
{
	perf_begin(_loop_perf);
	_vehicle_status_sub.update(&_vehicle_status);
	_vehicle_attitude_sub.update(&_vehicle_attitude);
	_vehicle_local_position_sub.update(&_vehicle_local_position);
	_vehicle_thrust_sp_sub.update(&_vehicle_thrust_sp);
	_dyt_aisheng_telemetry_sub.update(&_dyt_aisheng);
	Intercepe_Strike_Model_AS_U.SeekerData.timestamp = _dyt_aisheng.timestamp;
	Intercepe_Strike_Model_AS_U.SeekerData.workStatus = _dyt_aisheng.work_status;
	Intercepe_Strike_Model_AS_U.SeekerData.selfCheckResult = _dyt_aisheng.self_check;
	Intercepe_Strike_Model_AS_U.SeekerData.trackConfidence = _dyt_aisheng.tracking_confidence;
	Intercepe_Strike_Model_AS_U.SeekerData.trackBoxWidth = _dyt_aisheng.tracking_gate_w;
	Intercepe_Strike_Model_AS_U.SeekerData.trackBoxHeight = _dyt_aisheng.tracking_gate_h;
	Intercepe_Strike_Model_AS_U.SeekerData.losRatePitch = _dyt_aisheng.pitch_los_rate;
	Intercepe_Strike_Model_AS_U.SeekerData.losRateYaw = _dyt_aisheng.yaw_los_rate;
	Intercepe_Strike_Model_AS_U.SeekerData.misalignPitch = _dyt_aisheng.pitch_misalignment;
	Intercepe_Strike_Model_AS_U.SeekerData.misalignYaw = _dyt_aisheng.yaw_misalignment;
	Intercepe_Strike_Model_AS_U.SeekerData.frameAnglePitch = _dyt_aisheng.pitch_angle;
	Intercepe_Strike_Model_AS_U.SeekerData.frameAngleYaw = _dyt_aisheng.yaw_angle;
	Intercepe_Strike_Model_AS_U.arming_state = _vehicle_status.arming_state;
	// ---- 2. 读取当前姿态（欧拉角）----
	float cur_roll, cur_pitch, cur_yaw;
	quat_to_euler(_vehicle_attitude.q, cur_roll, cur_pitch, cur_yaw);
	Intercepe_Strike_Model_AS_U.current_roll = cur_roll;
	Intercepe_Strike_Model_AS_U.current_pitch = cur_pitch;
	Intercepe_Strike_Model_AS_U.current_yaw = cur_yaw;
	Intercepe_Strike_Model_AS_U.current_thr = -_vehicle_thrust_sp.xyz[2];
	Intercepe_Strike_Model_AS_U.current_vx = _vehicle_local_position.vx;
	Intercepe_Strike_Model_AS_U.current_vy = _vehicle_local_position.vy;
	Intercepe_Strike_Model_AS_U.current_vz = _vehicle_local_position.vz;
	Intercepe_Strike_Model_AS_U.current_posn = _vehicle_local_position.x;
	Intercepe_Strike_Model_AS_U.current_pose = _vehicle_local_position.y;
	Intercepe_Strike_Model_AS_U.current_posd = _vehicle_local_position.z;
	Intercepe_Strike_Model_AS_U.current_vel_mag = sqrtf(_vehicle_local_position.vx * _vehicle_local_position.vx
                                                  + _vehicle_local_position.vy * _vehicle_local_position.vy
                                                  + _vehicle_local_position.vz * _vehicle_local_position.vz);
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
	if (_param_strike_lowpass_hz != PARAM_INVALID) {
		param_get(_param_strike_lowpass_hz, &_strike_lowpass_hz);
	}
	if (_param_param1 != PARAM_INVALID) {
		param_get(_param_param1, &_param1);
	}
	if (_param_param2 != PARAM_INVALID) {
		param_get(_param_param2, &_param2);
	}
	if (_param_param3 != PARAM_INVALID) {
		param_get(_param_param3, &_param3);
	}
	if (_param_param4 != PARAM_INVALID) {
		param_get(_param_param4, &_param4);
	}
	if (_param_param5 != PARAM_INVALID) {
		param_get(_param_param5, &_param5);
	}
	// ---- 打击模式开关 ----
	Intercepe_Strike_Model_AS_U.bool_search = (_bool_search != 0) ? 1 : 0;
	Intercepe_Strike_Model_AS_U.bool_request = (_bool_request != 0) ? 1 : 0;

	// -------- 把类成员拷贝给Simulink导出全局结构体 --------
	STRIKE_PARAM.STRIKE_ROLL_KP    = _strike_roll_kp;
	STRIKE_PARAM.STRIKE_ROLL_KI    = _strike_roll_ki;
	STRIKE_PARAM.STRIKE_PITCH_KP   = _strike_pitch_kp;
	STRIKE_PARAM.STRIKE_PITCH_KI   = _strike_pitch_ki;
	STRIKE_PARAM.STRIKE_LOWPASS_HZ = _strike_lowpass_hz;
	STRIKE_PARAM.param1            = _param1;
	STRIKE_PARAM.param2            = _param2;
	STRIKE_PARAM.param3            = _param3;
	STRIKE_PARAM.param4            = _param4;
	STRIKE_PARAM.param5            = _param5;
	Intercepe_Strike_Model_AS_step();

	// 日志记录
	strike_mode_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.timestamp_seekerdata = Intercepe_Strike_Model_AS_U.SeekerData.timestamp;
	status.self_check_result    = Intercepe_Strike_Model_AS_U.SeekerData.selfCheckResult;
	status.work_status          = Intercepe_Strike_Model_AS_U.SeekerData.workStatus;
	status.track_confidence     = Intercepe_Strike_Model_AS_U.SeekerData.trackConfidence;
	status.track_box_width      = Intercepe_Strike_Model_AS_U.SeekerData.trackBoxWidth;
	status.track_box_height     = Intercepe_Strike_Model_AS_U.SeekerData.trackBoxHeight;
	status.los_rate_pitch       = Intercepe_Strike_Model_AS_U.SeekerData.losRatePitch;
	status.los_rate_yaw         = Intercepe_Strike_Model_AS_U.SeekerData.losRateYaw;
	status.misalign_pitch       = Intercepe_Strike_Model_AS_U.SeekerData.misalignPitch;
	status.misalign_yaw         = Intercepe_Strike_Model_AS_U.SeekerData.misalignYaw;
	status.frame_angle_pitch    = Intercepe_Strike_Model_AS_U.SeekerData.frameAnglePitch;
	status.frame_angle_yaw      = Intercepe_Strike_Model_AS_U.SeekerData.frameAngleYaw;
	status.bool_search          = Intercepe_Strike_Model_AS_U.bool_search;
	status.bool_request         = Intercepe_Strike_Model_AS_U.bool_request;
	status.arming_state         = Intercepe_Strike_Model_AS_U.arming_state;
	status.current_roll         = Intercepe_Strike_Model_AS_U.current_roll;
	status.current_pitch        = Intercepe_Strike_Model_AS_U.current_pitch;
	status.current_yaw          = Intercepe_Strike_Model_AS_U.current_yaw;
	status.current_vx           = Intercepe_Strike_Model_AS_U.current_vx;
	status.current_vy           = Intercepe_Strike_Model_AS_U.current_vy;
	status.current_vz           = Intercepe_Strike_Model_AS_U.current_vz;
	status.current_posn         = Intercepe_Strike_Model_AS_U.current_posn;
	status.current_pose         = Intercepe_Strike_Model_AS_U.current_pose;
	status.current_posd         = Intercepe_Strike_Model_AS_U.current_posd;
	status.current_thr          = Intercepe_Strike_Model_AS_U.current_thr;
	status.current_vel_mag      = Intercepe_Strike_Model_AS_U.current_vel_mag;
	status.time_gate            = Intercepe_Strike_Model_AS_DW.time_gate;
	status.guided_state         = Intercepe_Strike_Model_AS_B.guided_state;
	status.cmd_timestamp        = Intercepe_Strike_Model_AS_Y.VehicleCommand.timestamp;
	status.cmd_command          = Intercepe_Strike_Model_AS_Y.VehicleCommand.command;
	status.cmd_param1           = Intercepe_Strike_Model_AS_Y.VehicleCommand.param1;
	status.cmd_param2           = Intercepe_Strike_Model_AS_Y.VehicleCommand.param2;
	status.cmd_param3           = Intercepe_Strike_Model_AS_Y.VehicleCommand.param3;
	status.cmd_param4           = Intercepe_Strike_Model_AS_Y.VehicleCommand.param4;
	status.cmd_param5           = Intercepe_Strike_Model_AS_Y.VehicleCommand.param5;
	status.cmd_param6           = Intercepe_Strike_Model_AS_Y.VehicleCommand.param6;
	status.cmd_param7           = Intercepe_Strike_Model_AS_Y.VehicleCommand.param7;
	status.cmd_valid            = Intercepe_Strike_Model_AS_Y.VehicleCommand.cmd_valid;
	_strike_mode_status_pub.publish(status);

	const VehicleCommandBus &sim_cmd = Intercepe_Strike_Model_AS_Y.VehicleCommand;

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
