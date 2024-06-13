#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>

#include <uORB/topics/my_task.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/actuator_armed.h>

using namespace time_literals;

class My_pwm_out: public ModuleBase<My_pwm_out>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	My_pwm_out();
	~My_pwm_out() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;
	void Press_PID(my_task_s &my_task_param);
private:
	void Run() override;
	void PID_init();
	float PID_realize(float ActualSpeed,float speed);//位置式

	my_task_s				 my_task{};
	distance_sensor_s 			distance_sensor{};
	input_rc_s				 input_rc{};
	vehicle_local_position_s		vehicle_local_position{};
	actuator_outputs_s			actuator_outputs{};
	sensor_baro_s				sensor_baro{};
	actuator_armed_s			actuator_armed{};
	hrt_abstime				_time_stamp_last_loop{0};
	// Publications
	uORB::Publication<my_task_s>			_my_task_pub{ORB_ID(my_task)};
	// Subscriptions
	uORB::Subscription 				_my_task_sub{ORB_ID(my_task)};
	uORB::Subscription 				_distance_sensor_sub{ORB_ID(distance_sensor)};
	uORB::Subscription				 _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription				 _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription 				_actuator_outputs_sub{ORB_ID(actuator_outputs)};
	uORB::Subscription 				_sensor_baro_sub{ORB_ID(sensor_baro),2};//腔体内气压
	uORB::Subscription 				_actuator_armed_sub{ORB_ID(actuator_armed)};

	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};        // subscription that schedules WorkItemExample when updated
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)


	bool _armed{false};
};
