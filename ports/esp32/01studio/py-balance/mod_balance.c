#include "py/obj.h"
#include "mod_balance.h"
#include "system_init.h"
#include "sensors_qmi8658.h"
 #include "pid.h"
#include "py/runtime.h"
#include "battery.h"
#include "hcsr04.h"
#include "control.h"
#include "param_storage.h"
typedef struct _balance_obj_t{
    mp_obj_base_t base;

}balance_obj_t;

static float pit = 0;
static float yaw = 0;

static bool is_init = false; 

static int speed_level = 0;

static void initBlance(void)
{
    if(is_init) return;
    systemInit();

}

static mp_obj_t read_cal_data(mp_obj_t self_in)
{
    mp_obj_t tuple[2];
    qmi8658Calibration();
    tuple[0] = mp_obj_new_int(qmi8658CalData.middleAngle * 10);
    tuple[1] = mp_obj_new_int(qmi8658CalData.gyroOffset * 10);
	return mp_obj_new_tuple(2, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_1(read_cal_data_obj, read_cal_data);

static mp_obj_t cal_states()
{
    bool states = getCalibrated();
    return mp_obj_new_bool(states);
}
static MP_DEFINE_CONST_FUN_OBJ_1(cal_states_obj, cal_states);

static mp_obj_t confirm_cal()
{
    if (confirmSensorCalibration()) return mp_const_true;

    return mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_1(confirm_cal_obj, confirm_cal);

extern balance_pid_t anglePid;
extern balance_pid_t speedPid;
extern balance_pid_t turnPid;
//angle   leftpwm righpwm   nm
static mp_obj_t read_states(mp_obj_t self_in)
{
    mp_obj_t tuple[8];
    tuple[0] = mp_obj_new_int(balCtrl.angle.fusion * 10);
    tuple[1] = mp_obj_new_int(balCtrl.speed.showSpeed);
    tuple[2] = mp_obj_new_int(ultrasonicGetDistance());
    tuple[3] = mp_obj_new_int(analogReadVoltage() * 100.0f);
    tuple[4] = mp_obj_new_int(balCtrl.runFlag);
    tuple[5] = mp_obj_new_int(speed_level);
    tuple[6] = mp_obj_new_int(pit);
    tuple[7] = mp_obj_new_int(yaw);

	return mp_obj_new_tuple(8, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_1(read_states_obj, read_states);

static mp_obj_t start(mp_obj_t self_in)
{

    // Reset PID states
    pidInit(&anglePid);
    pidInit(&speedPid);
    pidInit(&turnPid);

    // Reset control ticks
    balCtrl.angleTick = 0;
    balCtrl.speedTick = 0;

    // Reset speed states
    balCtrl.speed.leftSpeed = 0;
    balCtrl.speed.rightSpeed = 0;
    balCtrl.speed.aveSpeed = 0;
    balCtrl.speed.difSpeed = 0;
    balCtrl.speed.showSpeed = 0;

    // Reset motor states
    balCtrl.motor.leftPwm = 0;
    balCtrl.motor.rightPwm = 0;
    balCtrl.motor.avePwm = 0;
    balCtrl.motor.difPwm = 0;
    
    balCtrl.runFlag = true;

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(start_obj, start);

static mp_obj_t stop(mp_obj_t self_in)
{
    balCtrl.runFlag = false;

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(stop_obj, stop);

static mp_obj_t speed(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t speed_args[] = {
        { MP_QSTR_value, MP_ARG_INT, {.u_int = 4} }
    };
    
    mp_arg_val_t args[MP_ARRAY_SIZE(speed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(speed_args), speed_args, args);

    mp_int_t target_speed = args[0].u_int;

    if (target_speed < 1 || target_speed > 5) {
        mp_raise_ValueError(MP_ERROR_TEXT("speed must be between 1 and 5"));
    }

    speed_level = target_speed;

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(speed_obj, 0, speed);


static mp_obj_t balance_control(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t balance_move_args[] = {
		{ MP_QSTR_pit, MP_ARG_INT, {.u_int = 0} },
		{ MP_QSTR_yaw,	MP_ARG_INT, {.u_int = 0} },


    };
    mp_arg_val_t args[MP_ARRAY_SIZE(balance_move_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(balance_move_args), balance_move_args, args);

    float speed = 0,turn = 0;

    pit = args[0].u_int;
    speed = ((float)(args[0].u_int)/ 100 * speed_level * 1.3);
    speedPid.Target = speed;


    yaw = args[1].u_int;
    turn = ((float)(args[1].u_int)/100 * speed_level * 1.3 );
    turnPid.Target = turn;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(balance_control_obj, 1, balance_control);

// static mp_obj_t set_angle_pid(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
// {
//     static const mp_arg_t angle_pid_args[] = {
// 		{ MP_QSTR_p, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
// 		{ MP_QSTR_i, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//         { MP_QSTR_d, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//         { MP_QSTR_max, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//         { MP_QSTR_min, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//         { MP_QSTR_offset, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//         { MP_QSTR_eimax, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//         { MP_QSTR_eimin, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//     };
//     mp_arg_val_t args[MP_ARRAY_SIZE(angle_pid_args)];
//     mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(angle_pid_args), angle_pid_args, args);
    
//     anglePid.Kp =  mp_obj_get_float(args[0].u_obj);
//     anglePid.Ki = mp_obj_get_float(args[1].u_obj);
//     anglePid.Kd = mp_obj_get_float(args[2].u_obj);
//     anglePid.OutMax = mp_obj_get_float(args[3].u_obj);
//     anglePid.OutMin = mp_obj_get_float(args[4].u_obj);
//     anglePid.OutOffset = mp_obj_get_float(args[5].u_obj);
//     anglePid.ErrorIntMax = mp_obj_get_float(args[6].u_obj);
//     anglePid.ErrorIntMin= mp_obj_get_float(args[7].u_obj);

//     printf("%f %f %f %f %f %f %f %f \n",anglePid.Kp, anglePid.Ki, anglePid.Kd,
//                                         anglePid.OutMax,anglePid.OutMin,
//                                         anglePid.OutOffset,
//                                         anglePid.ErrorIntMax, anglePid.ErrorIntMin);

//     return mp_const_none;
// }
// static MP_DEFINE_CONST_FUN_OBJ_KW(set_angle_pid_obj, 1, set_angle_pid);

// static mp_obj_t set_speed_pid(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
// {
//     static const mp_arg_t speed_pid_args[] = {
// 		{ MP_QSTR_p, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
// 		{ MP_QSTR_i, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//         { MP_QSTR_d, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//         { MP_QSTR_max, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//         { MP_QSTR_min, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//         { MP_QSTR_eimax, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//         { MP_QSTR_eimin, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//         { MP_QSTR_offset, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//     };
//     mp_arg_val_t args[MP_ARRAY_SIZE(speed_pid_args)];
//     mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(speed_pid_args), speed_pid_args, args);
    
//     speedPid.Kp =  mp_obj_get_float(args[0].u_obj);
//     speedPid.Ki = mp_obj_get_float(args[1].u_obj);
//     speedPid.Kd = mp_obj_get_float(args[2].u_obj);
//     speedPid.OutMax = mp_obj_get_float(args[3].u_obj);
//     speedPid.OutMin = mp_obj_get_float(args[4].u_obj);
//     speedPid.ErrorIntMax = mp_obj_get_float(args[5].u_obj);
//     speedPid.ErrorIntMin = mp_obj_get_float(args[6].u_obj);
//     speedPid.OutOffset = mp_obj_get_float(args[7].u_obj);

//     printf("%f %f %f %f %f %f %f %f\n",speedPid.Kp, speedPid.Ki, speedPid.Kd,
//                                     speedPid.OutMax, speedPid.OutMin,
//                                     speedPid.ErrorIntMax, speedPid.ErrorIntMin,
//                                     speedPid.OutOffset );

//     return mp_const_none;
// }
// static MP_DEFINE_CONST_FUN_OBJ_KW(set_speed_pid_obj, 1, set_speed_pid);

// static mp_obj_t set_turn_pid(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
// {
//     static const mp_arg_t turn_pid_args[] = {
// 		{ MP_QSTR_p, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
// 		{ MP_QSTR_i, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//         { MP_QSTR_d, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//         { MP_QSTR_max, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//         { MP_QSTR_min, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//         { MP_QSTR_eimax, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//         { MP_QSTR_eimin, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
//     };
//     mp_arg_val_t args[MP_ARRAY_SIZE(turn_pid_args)];
//     mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(turn_pid_args), turn_pid_args, args);
    
//     turnPid.Kp =  mp_obj_get_float(args[0].u_obj);
//     turnPid.Ki = mp_obj_get_float(args[1].u_obj);
//     turnPid.Kd = mp_obj_get_float(args[2].u_obj);
//     turnPid.OutMax = mp_obj_get_float(args[3].u_obj);
//     turnPid.OutMin= mp_obj_get_float(args[4].u_obj);
//     turnPid.ErrorIntMax = mp_obj_get_float(args[5].u_obj);
//     turnPid.ErrorIntMin = mp_obj_get_float(args[6].u_obj);


//     printf("%f %f %f %f %f %f %f\n",turnPid.Kp, turnPid.Ki, turnPid.Kp,
//                                     turnPid.OutMax, turnPid.OutMin,
//                                     turnPid.ErrorIntMax, turnPid.ErrorIntMin);

//     return mp_const_none;
// }
// static MP_DEFINE_CONST_FUN_OBJ_KW(set_turn_pid_obj, 1, set_turn_pid);

// static mp_obj_t read_pid_debug(mp_obj_t self_in)
// {
//     mp_obj_t tuple[8];
//     tuple[0] = mp_obj_new_float(anglePid.Error1);
//     tuple[1] = mp_obj_new_float(anglePid.Error0);
//     tuple[2] = mp_obj_new_float(anglePid.ErrorInt);
//     tuple[3] = mp_obj_new_float(anglePid.Out);
//     tuple[4] = mp_obj_new_int(balCtrl.motor.leftPwm);
//     tuple[5] = mp_obj_new_int(balCtrl.motor.rightPwm);
//     tuple[6] = mp_obj_new_float(balCtrl.speed.aveSpeed);
//     tuple[7] = mp_obj_new_float(speedPid.ErrorInt);  

// 	return mp_obj_new_tuple(8, tuple);
// }
// static MP_DEFINE_CONST_FUN_OBJ_1(read_pid_debug_obj, read_pid_debug);



static mp_obj_t balance_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args)
{

	
    speed_level = 4;

    initBlance();

    balance_obj_t *balance_type;
    balance_type = m_new_obj(balance_obj_t);
    balance_type->base.type = &balance_balance_type;
    

    return MP_OBJ_FROM_PTR(balance_type);
}


static const mp_rom_map_elem_t balance_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR__name__), MP_ROM_QSTR(MP_QSTR_balance) },
    { MP_ROM_QSTR(MP_QSTR_read_cal_data), MP_ROM_PTR(&read_cal_data_obj) },
    { MP_ROM_QSTR(MP_QSTR_confirm_cal), MP_ROM_PTR(&confirm_cal_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_states), MP_ROM_PTR(&read_states_obj) },
    { MP_ROM_QSTR(MP_QSTR_cal_states), MP_ROM_PTR(&cal_states_obj) },
    { MP_ROM_QSTR(MP_QSTR_control), MP_ROM_PTR(&balance_control_obj) },
    { MP_ROM_QSTR(MP_QSTR_start), MP_ROM_PTR(&start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_speed), MP_ROM_PTR(&speed_obj)},

    // { MP_ROM_QSTR(MP_QSTR_set_angle_pid), MP_ROM_PTR(&set_angle_pid_obj) },
    // { MP_ROM_QSTR(MP_QSTR_set_speed_pid), MP_ROM_PTR(&set_speed_pid_obj) },
    // { MP_ROM_QSTR(MP_QSTR_set_turn_pid), MP_ROM_PTR(&set_turn_pid_obj) },

    // { MP_ROM_QSTR(MP_QSTR_read_pid_debug), MP_ROM_PTR(&read_pid_debug_obj) },
};

static MP_DEFINE_CONST_DICT(balance_balance_locals_dict, balance_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    balance_balance_type,
    MP_QSTR_balance,
    MP_TYPE_FLAG_NONE,
    make_new, balance_make_new,
    locals_dict, &balance_balance_locals_dict
);
