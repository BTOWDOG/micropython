#include "param_storage.h"
#include "sensors_qmi8658.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "py/runtime.h"
#include "py/stream.h"
#include "extmod/vfs.h"
#include "py/mpprint.h"
#include "py/mphal.h"


#define CONFIG_FILE "/cal_data.txt"

static bool isInit = false;
static bool isCalibrated = false;


/*
 * 打开 MicroPython VFS 文件
 *
 * mode:
 *   "r" 只读，文件必须存在
 *   "w" 写入，不存在自动创建，存在则清空
 */
static mp_obj_t openVfsFile(const char *path, const char *mode)
{
    mp_obj_t args[2] = {
        mp_obj_new_str(path, strlen(path)),
        mp_obj_new_str(mode, strlen(mode)),
    };

    return mp_vfs_open(
        2,
        args,
        (mp_map_t *)&mp_const_empty_map
    );
}


/*
 * 初始化参数存储
 *
 * 注意：
 * 必须在 MicroPython VFS 已经挂载以后调用。
 */
void storageInit(void)
{
    if (isInit)
    {
        return;
    }

    if (loadConfig())
    {
        isCalibrated = true;
        
    }
    else
    {
        isCalibrated = false;
        qmi8658CalData.middleAngle = 0;
        qmi8658CalData.gyroOffset = 0;
        mp_hal_stdout_tx_str("WARNING: QMI8658 not calibrated!\n");
    }

    isInit = true;
}


/*
 * 保存校准参数
 *
 * 文件不存在：
 *      自动创建
 *
 * 文件存在：
 *      清空并重新写入
 */
bool saveConfig(void)
{
    char buffer[128];

    int len = snprintf(
        buffer,
        sizeof(buffer),
        "middleAngle=%.6f\n"
        "gyroOffset=%.6f\n",
        qmi8658CalData.middleAngle,
        qmi8658CalData.gyroOffset
    );

    if (len <= 0 || len >= sizeof(buffer))
    {
        mp_hal_stdout_tx_str("saveConfig: format error\n");
        return false;
    }

    /*
     * mp_vfs_open() / stream API 发生文件系统异常时
     * 会抛出 MicroPython exception。
     *
     * 这里捕获，让 saveConfig() 仍然通过 bool 返回成功/失败。
     */
    nlr_buf_t nlr;

    if (nlr_push(&nlr) == 0)
    {
        /*
         * "w"：
         *
         * 文件不存在 -> 创建
         * 文件存在   -> 清空
         */
        mp_obj_t file = openVfsFile(CONFIG_FILE, "w");

        int err = 0;

        mp_uint_t written = mp_stream_rw(
            file,
            (void *)buffer,
            (mp_uint_t)len,
            &err,
            MP_STREAM_RW_WRITE
        );

        /*
         * close 会让文件系统完成最终写入。
         */
        mp_stream_close(file);

        nlr_pop();

        if (written == MP_STREAM_ERROR || err != 0 || written != (mp_uint_t)len)
        {
            mp_printf(&mp_plat_print,
                "saveConfig: write failed "
                "(written=%u, err=%d)\n",
                (unsigned)written,
                err);

            return false;
        }

        isCalibrated = true;

        return true;
    }
    else
    {
        /*
         * VFS 抛出了异常，例如：
         *
         * 文件系统没有挂载
         * Flash 写入失败
         * 文件系统只读
         */
        mp_hal_stdout_tx_str("saveConfig: VFS exception\n");
        mp_obj_print_exception(&mp_plat_print, MP_OBJ_FROM_PTR(nlr.ret_val));

        return false;
    }
}


/*
 * 从文件读取校准参数
 */
bool loadConfig(void)
{
    /*
     * 先判断文件是否存在。
     *
     * 第一次使用设备时文件不存在，
     * 这是正常情况，不需要抛异常。
     */
    if (mp_vfs_import_stat(CONFIG_FILE) != MP_IMPORT_STAT_FILE)
    {

        return false;
    }

    char buffer[128];

    float middleAngle = 0.0f;
    float gyroOffset = 0.0f;

    bool result = false;

    nlr_buf_t nlr;

    if (nlr_push(&nlr) == 0)
    {
        mp_obj_t file = openVfsFile(CONFIG_FILE, "r");

        int err = 0;

        /*
         * 文件目前只有几十个字节，
         * 128 bytes 完全足够。
         */
        mp_uint_t readLen = mp_stream_rw(
            file,
            buffer,
            sizeof(buffer) - 1,
            &err,
            MP_STREAM_RW_READ
        );

        mp_stream_close(file);

        /*
         * 先退出异常保护区域，
         * 后面再进行普通 C 数据处理。
         */
        nlr_pop();

        if (readLen == MP_STREAM_ERROR ||
            err != 0)
        {
            return false;
        }

        if (readLen == 0)
        {
            return false;
        }

        /*
         * 添加字符串结束符
         */
        buffer[readLen] = '\0';

        /*
         * 文件格式：
         *
         * middleAngle=1.234000
         * gyroOffset=-0.123000
         */
        int count = sscanf(
            buffer,
            "middleAngle=%f\n"
            "gyroOffset=%f",
            &middleAngle,
            &gyroOffset
        );

        if (count != 2)
        {
            return false;
        }

        /*
         * 防止用户手动编辑成 nan / inf。
         */
        if (!isfinite(middleAngle) ||
            !isfinite(gyroOffset))
        {
            mp_hal_stdout_tx_str("loadConfig: invalid value\n");
            return false;
        }

        /*
         * 只有全部读取成功之后
         * 才修改真正的校准参数。
         *
         * 避免文件损坏时把原参数覆盖掉。
         */
        qmi8658CalData.middleAngle = middleAngle;
        qmi8658CalData.gyroOffset = gyroOffset;

        result = true;
    }
    else
    {

        mp_obj_print_exception(
            &mp_plat_print,
            MP_OBJ_FROM_PTR(nlr.ret_val)
        );

        result = false;
    }

    return result;
}

bool getCalibrated()
{
    return isCalibrated;
}