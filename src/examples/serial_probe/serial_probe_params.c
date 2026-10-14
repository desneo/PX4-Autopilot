/****************************************************************************
 *
 * Parameter definitions for the TELEM2 DYT telemetry parser.
 *
 * This file is only scanned by the PX4 parameter generator; it is not
 * compiled, so PARAM_DEFINE_INT32 has no macro definition here on purpose.
 *
 ****************************************************************************/

/**
 * DYT device brand on TELEM2.
 *
 * Selects the DYT telemetry device and therefore the port baud rate:
 * zhongkeyoucheng uses 115200 baud, aisheng uses 230400 baud.
 * The parser itself identifies the vendor by the frame header, so both frame
 * types are accepted at either baud rate.
 * Change the parameter and restart the module to switch the baud rate.
 *
 * @group Serial
 * @min 0
 * @max 1
 * @value 0 zhongkeyoucheng(115200)
 * @value 1 aisheng(230400)
 */
PARAM_DEFINE_INT32(DYT_BRAND, 0);

/**
 * Maximum DYT manual gimbal angular rate.
 *
 * Limits the absolute yaw and pitch rate accepted from
 * dyt_gimbal_rate_command. A conservative default is used for manual target
 * search. Commands beyond this limit are clamped.
 *
 * @group Serial
 * @unit deg/s
 * @min 0.1
 * @max 100.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(DYT_RATE_MAX, 10.0f);

/**
 * DYT manual gimbal angular-rate slew limit.
 *
 * Limits how quickly the commanded yaw and pitch rates change. Set to zero to
 * disable the ramp and apply a new requested rate immediately.
 *
 * @group Serial
 * @unit deg/s^2
 * @min 0.0
 * @max 500.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(DYT_RATE_SLEW, 20.0f);

/**
 * DYT continuous-motion command keepalive interval.
 *
 * A changed command and a stop are sent immediately. While motion remains
 * active, the current rate command is repeated at this interval.
 *
 * @group Serial
 * @unit ms
 * @min 20
 * @max 1000
 */
PARAM_DEFINE_INT32(DYT_CMD_INT, 100);

/**
 * Aisheng movement-command gain.
 *
 * Converts requested angular rate in deg/s to the signed -100..100 movement
 * value used by command 0xB1. The vendor protocol does not define a physical
 * rate scale, so this value must be calibrated on the actual gimbal.
 *
 * @group Serial
 * @min 0.1
 * @max 100.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(DYT_AS_GAIN, 10.0f);
