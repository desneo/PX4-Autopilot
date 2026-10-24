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
