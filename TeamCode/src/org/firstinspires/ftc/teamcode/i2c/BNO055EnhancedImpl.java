package org.firstinspires.ftc.teamcode.i2c;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import virtual_robot.controller.VirtualBot;
//import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
//import com.qualcomm.robotcore.hardware.TimestampedI2cData;
//import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
//import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.ReadWriteFile;
//
//import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
//
//import java.io.File;
//import java.io.IOException;

/**
 * Created by FTC Team 8397 on 9/11/2019.
 */

//@I2cDeviceType
//@DeviceProperties(xmlTag = "BN0055EnhancedImpl", name = "BN0055EnhancedImpl", description = "BNO055EnhancedImpl")
public class BNO055EnhancedImpl extends BNO055IMUImpl implements BNO055Enhanced {

    public BNO055EnhancedImpl(VirtualBot bot, int latencyMillis){
        super(bot, latencyMillis);
    }

//    final static byte bCHIP_ID_VALUE = (byte)0xa0;
//
//    enum POWER_MODE
//    {
//        NORMAL(0X00),
//        LOWPOWER(0X01),
//        SUSPEND(0X02);
//        //------------------------------------------------------------------------------------------
//        protected byte value;
//        POWER_MODE(int value) { this.value = (byte)value; }
//        public byte getValue() { return this.value; }
//    }

//    @Override
//    public String getDeviceName() {
//        return "BNO055";
//    }
//
//    @Override
//    public Manufacturer getManufacturer() {
//        return Manufacturer.Other;
//    }
//
//    public BNO055EnhancedImpl (I2cDeviceSynch deviceClient){
//        super(deviceClient);
//    }

    /**
     * Do one attempt at initializing the device to be running in the indicated operation mode
     */
//    protected boolean internalInitializeOnce(SystemStatus expectedStatus)
//    {
//        // Validate parameters
//        if (SensorMode.CONFIG == parameters.mode)
//            throw new IllegalArgumentException("SensorMode.CONFIG illegal for use as initialization mode");
//
//        ElapsedTime elapsed = new ElapsedTime();
//        if (parameters.accelerationIntegrationAlgorithm != null)
//        {
//            this.accelerationAlgorithm = parameters.accelerationIntegrationAlgorithm;
//        }
//
//        // Lore: "send a throw-away command [...] just to make sure the BNO is in a good state
//        // and ready to accept commands (this seems to be necessary after a hard power down)."
//        write8(Register.PAGE_ID, 0);
//
//        // Make sure we have the right device
//        byte chipId = read8(Register.CHIP_ID);
//        if (chipId != bCHIP_ID_VALUE)
//        {
//            delayExtra(650);     // delay value is from from Table 0-2 in the BNO055 specification
//            chipId = read8(Register.CHIP_ID);
//            if (chipId != bCHIP_ID_VALUE)
//            {
//                log_e("unexpected chip: expected=%d found=%d", bCHIP_ID_VALUE, chipId);
//                return false;
//            }
//        }
//
//        // Get us into config mode, for sure
//        setSensorMode(SensorMode.CONFIG);
//
//        // Reset the system, and wait for the chip id register to switch back from its reset state
//        // to the it's chip id state. This can take a very long time, some 650ms (Table 0-2, p13)
//        // perhaps. While in the reset state the chip id (and other registers) reads as 0xFF.
//        TimestampedI2cData.suppressNewHealthWarnings(true);
//        try {
//            elapsed.reset();
//            write8(Register.SYS_TRIGGER, 0x20);
//            for (;;)
//            {
//                chipId = read8(Register.CHIP_ID);
//                if (chipId == bCHIP_ID_VALUE)
//                    break;
//                delayExtra(10);
//                if (elapsed.milliseconds() > msAwaitChipId)
//                {
//                    log_e("failed to retrieve chip id");
//                    return false;
//                }
//            }
//            delayLoreExtra(50);
//        }
//        finally
//        {
//            TimestampedI2cData.suppressNewHealthWarnings(false);
//        }
//
//        // Set to normal power mode
//        write8(Register.PWR_MODE, POWER_MODE.NORMAL.getValue());
//        delayLoreExtra(10);
//
//        // Make sure we're looking at register page zero, as the other registers
//        // we need to set here are on that page.
//        write8(Register.PAGE_ID, 0);
//
//        // Set the output units. Section 3.6, p31
//        int unitsel = (parameters.pitchMode.bVal << 7) |       // pitch angle convention
//                (parameters.temperatureUnit.bVal << 4) | // temperature
//                (parameters.angleUnit.bVal << 2) |       // euler angle units
//                (parameters.angleUnit.bVal << 1) |       // gyro units, per second
//                (parameters.accelUnit.bVal /*<< 0*/);    // accelerometer units
//        write8(Register.UNIT_SEL, unitsel);
//
//        write8(Register.AXIS_MAP_CONFIG, ((BNO055Enhanced.Parameters)parameters).axesMap.bVal);
//        write8(Register.AXIS_MAP_SIGN, ((BNO055Enhanced.Parameters)parameters).axesSign.bVal);
//
//        // Use or don't use the external crystal
//        // See Section 5.5 (p100) of the BNO055 specification.
//        write8(Register.SYS_TRIGGER, parameters.useExternalCrystal ? 0x80 : 0x00);
//        delayLoreExtra(50);
//
//        // Switch to page 1 so we can write some more registers
//        write8(Register.PAGE_ID, 1);
//
//        // Configure selected page 1 registers
//        write8(Register.ACC_CONFIG, parameters.accelPowerMode.bVal | parameters.accelBandwidth.bVal | parameters.accelRange.bVal);
//        write8(Register.MAG_CONFIG, parameters.magPowerMode.bVal | parameters.magOpMode.bVal | parameters.magRate.bVal);
//        write8(Register.GYR_CONFIG_0, parameters.gyroBandwidth.bVal | parameters.gyroRange.bVal);
//        write8(Register.GYR_CONFIG_1, parameters.gyroPowerMode.bVal);
//
//        // Switch back
//        write8(Register.PAGE_ID, 0);
//
//        // Run a self test. This appears to be a necessary step in order for the
//        // sensor to be able to actually be used. That is, we've observed that absent this,
//        // the sensors do not return correct data. We wish that were documented somewhere.
//        write8(Register.SYS_TRIGGER, read8(Register.SYS_TRIGGER) | 0x01);           // SYS_TRIGGER=0x3F
//
//        // Start a timer: we only give the self-test a certain length of time to run
//        elapsed.reset();
//
//        // It's a little unclear how to conclude when the self test is complete. getSystemStatus()
//        // can report SystemStatus.SELF_TEST, and one might be lead to think that that will remain
//        // true while the self test is running, but that appears not actually to be the case, as
//        // sometimes we see SystemStatus.SELF_TEST being reported even after two full seconds. So,
//        // we fall back on to what we've always done, and just check the results of the tested
//        // sensors we actually care about.
//
//        // Per Section 3.9.2 Built In Self Test, when we manually kick off a self test,
//        // the accelerometer, gyro, and magnetometer are tested, but the microcontroller is not.
//        // So: we only look for successful results from those three.
//        final int successfulResult = 0x07;
//        final int successfulResultMask = 0x07;
//        boolean selfTestSuccessful = false;
//        while (!selfTestSuccessful && elapsed.milliseconds() < msAwaitSelfTest)
//        {
//            selfTestSuccessful = (read8(Register.SELFTEST_RESULT)&successfulResultMask) == successfulResult;    // SELFTEST_RESULT=0x36
//        }
//        if (!selfTestSuccessful)
//        {
//            int result = read8(Register.SELFTEST_RESULT);
//            log_e("self test failed: 0x%02x", result);
//            return false;
//        }
//
//        if (this.parameters.calibrationData != null)
//        {
//            writeCalibrationData(this.parameters.calibrationData);
//        }
//        else if (this.parameters.calibrationDataFile != null)
//        {
//            try {
//                File file = AppUtil.getInstance().getSettingsFile(this.parameters.calibrationDataFile);
//                String serialized = ReadWriteFile.readFileOrThrow(file);
//                CalibrationData data = CalibrationData.deserialize(serialized);
//                writeCalibrationData(data);
//            }
//            catch (IOException e)
//            {
//                // Ignore the absence of the indicated file, etc
//            }
//        }
//
//        // Finally, enter the requested operating mode (see section 3.3).
//        setSensorMode(parameters.mode);
//
//        // At this point, the chip should in fact report correctly that it's in the mode requested.
//        // See Section '4.3.58 SYS_STATUS' of the BNO055 specification. That said, we've seen issues
//        // where the first mode request somehow doesn't take, so we re-issue. We don't understand the
//        // circumstances that cause this condition (or we'd avoid them!).
//        SystemStatus status = getSystemStatus();
//        if (status != expectedStatus)
//        {
//            log_w("re-issuing IMU mode: system status=%s expected=%s", status, expectedStatus);
//            delayLore(100);
//            setSensorMode(parameters.mode);
//            status = getSystemStatus();
//        }
//
//        if (status==expectedStatus)
//            return true;
//        else
//        {
//            log_w("IMU initialization failed: system status=%s expected=%s", status, expectedStatus);
//            return false;
//        }
//    }

}
