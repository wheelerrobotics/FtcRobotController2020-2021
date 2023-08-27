/*
Copyright (c) 2017 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package com.qualcomm.hardware.stmicroelectronics;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDeviceHealth.HealthStatus;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.DYNAMIC_SPAD_REF_EN_START_OFFSET;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.FINAL_RANGE_CONFIG_VCSEL_PERIOD;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.GLOBAL_CONFIG_REF_EN_START_SELECT;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.GLOBAL_CONFIG_SPAD_ENABLES_REF_0;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.GPIO_HV_MUX_ACTIVE_HIGH;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.MSRC_CONFIG_CONTROL;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.MSRC_CONFIG_TIMEOUT_MACROP;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.OSC_CALIBRATE_VAL;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.PRE_RANGE_CONFIG_VCSEL_PERIOD;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.RESULT_INTERRUPT_STATUS;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.RESULT_RANGE_STATUS;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.SYSRANGE_START;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.SYSTEM_INTERMEASUREMENT_PERIOD;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.SYSTEM_INTERRUPT_CLEAR;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.SYSTEM_INTERRUPT_CONFIG_GPIO;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.SYSTEM_SEQUENCE_CONFIG;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.vcselPeriodType.VcselPeriodFinalRange;
import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.vcselPeriodType.VcselPeriodPreRange;



/**
 * {@link VL53L0X} implements support for the STMicroelectronics VL53L0x time-of-flight distance sensor.
 *
 * @see <a href="http://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html">STMicroelectronics VL53L0X Sensor</a>
 *
 */
public class MB1242 extends I2cDeviceSynchDevice<I2cDeviceSynch> implements DistanceSensor
{
    //***********************************************************************************************
    // User methods.
    //***********************************************************************************************
    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName()
    {
        return "MB1242 Ultrasonic Distance Sensor";
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        double range = (double)this.readRangeContinuousMillimeters();

        if (unit == DistanceUnit.CM) {
            return range / 10;
        } else if (unit == DistanceUnit.METER) {
            return range / 1000;
        } else if (unit == DistanceUnit.INCH) {
            return range / 25.4;
        } else {
            return range;
        }
    }

    /**
     * Did a timeout occur?
     */
    public boolean didTimeoutOccur() {
        return did_timeout;
    }

    //***********************************************************************************************
    // Constant values and sensor memory register.
    //***********************************************************************************************
    // default I2C address as defined in STMicroelectronics documentatation.
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create8bit(0x52);

    // Distance value in millimeters used when the sensor is not responding
    protected final static int FAKE_DISTANCE_MM = 65535;

    // String tag for logging.
    protected String MYTAG = "STMicroVL53L0X: ";

    // The register addresses are listed in the vl53l0x_device.h file that
    // is part of the VL53L0XFoo API.
    // Note that the numerical order of the registers as listed in the vl530l0x_device.h file
    // is not ordered from smallest to largest address values (the addresses seem haphazardly assigned).
    // This registry does not lend itself to an optimized read window.


    // make this a long since example has this as unsigned 32bit int.
    //uint32_t measurement_timing_budget_us;
    long measurement_timing_budget_us;

    enum vcselPeriodType { VcselPeriodPreRange, VcselPeriodFinalRange };

    protected int io_timeout = 0;
    protected ElapsedTime ioElapsedTime;
    boolean did_timeout = false;
    boolean assume_uninitialized = true;

    //***********************************************************************************************
    // Construction and initialization.
    //***********************************************************************************************

    // Constructor.
    public MB1242(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);

        //this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();

        ioElapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        did_timeout = false;
    }

    // initialize the sensor.
    @Override
    protected synchronized boolean doInitialize()
    {
        // *************************************************************
        // check to see if this is really a VL53L0X sensor.
        // documentation indates that after fresh reset, without API loaded,
        // the following registers should have the following values (reg addr, value)
        // *************************************************************
        // 0xC0 0xEE
        // 0xC1 0xAA
        // 0xC2 0x10
        // 0x51 0x0099
        // 0x61 0x0000
        // check the registers to see if they match these known values...
        // note that in my testing, register 0x51 seems to return value of 0x00.
        byte bVal;
        RobotLog.dd(MYTAG, "Checking to see if it's really a VL53L0X sensor...");
        bVal = deviceClient.read8(0xC0);
        RobotLog.dd(MYTAG, "Reg 0xC0 = %x (should be 0xEE)", bVal);
        bVal = deviceClient.read8(0xC1);
        RobotLog.dd(MYTAG, "Reg 0xC1 = %x (should be 0xAA)", bVal);
        bVal = deviceClient.read8(0xC2);
        RobotLog.dd(MYTAG, "Reg 0xC2 = %x (should be 0x10)", bVal);
        bVal = deviceClient.read8(0x51);
        RobotLog.dd(MYTAG, "Reg 0x51 = %x (should be 0x0099)", bVal);
        bVal = deviceClient.read8(0x61);
        RobotLog.dd(MYTAG, "Reg 0x61 = %x (should be 0x0000)", bVal);

        // initialize the sensor.
        // if you specify false as the bUse2v8 argument, the sensor will operate in 1.8V mode.
        return initVL53L0X(false);
    }

    // Set the return signal rate limit check value in units of MCPS (mega counts
    // per second). "This represents the amplitude of the signal reflected from the
    // target and detected by the device"; setting this limit presumably determines
    // the minimum measurement necessary for the sensor to report a valid reading.
    // Setting a lower limit increases the potential range of the sensor but also
    // seems to increase the likelihood of getting an inaccurate reading because of
    // unwanted reflections from objects other than the intended target.
    // Defaults to 0.25 MCPS as initialized by the ST API and this library.
    private boolean setSignalRateLimit(float limit_Mcps)
    {
        // check range.
        if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

        // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
        writeShort(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (short)(limit_Mcps * (1 << 7)));
        return true;
    }

    // Get the return signal rate limit check value in MCPS
    private float getSignalRateLimit()
    {
        return (float)readShort(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
    }

    // Get reference SPAD (single photon avalanche diode) count and type
    // based on VL53L0X_get_info_from_device(),
    // but only gets reference SPAD count and type
    private boolean getSpadInfo() {
        byte tmp;

        writeReg(0x80, 0x01);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);

        writeReg(0xFF, 0x06);
        writeReg(0x83, (byte) (this.deviceClient.read8(0x83) | 0x04));
        writeReg(0xFF, 0x07);
        writeReg(0x81, 0x01);

        writeReg(0x80, 0x01);

        writeReg(0x94, 0x6b);
        writeReg(0x83, 0x00);

        // example had a timeout mechanism, but
        // this was disabled and not active.
        // checkTimeoutExpired() in example returned false since timer
        // was never initialized.
        // comment it out in our translation of the sample driver.
//        startTimeout();
//        while (this.deviceClient.read8(0x83) == 0x00) {
//            if (checkTimeoutExpired()) {
//                return false;
//            }
//        }

        writeReg(0x83, 0x01);
        tmp = readReg(0x92);

        //  *count = tmp & 0x7f;
        //  *type_is_aperture = (tmp >> 7) & 0x01;
        spad_count = (byte) (tmp & 0x7f);
        spad_type_is_aperture = ((tmp >> 7) & 0x01) == 0 ? false : true;

        writeReg(0x81, 0x00);
        writeReg(0xFF, 0x06);
        writeReg(0x83, readReg(0x83) & ~0x04);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x01);

        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);

        return true;
    }

    // Get the measurement timing budget in microseconds
    // based on VL53L0X_get_measurement_timing_budget_micro_seconds()
    // in us
    long getMeasurementTimingBudget() {
        // getMeasurementTimingBudget method uses local structures and passes them by
        // reference... we have to define them as classes.  Then when they are passed as an argument
        // their fields will get updated within the method.
        SequenceStepEnables enables = new SequenceStepEnables();
        SequenceStepTimeouts timeouts = new SequenceStepTimeouts();

        final int StartOverhead = 1910; // note that this is different than the value in set_
        final int EndOverhead = 960;
        final int MsrcOverhead = 660;
        final int TccOverhead = 590;
        final int DssOverhead = 690;
        final int PreRangeOverhead = 660;
        final int FinalRangeOverhead = 550;

        // "Start and end overhead times always present"
        long budget_us = StartOverhead + EndOverhead;

        getSequenceStepEnables(enables);
        getSequenceStepTimeouts(enables, timeouts);

        if (enables.tcc) {
            budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
        }

        if (enables.dss) {
            budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
        } else if (enables.msrc) {
            budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
        }

        if (enables.pre_range) {
            budget_us += (timeouts.pre_range_us + PreRangeOverhead);
        }

        if (enables.final_range) {
            budget_us += (timeouts.final_range_us + FinalRangeOverhead);
        }

        measurement_timing_budget_us = budget_us; // store for internal reuse
        return budget_us;
    }

    protected class SequenceStepEnables {
        boolean tcc, msrc, dss, pre_range, final_range;
    }

    protected class SequenceStepTimeouts {
        int pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

        int msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
        long msrc_dss_tcc_us, pre_range_us, final_range_us;
    }

    // Get sequence step enables
    // based on VL53L0X_GetSequenceStepEnables()
    protected void getSequenceStepEnables(SequenceStepEnables enables) {
        int sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);

        enables.tcc = ((sequence_config >> 4) & 0x1) !=0 ? true : false;
        enables.dss = ((sequence_config >> 3) & 0x1) != 0 ? true : false;
        enables.msrc = ((sequence_config >> 2) & 0x1) != 0 ? true : false;
        enables.pre_range = ((sequence_config >> 6) & 0x1) != 0 ? true : false;
        enables.final_range = ((sequence_config >> 7) & 0x1) != 0 ? true : false;
    }


    // Get sequence step timeouts
    // based on get_sequence_step_timeout(),
    // but gets all timeouts instead of just the requested one, and also stores
    // intermediate values
    protected void getSequenceStepTimeouts(SequenceStepEnables enables, SequenceStepTimeouts timeouts) {
        timeouts.pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

        timeouts.msrc_dss_tcc_mclks = readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
        timeouts.msrc_dss_tcc_us =
                timeoutMclksToMicroseconds(timeouts.msrc_dss_tcc_mclks,
                        timeouts.pre_range_vcsel_period_pclks);

        timeouts.pre_range_mclks =
                decodeTimeout(readShort(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
        timeouts.pre_range_us =
                timeoutMclksToMicroseconds(timeouts.pre_range_mclks,
                        timeouts.pre_range_vcsel_period_pclks);

        timeouts.final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);

        timeouts.final_range_mclks =
                decodeTimeout(readShort(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

        if (enables.pre_range) {
            timeouts.final_range_mclks -= timeouts.pre_range_mclks;
        }

        timeouts.final_range_us =
                timeoutMclksToMicroseconds(timeouts.final_range_mclks,
                        timeouts.final_range_vcsel_period_pclks);
    }

    // Decode sequence step timeout in MCLKs from register value
    // based on VL53L0X_decode_timeout()
    // Note: the original function returned a uint32_t, but the return value is
    // always stored in a uint16_t.
    int decodeTimeout(int reg_val) {
        // format: "(LSByte * 2^MSByte) + 1"
        return (int) ((reg_val & 0x00FF) <<
                (int) ((reg_val & 0xFF00) >> 8)) + 1;
    }

    // Get the VCSEL pulse period in PCLKs for the given period type.
    // based on VL53L0X_get_vcsel_pulse_period()
    protected int getVcselPulsePeriod(vcselPeriodType type) {
        if (type == VcselPeriodPreRange) {
            return decodeVcselPeriod(readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
        } else if (type == VcselPeriodFinalRange) {
            return decodeVcselPeriod(readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
        } else {
            return 255;
        }
    }

    // Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
    // from register value
    // based on VL53L0X_decode_vcsel_period()
    protected int decodeVcselPeriod(int reg_val)   {
        int val = (((reg_val) + 1) << 1);
        return val;
    }

    // Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
    // based on VL53L0X_calc_timeout_us()
    protected long timeoutMclksToMicroseconds(int timeout_period_mclks, int vcsel_period_pclks) {
        long macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

        return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
    }

    // Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
    // based on VL53L0X_calc_macro_period_ps()
    // PLL_period_ps = 1655; macro_period_vclks = 2304
    protected long calcMacroPeriod(int vcsel_period_pclks) {
        long val = ((((long)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000);
        return val;
    }

    // Set the measurement timing budget in microseconds, which is the time allowed
    // for one measurement; the ST API and this library take care of splitting the
    // timing budget among the sub-steps in the ranging sequence. A longer timing
    // budget allows for more accurate measurements. Increasing the budget by a
    // factor of N decreases the range measurement standard deviation by a factor of
    // sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
    // based on VL53L0X_set_measurement_timing_budget_micro_seconds()
    protected boolean setMeasurementTimingBudget(long budget_us) {
        SequenceStepEnables enables = new SequenceStepEnables();
        SequenceStepTimeouts timeouts = new SequenceStepTimeouts();

        final int StartOverhead = 1320; // note that this is different than the value in get_
        final int EndOverhead = 960;
        final int MsrcOverhead = 660;
        final int TccOverhead = 590;
        final int DssOverhead = 690;
        final int PreRangeOverhead = 660;
        final int FinalRangeOverhead = 550;

        final long MinTimingBudget = 20000;

        if (budget_us < MinTimingBudget) {
            return false;
        }

        long used_budget_us = StartOverhead + EndOverhead;

        getSequenceStepEnables(enables);
        getSequenceStepTimeouts(enables, timeouts);

        if (enables.tcc) {
            used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
        }

        if (enables.dss) {
            used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
        } else if (enables.msrc) {
            used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
        }

        if (enables.pre_range) {
            used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
        }

        if (enables.final_range) {
            used_budget_us += FinalRangeOverhead;

            // "Note that the final range timeout is determined by the timing
            // budget and the sum of all other timeouts within the sequence.
            // If there is no room for the final range timeout, then an error
            // will be set. Otherwise the remaining time will be applied to
            // the final range."

            if (used_budget_us > budget_us) {
                // "Requested timeout too big."
                return false;
            }

            long final_range_timeout_us = budget_us - used_budget_us;

            // set_sequence_step_timeout() begin
            // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

            // "For the final range timeout, the pre-range timeout
            //  must be added. To do this both final and pre-range
            //  timeouts must be expressed in macro periods MClks
            //  because they have different vcsel periods."

            long final_range_timeout_mclks =
                    timeoutMicrosecondsToMclks(final_range_timeout_us,
                            timeouts.final_range_vcsel_period_pclks);

            if (enables.pre_range) {
                final_range_timeout_mclks += timeouts.pre_range_mclks;
            }

            writeShort(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                    (short)encodeTimeout((int)final_range_timeout_mclks));

            // set_sequence_step_timeout() end

            measurement_timing_budget_us = budget_us; // store for internal reuse
        }
        return true;
    }

    // Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
    // based on VL53L0X_calc_timeout_mclks()
    protected long timeoutMicrosecondsToMclks(long timeout_period_us, int vcsel_period_pclks) {
        long macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

        return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
    }

    // Encode sequence step timeout register value from timeout in MCLKs
    // based on VL53L0X_encode_timeout()
    // Note: the original function took a uint16_t, but the argument passed to it
    // is always a uint16_t.
    protected long encodeTimeout(int timeout_mclks) {
        // format: "(LSByte * 2^MSByte) + 1"
        long ls_byte = 0;
        int ms_byte = 0;

        if (timeout_mclks > 0) {
            ls_byte = timeout_mclks - 1;

            while ((ls_byte & 0xFFFFFF00) > 0) {
                ls_byte >>= 1;
                ms_byte++;
            }

            return (ms_byte << 8) | (ls_byte & 0xFF);
        } else {
            return 0;
        }
    }

    // based on VL53L0X_perform_single_ref_calibration()
    protected boolean performSingleRefCalibration(int vhv_init_byte) {
        writeReg(SYSRANGE_START.bVal, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

        // in example, during initialization the timeout was disabled, so i'm commenting out here.
        // if we call this method after init is complete, we might have
        // to implement this timeout-related code.
//        startTimeout();
//        while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
//            if (checkTimeoutExpired()) {
//                return false;
//            }
//        }

        writeReg(SYSTEM_INTERRUPT_CLEAR.bVal, 0x01);

        writeReg(SYSRANGE_START.bVal, 0x00);

        return true;
    }

    protected void startContinuous() {
        this.startContinuous(0);
    }
    // Start continuous ranging measurements. If period_ms (optional) is 0 or not
    // given, continuous back-to-back mode is used (the sensor takes measurements as
    // often as possible); otherwise, continuous timed mode is used, with the given
    // inter-measurement period in milliseconds determining how often the sensor
    // takes a measurement.
    // based on VL53L0X_StartMeasurement()
    protected void startContinuous(int period_ms) {
        writeReg(0x80, 0x01);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        writeReg(0x91, stop_variable);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);

        if (period_ms != 0) {
            // continuous timed mode

            // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

            int osc_calibrate_val = readShort(OSC_CALIBRATE_VAL);

            if (osc_calibrate_val != 0) {
                period_ms *= osc_calibrate_val;
            }
            this.deviceClient.write(SYSTEM_INTERMEASUREMENT_PERIOD.bVal, TypeConversion.intToByteArray(period_ms));

            // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

            writeReg(SYSRANGE_START.bVal, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
        } else {
            // continuous back-to-back mode
            writeReg(SYSRANGE_START.bVal, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
        }
    }

    // Stop continuous measurements
    // based on VL53L0X_StopMeasurement()
    protected void  stopContinuous() {
        writeReg(SYSRANGE_START.bVal, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        writeReg(0x91, 0x00);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
    }

    protected void setTimeout(int timeout) { io_timeout = timeout; }
    protected int  getTimeout() { return io_timeout; }

    // Returns a range reading in millimeters when continuous mode is active
    // (readRangeSingleMillimeters() also calls this function after starting a
    // single-shot range measurement)
    protected int readRangeContinuousMillimeters() {
        if(io_timeout > 0) {
            ioElapsedTime.reset();
        }
        if (assume_uninitialized) {
            return FAKE_DISTANCE_MM; // We have strong evidence that the sensor is not initialized, so we fail fast.
        }
        while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
            if ((did_timeout && deviceClient.getHealthStatus() == HealthStatus.UNHEALTHY) || Thread.currentThread().isInterrupted()) {
                /* The last time we tried to read the sensor, it timed out, and the read attempt
                   we just made also failed (indicated by the status being unhealthy).
                   Exit now so that we don't bottleneck the whole system. */
                return FAKE_DISTANCE_MM;
            }
            if (ioElapsedTime.milliseconds() > io_timeout) {
                did_timeout = true;
                if (deviceClient.getHealthStatus() == HealthStatus.HEALTHY) {
                    /* If we are able to communicate with the device (it is healthy) and yet we got
                        a timeout,it's safe to assume that the device is not currently initialized. */
                    assume_uninitialized = true;
                    // TODO(Noah): Display warning that the sensor needs to be re-initialized
                }
                return FAKE_DISTANCE_MM;
            }
        }
        did_timeout = false; // The read succeeded. Clear the timeout flag.

        // assumptions: Linearity Corrective Gain is 1000 (default);
        // fractional ranging is not enabled
        int range = (int)TypeConversion.byteArrayToShort(deviceClient.read(RESULT_RANGE_STATUS.bVal + 10, 2));
        writeReg(SYSTEM_INTERRUPT_CLEAR.bVal, 0x01);

        return range;
    }

    //***********************************************************************************************
    // Useful methods to read/write from device.
    //***********************************************************************************************

    // read a byte.
    protected byte readReg(Register reg)
    {
        return this.deviceClient.read8(reg.bVal);
    }
    protected byte readReg(byte bVal)
    {
        return this.deviceClient.read8(bVal);
    }
    protected byte readReg(int iVal)
    {
        return this.deviceClient.read8((byte)iVal);
    }

    // write a byte.
    protected void writeReg(Register reg, byte value)
    {
        this.writeReg(reg, value, I2cWaitControl.NONE);
    }
    protected void writeReg(byte addr, byte value)
    {
        this.writeReg(addr, value, I2cWaitControl.NONE);
    }
    protected void writeReg(int addr, int value)
    {
        this.writeReg((byte)addr, (byte)value, I2cWaitControl.NONE);
    }

    // write a byte and specify wait behavior for the transaction.
    protected void writeReg(Register reg, byte value, I2cWaitControl waitControl)
    {
        this.deviceClient.write8(reg.bVal, value, waitControl);
    }
    protected void writeReg(byte addr, byte value, I2cWaitControl waitControl)
    {
        this.deviceClient.write8(addr, value, waitControl);
    }

    protected void writeReg(int addr, int value, I2cWaitControl waitControl)
    {
        this.deviceClient.write8((byte)addr, (byte)value, waitControl);
    }


    protected int readUnsignedByte(Register reg)
    {
        return TypeConversion.unsignedByteToInt(this.readReg(reg));
    }

    protected void writeShort(final Register reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

}
