package org.firstinspires.ftc.teamcode.i2c;

import com.qualcomm.hardware.bosch.BNO055IMU;

/**
 * Created by FTC Team 8397 on 9/11/2019.
 */
public interface BNO055Enhanced extends BNO055IMU {

    class Parameters extends BNO055IMU.Parameters{

        public AxesMap axesMap = AxesMap.XYZ;
        public AxesSign axesSign = AxesSign.PPP;

    }
    /**
     * Provides for mapping of the intrinsic XYZ axes of the device onto different axes
     */
    enum AxesMap
    {
        XYZ(0b100100),  //The Default
        XZY(0b011000),  //Swap Z and Y axes
        YXZ(0b100001),  //Swap X and Y axes
        YZX(0b001001),  //One of the two possible three-way axis swaps
        ZYX(0b000110),  //Swap X and Z axes
        ZXY(0b010010);  //The other three-way axis swap

        public final byte bVal;
        AxesMap(int i) { this.bVal = (byte)i; }

    }

    enum AxesSign
    {
        PPP(0b000), //All axes positive, the default
        PPN(0b001), //Z axis negative
        PNP(0b010), //Y axis negative
        PNN(0b011), //Y and Z axes negative
        NPP(0b100), //X axis negative
        NPN(0b101), //X and Z axes negative
        NNP(0b110), //X and Y axes negative
        NNN(0b111); //All axes negative

        public final byte bVal;
        AxesSign(int i) { this.bVal = (byte)i; }

    }
}
