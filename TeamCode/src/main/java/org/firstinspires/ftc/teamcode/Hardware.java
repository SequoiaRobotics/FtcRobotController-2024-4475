package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.AutoClearEncoder;
import org.firstinspires.ftc.teamcode.hardware.HardwareMapper;
import org.firstinspires.ftc.teamcode.hardware.HardwareName;
import org.firstinspires.ftc.teamcode.hardware.MotorSet;
import org.firstinspires.ftc.teamcode.hardware.Reversed;
import org.firstinspires.ftc.teamcode.hardware.ZeroPower;
import org.firstinspires.ftc.teamcode.localization.TriOdoProvider;

public class Hardware extends HardwareMapper implements TriOdoProvider {
    // left = left motor = exp 0 frontLeft
    // right = right motor = ctr 0 frontRight
    // center = ctr 3 intake

    @HardwareName("frontLeft")
    @Reversed
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor frontLeft;

    @HardwareName("frontRight")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor frontRight;

    @HardwareName("backLeft")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    @Reversed
    public DcMotor backLeft;

    @HardwareName("backRight")
    @ZeroPower(DcMotor.ZeroPowerBehavior.BRAKE)
    public DcMotor backRight;

    @HardwareName("frontLeft")
    @AutoClearEncoder
    public DcMotor encoderLeft;

    @HardwareName("intake")
    @AutoClearEncoder
    public DcMotor encoderCenter;

    @HardwareName("frontRight")
    @AutoClearEncoder
    public DcMotor encoderRight;

    @Override
    public DcMotor getLeftEncoder() {
        return encoderLeft;
    }

    @Override
    public DcMotor getRightEncoder() {
        return encoderRight;
    }

    @Override
    public DcMotor getCenterEncoder() {
        return encoderCenter;
    }

    @Override
    public double getTrackWidth() {
        return 14 + 7 / 16.;
    }

    @Override
    public double getForwardOffset() {
        return -(6 + 3 / 4.);
    }

    public MotorSet driveMotors;

    public Hardware(HardwareMap hwMap) {
        super(hwMap);
        driveMotors = new MotorSet(
                frontLeft,
                frontRight,
                backLeft,
                backRight
        );
    }
}