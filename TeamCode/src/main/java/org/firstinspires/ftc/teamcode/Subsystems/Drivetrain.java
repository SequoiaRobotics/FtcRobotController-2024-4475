package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.Base64;

public class Drivetrain {
    public DcMotor fL;
    public DcMotor bL;
    public DcMotor fR;
    public DcMotor bR;
    public Encoder encoder;
    //public static double TICKS_PER_REV = 1;
    public static double GEAR_RATIO = 1;
    public static double WHEEL_RADIUS_INCHES = 1.88976;
    public static double TICKS_PER_REV = 537.6;
    //public IMU imu;

    public void init(HardwareMap map) {
        fL = map.dcMotor.get("frontLeft");
        bL = map.dcMotor.get("backLeft");
        fR = map.dcMotor.get("frontRight");
        bR = map.dcMotor.get("backRight");
        encoder = new Encoder(map.get(DcMotorEx.class, "frontLeft"));
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder

        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public double getPosition() {
        return -encoder.getCurrentPosition();
    }

    public void moveMoveMOVE(double power){
        double fLPow = power;
        double bLPow = power;
        double fRPow = power;
        double bRPow = power;
        setPowers(fLPow,bLPow,fRPow,bRPow);
    }

    // left joystick controls forward/backward and strafe, right controls turning
    public void move(double power, double strafe, double turn) {
        // normalize so doesn't exceed 1
        //double norm = Math.max(Math.abs(power) + Math.abs(strafe) + Math.abs(turn), 1);
        double norm = 20;
        double fLPow = power + strafe + turn;
        double bLPow = power - strafe + turn;
        double fRPow = power - strafe - turn;
        double bRPOw = power + strafe - turn;

        setPowers(fLPow/norm, bLPow/norm, fRPow/norm, bRPOw/norm);
    }

    public void setPowers(double fLPow, double bLPow, double fRPow, double bRPOw) {
        fL.setPower(fLPow);
        bL.setPower(bLPow);
        fR.setPower(fRPow);
        bR.setPower(bRPOw);
    }

    public void setPowers(double pow) {
        this.setPowers(pow, pow, pow, pow);
    }

    public void setTurnPower(double pow) {
        fL.setPower(pow);
        bL.setPower(pow);
        fR.setPower(-pow);
        bR.setPower(-pow);
    }
}