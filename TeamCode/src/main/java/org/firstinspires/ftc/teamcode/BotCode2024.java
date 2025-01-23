package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import org.firstinspires.ftc.teamcode.PIDController;



@TeleOp
public class BotCode2024 extends LinearOpMode {
    // PIDController control = new PIDController(0.05,0,0);
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor motorArm = hardwareMap.dcMotor.get("Arm");
        DcMotor motorLift = hardwareMap.dcMotor.get("Lift");
        DcMotor motorInTake = hardwareMap.dcMotor.get("InTake");
        DcMotor motorOutTake = hardwareMap.dcMotor.get("OutTake");
        Servo LiftflipLeft = hardwareMap.servo.get("LiftflipLeft");
        Servo LiftflipRight = hardwareMap.servo.get("LiftflipRight");
        Servo ClawflipR = hardwareMap.servo.get("clawflipR");
        Servo ClawflipL = hardwareMap.servo.get("clawflipL");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorInTake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorOutTake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double arm = gamepad1.right_trigger-gamepad1.left_trigger;//armpower
            boolean spin = gamepad1.dpad_down;
            boolean stop = gamepad1.dpad_up;
            boolean up = gamepad2.dpad_up;
            boolean down = gamepad2.dpad_down;
            boolean getPos = gamepad1.right_bumper;
            boolean transferPos = gamepad1.left_bumper;
            boolean liftArmFlip1 = gamepad2.b;
            boolean liftArmFlip2 = gamepad2.a;
            boolean liftArmFlip3 = gamepad2.y;
            boolean liftArmFlip4 = gamepad2.x;

            double OutTakeUp = gamepad2.right_trigger;
            double OutTakeDown = gamepad2.left_trigger;
            double LiftPower = (OutTakeUp-OutTakeDown);


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            motorFrontLeft.setPower(frontLeftPower*(1));
            motorBackLeft.setPower(backLeftPower*(1));
            motorFrontRight.setPower(frontRightPower*(1));
            motorBackRight.setPower(backRightPower*(1));
            motorArm.setPower(arm);

            if(spin){
                motorInTake.setPower(-1);
            }
            if(stop){
                motorInTake.setPower(0);
            }
            if (up){
                motorLift.setPower(1);
            }
            else if (down) {
                motorLift.setPower(-1);
            }
            else{
                motorOutTake.setPower(0.4*(-LiftPower));
                motorLift.setPower(LiftPower);
            }
            if (getPos){
                ClawflipR.setPosition(0.6);
                ClawflipL.setPosition(0.4);
            }
            if (transferPos){
                ClawflipR.setPosition(0.5);
                ClawflipL.setPosition(0.5);
            }
            if (liftArmFlip1) {
                LiftflipLeft.setPosition(0.6);
                LiftflipRight.setPosition(0.4);
            }
            if (liftArmFlip2) {
                LiftflipLeft.setPosition(0);
                LiftflipRight.setPosition(1);
            }
            if (liftArmFlip3) {
                LiftflipLeft.setPosition(0.4);
                LiftflipRight.setPosition(0.6);
                ClawflipR.setPosition(0);
                ClawflipL.setPosition(1);
            }
            if (liftArmFlip4) {
                ClawflipR.setPosition(0.6);
                ClawflipL.setPosition(0.4);
                LiftflipLeft.setPosition(0.6);
                LiftflipRight.setPosition(0.4);
            }


        }
    }
}