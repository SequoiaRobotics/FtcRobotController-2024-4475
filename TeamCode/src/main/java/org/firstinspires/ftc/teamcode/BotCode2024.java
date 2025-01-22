package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import org.firstinspires.ftc.teamcode.PIDController;

@TeleOp
public class BotCode2024 extends LinearOpMode {
    //PIDController control = new PIDController(0.05,0,0);
    @Override
    public void runOpMode(){

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor motorArm = hardwareMap.dcMotor.get("Arm");
        DcMotor motorArm2 = hardwareMap.dcMotor.get("Arm2");
        DcMotor motorLift = hardwareMap.dcMotor.get("Lift");
        DcMotor motorInTake = hardwareMap.dcMotor.get("InTake");
        CRServo LiftDown = hardwareMap.crservo.get("LiftDown");
        //Servo Claw = hardwareMap.servo.get("claw");
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
        motorArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double arm = gamepad1.right_trigger-gamepad1.left_trigger;//armpower
            boolean lift = gamepad2.dpad_up;
            boolean liftDown = gamepad2.dpad_down;
            boolean Flip = gamepad2.a;
            boolean unFlip = gamepad2.b;
            boolean FlipStartPos = gamepad2.y;
            boolean FlipPickUpPos = gamepad1.x;
            boolean transferPos = gamepad1.y;
            //boolean downServo = gamepad2.right_bumper;
            //boolean upServo = gamepad2.left_bumper;
            boolean clawStartPos = gamepad1.left_bumper;
            boolean clawFlip = gamepad1.a;
            boolean clawUnFlip = gamepad1.b;
            int armPos = 0;
            boolean spin = gamepad1.dpad_down;
            boolean stop = gamepad1.dpad_up;


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
            motorArm2.setPower(arm);
            //motorLift.setPower(lift);
            if(spin){
                motorInTake.setPower(1);
            }
            if(stop){
                motorInTake.setPower(0);
            }
            if(lift){
                motorLift.setPower(-1);
                LiftDown.setPower(-1);
            }
            else if(liftDown){
                motorLift.setPower(0.7);
                LiftDown.setPower(1);
            }
            else{
                motorLift.setPower(0);
                LiftDown.setPower(0);
            }
            /*
            if(downServo){
                LiftDown.setPower(1);
            }
            if(upServo){
                LiftDown.setPower(-1);
            }
            */
            if(Flip){
                LiftflipLeft.setPosition(1);
                LiftflipRight.setPosition(0);
            }
            if(unFlip){
                LiftflipLeft.setPosition(0.35);
                LiftflipRight.setPosition(0.65);
            }
            if(FlipStartPos){
                LiftflipLeft.setPosition(0);
                LiftflipRight.setPosition(1);
            }
            if(FlipPickUpPos){
                ClawflipR.setPosition(0.3);
                ClawflipL.setPosition(0.7);
            }
            else if(transferPos){
                ClawflipR.setPosition(0);
                ClawflipL.setPosition(1);
            }
            else if(clawStartPos){
                ClawflipR.setPosition(1);
                ClawflipL.setPosition(0);
            }
            if(clawFlip){
                ClawflipR.setPosition(0);
                ClawflipL.setPosition(1);
            }
            if(clawUnFlip){
                ClawflipR.setPosition(0);
                ClawflipL.setPosition(1);
            }
//            if(arm != 0){
//                motorArm.setTargetPosition(armPos);
//                motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motorArm.setPower(1);
//            }
//            else{
//                motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                armPos = motorArm.getCurrentPosition();
//
//            }
        }
    }
}