package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Impasta;

@TeleOp
public class ImpastaTeleOp extends LinearOpMode {
    // Declaring hardware variables
    private DcMotor fl, fr, bl, br, leftSlide, rightSlide, Winch, Intake;
    private Servo out1, out2, launchPlane, aimLauncher;
    private Servo DRV4BL, DRV4BR;
    private DistanceSensor leftSensor;
    private DistanceSensor rightSensor;
    private AHRS imu;
    private double up, down, current, distance;
    Impasta impasta;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initializing motors, servos, and sensors
        fl = hardwareMap.dcMotor.get("leftFront"); //Drivebase
        fr = hardwareMap.dcMotor.get("rightFront"); //Drivebase
        bl = hardwareMap.dcMotor.get("leftRear"); //Drivebase
        br = hardwareMap.dcMotor.get("rightRear"); //Drivebase

        leftSlide = hardwareMap.dcMotor.get("frontEncoder"); //Slides
        rightSlide = hardwareMap.dcMotor.get("Right Slide"); //Slides

        Intake = hardwareMap.dcMotor.get("leftEncoder"); //Pixel Intake

        DRV4BL = hardwareMap.servo.get("leftV4B"); //Virtual Four Bar Servos // Left Side
        DRV4BR = hardwareMap.servo.get("rightV4B"); //Virtual Four Bar Servos //Right Side

        launchPlane = hardwareMap.servo.get("launcher");

        out1 = hardwareMap.servo.get("leftOut"); //Outtake
        out2 = hardwareMap.servo.get("rightOut"); //Outtake

//        leftSensor = hardwareMap.get(DistanceSensor.class, "leftSensor");
//        rightSensor = hardwareMap.get(DistanceSensor.class, "rightSensor");

        distance = 1;

        imu = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData);

        // Creating an instance of the Impasta class
        impasta = new Impasta(fl, fr, bl, br, leftSlide, rightSlide, Intake, imu);

        impasta.reset();
        boolean reset = true;

        boolean DRV4BReset = false;

        telemetry.addLine("Initialization Done, pos reset: " + reset + " DRV4B reset: " + DRV4BReset);
        telemetry.update();

        // Waiting for the start button to be pressed
        waitForStart();

        if (isStopRequested()) return;
        // Main TeleOp loop
        while (opModeIsActive()) {
            /** gamepad1                                                                                */
            // Driving the robot based on gamepad input
            impasta.driveBaseField(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, -gamepad1.right_stick_x);

            // Controlling intake based on trigger input
            impasta.intake(gamepad1.left_trigger - gamepad1.right_trigger);

            // Resetting IMU yaw angle if left bumper is pressed
            if (gamepad1.left_bumper || gamepad1.right_bumper) {
                impasta.reset();
                gamepad1.rumble(2000);
            }

            if (gamepad2.dpad_down && !impasta.atLower()) {
                impasta.runToPos(0);
            } else if (gamepad2.dpad_left) {
                impasta.runToPos(150);
            } else if (gamepad2.dpad_right) {
                impasta.runToPos(300);
            } else if (gamepad2.dpad_up && !impasta.atUpper()) {
                impasta.runToPos(450);
            } else {
                impasta.setSlidesPower(gamepad2.left_stick_y);
            }

//            //TODO Test Airplane Aim
            if (gamepad2.triangle) {
                launchPlane.setPosition(0);
            } else if (gamepad2.square) {
                launchPlane.setPosition(1);
            }
//
//            current = aimLauncher.getPosition();
//            if (gamepad2.dpad_up) {
//                aimLauncher.setPosition(current + 0.1);
//            } else if (gamepad2.dpad_down) {
//                aimLauncher.setPosition(current - 0.1);
//            }

            // Controlling the Resting/Scoring state of the Virtual Four Bar
//            if (gamepad2.circle) {
//                // Rotate the CR servos clockwise as long as the button is pressed
//                while (gamepad2.circle) {
//                    DRV4BL.setPower(1.0);  // Adjust the power as needed
//                    DRV4BR.setPower(1.0);  // Adjust the power as needed
//                }
//                // Stop the servos when the button is released
//                DRV4BL.setPower(0);
//                DRV4BR.setPower(0);
//
//            } else if (gamepad2.cross) {
//                // Rotate the CR servos counterclockwise as long as the button is pressed
//                while (gamepad2.cross) {
//                    DRV4BL.setPower(-1.0);  // Adjust the power as needed
//                    DRV4BR.setPower(-1.0);  // Adjust the power as needed
//                }
//                // Stop the servos when the button is released
//                DRV4BL.setPower(0);
//                DRV4BR.setPower(0);
//
//            }

            if (gamepad2.circle) {
                DRV4BL.setPosition(180);
                DRV4BR.setPosition(180);
            } else if (gamepad2.cross) {
                DRV4BL.setPosition(0);
                DRV4BR.setPosition(0);
            }

            telemetry.addLine("right" + DRV4BR.getPosition() + "\n left" + DRV4BL.getPosition());
            telemetry.update();

//            ElapsedTime time = new ElapsedTime();
//            double servoCurrentR = DRV4BR.getPosition();
//            double servoCurrentL = DRV4BL.getPosition();
//            DRV4BR.setPosition(0.01 * (time.seconds() + 1) * gamepad2.right_stick_y + servoCurrentR);
//            DRV4BL.setPosition(0.01 * (time.seconds() + 1) * gamepad2.right_stick_y + servoCurrentL);
//            time.reset();
//
//            telemetry.addLine("LeftServoPos: " + DRV4BR.getPo());
//            telemetry.addLine("RightServoPos: " + DRV4BL.getPosition());
//            telemetry.update();

            // Outtake switches between scoring and rest position based on button press //Swap later
            if (gamepad2.left_trigger > 0.3 /* || leftSensor.getDistance(DistanceUnit.INCH)<=distance */) {
                gamepad1.rumble(1000);
                out1.setPosition(0.75); // left //lower
            } else {
                out1.setPosition(0.55); // left //raise
            }

            if (gamepad2.right_trigger > 0.3 /* || rightSensor.getDistance(DistanceUnit.INCH)<=distance */) {
                gamepad1.rumble(1000);
                out2.setPosition(0.5); // right //lower
            } else {
                out2.setPosition(0.6); // right //raise
            }
        }
    }
}
