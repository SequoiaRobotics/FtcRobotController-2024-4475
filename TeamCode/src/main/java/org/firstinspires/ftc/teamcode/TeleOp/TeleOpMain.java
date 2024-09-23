package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSub;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

@TeleOp
@Config
public class TeleOpMain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Intake intake = new Intake(this);
        MecanumDrive drive = new MecanumDrive(this);
        Arm arm = new Arm(this);
        Slides slides = new Slides(this);

        waitForStart();
        
        drive.teleOp();
        intake.teleOp();
        arm.teleOp();
        slides.teleOp();
    }
}
