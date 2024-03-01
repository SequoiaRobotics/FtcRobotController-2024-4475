package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.AutonomousAwareness;

@Deprecated
@Config
public class AutoRobotOpMode {

    public enum RobotMovement {
        FORWARD(1, 0, 0),
        BACKWARD(-1, 0, 0),
        LEFT(0, -1, 0),
        RIGHT(0, 1, 0),
        FORWARD1(1, 0, 0, 1d),
        BACKWARD1(-1, 0, 0, 1d),
        LEFT1(0, -1, 0, 1d),
        RIGHT1(0, 1, 0, 1d),
        TURN_LEFT90(-90f),
        TURN_RIGHT90(90f),
        TURN_LEFT45(-45f),
        TURN_RIGHT45(45f),
        TURN_LEFT180(-180f),
        TURN_RIGHT180(180f);

        final double TIME;
        /**
         * FORWARD AND BACKWARD
         */
        final double AXIAL;
        /**
         * STRAFING, SIDE TO SIDE
         */
        final double LATERAL;
        /**
         * ROTATION
         */
        final double YAW;

        public double getAngle() {
            return YAW;
        }

        RobotMovement() {
            this.YAW = 0;
            this.TIME = -1;
            this.AXIAL = 0;
            this.LATERAL = 0;
        }

        RobotMovement(double yaw) {
            this.YAW = yaw;
            this.TIME = -1;
            this.AXIAL = 0;
            this.LATERAL = 0;
        }

        RobotMovement(double yaw, double time) {
            this.YAW = yaw;
            this.TIME = time;
            this.AXIAL = 1;
            this.LATERAL = 0;
        }

        RobotMovement (double axial, double lateral, double yaw) {
            this.YAW = yaw;
            this.TIME = -1;
            this.AXIAL = axial;
            this.LATERAL = lateral;
        }

        RobotMovement (double axial, double lateral, double yaw, double time) {
            this.YAW = yaw;
            this.TIME = time;
            this.AXIAL = axial;
            this.LATERAL = lateral;
        }

        public double getTime() {
            return TIME;
        }

        public double getAxial() {
            return AXIAL;
        }

        public double getLateral() {
            return LATERAL;
        }

        public double getYaw() {
            return YAW;
        }

        public double[] getMovement() {
            return new double[] {AXIAL, LATERAL, YAW, TIME};
        }
    }

    public static RobotMovement[] movements;

    @Deprecated
    @Disabled
    @Autonomous(name = "AutoRobotOpMode")
    public static class AutonomousRobotOpMode extends RobotOpMode {
        private static boolean debugRunInitPath = true;
        public AutonomousAwareness AA;
        private static boolean shouldStop = false;
        @Override
        public void gamePadMoveRobot() {
            // Do nothing
        }

        @Override
        public void init() {
            super.init();
            Motor lFD;
            Motor rFD;
            Motor lBD;
            Motor rBD;
            // Temporary Fix
            try {
                lFD = new Motor(hardwareMap, "fl_drv");
                rFD = new Motor(hardwareMap, "fr_drv");
                lBD = new Motor(hardwareMap, "bl_drv");
                rBD = new Motor(hardwareMap, "br_drv");
            } catch(Exception e) {
                log("FATAL ERROR", "Could not initialize drive motors: "+e.getMessage());
                sendTelemetryPacket(true);
                terminateOpModeNow();
                return;
            }
            AA = new AutonomousAwareness(AutonomousAwareness.StartingPosition.RED_LEFT, false,
                    lFD, rFD, lBD, rBD,
                    this.encoderLeft, this.encoderRight, this.encoderBack);

            log("Init","Following Path");
            AA.addToPath(new StartWaypoint());
            AA.addToPath(new GeneralWaypoint(200, 0, 0.8, 0.8, 30));
            AA.addToPath(new EndWaypoint());
            AA.initPath();
            AA.followPath();
        }

        @Override
        public void robotLoop(double delta) {
            if (shouldStop) {
                log("Robot Loop", "Stopping");
                terminateOpModeNow();
            }
            if (debugRunInitPath) {
                log("Robot Loop", "Initializing Path");
                AA.initPath();
                AA.followPath();
                debugRunInitPath = false;
            }
        }

        @Override
        public void stop() {
            log("Stop", "Sending stop signal");
            shouldStop = true;
        }
    }
}