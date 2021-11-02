package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bots.FrenzyBotI;
import org.firstinspires.ftc.teamcode.odometry.VSlamOdometry;

@TeleOp(name="Master Odo Cam", group="Robot15173")
public class MasterOdoCam extends MasterOdo {
    @Override
    protected void initBot() {
        this.bot = new FrenzyBotI();
    }

    @Override
    protected void initLocator() {
        if (locator == null) {
            this.locator = VSlamOdometry.getInstance(hardwareMap, VSlamOdometry.THREAD_INTERVAL, startX, startY, (int) initHead);
            startLocator(locator);
        }
    }

}
