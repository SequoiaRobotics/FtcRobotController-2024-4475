package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Point;

import org.firstinspires.ftc.teamcode.bots.UltimateBotI;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;

public class AutoReplayBase extends AutoBase {

    @Override
    protected void preStart() {
        super.preStart();
        String routeName = getModeName();
        if (!routeName.isEmpty()) {
            loadRoute(routeName);
            if (this.selectedRoute != null){
                this.setOpModeSide(this.selectedRoute.getName());
            }
        }
    }

    @Override
    protected void act() {
        super.act();
        if (opModeIsActive()) {
            runRoute(true);
        }
    }

    @Override
    protected void initBot() {
        this.bot = new UltimateBotI();
        bot.initDetectorThread(this.getOpModeSide(), this);
        ((UltimateBotI)bot).cameraInitAuto();
    }

    @Override
    protected void initLocator() {
        this.locator = new RobotCoordinatePosition(bot, new Point(startX, startY), initHead, RobotCoordinatePosition.THREAD_INTERVAL);
        locator.reverseHorEncoder();
        locator.setPersistPosition(true);
        startLocator(locator);
    }
}
