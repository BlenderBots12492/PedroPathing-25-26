package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.telecom.Call;

import com.bylazar.field.Line;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.tools.tools;

@Autonomous(name="Red Auto")
public class Auto extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        //prep for op mode
        Robot bot = new Robot(hardwareMap);

        //set up camera
        bot.initilizeCamera();
        //set up math to make it move
        bot.initilizePedroPathing(tools.Team.RED);

        PathChain initialLaunchPath;
        while (opModeInInit()) {
            Pose position = bot.getPosition();
            telemetry.addData("Position Updated", bot.vision.getLastPosition() != null);
            telemetry.addData("Vx", bot.vision.getLastPosition().getPosition().x);
            telemetry.addData("Vy", bot.vision.getLastPosition().getPosition().y);
            telemetry.addData("x", position.getX());
            telemetry.addData("y", position.getY());
            telemetry.addData("heading", position.getHeading());
            telemetry.update();
            bot.follower.update();

        }
        initialLaunchPath = bot.follower.pathBuilder()
                .addPath(new BezierLine(bot.follower.getPose(), bot.scorePose))
                .setLinearHeadingInterpolation(bot.follower.getHeading(), bot.scorePose.getHeading())
                .build();

        //start opMode
        waitForStart();
        bot.startWheels();
        //move according to path
        bot.follower.followPath(initialLaunchPath, true);
        //launch
        for (int i = 0; i < 3; i++) {
            //while robot busy and ready
            while (bot.follower.isBusy() && bot.launchState == Robot.launchStatus.READY) {
                bot.follower.update();
            }
            bot.launch();
            sleep(2000);
        }
        for (int i = 0; i < 3; i++) {
            bot.follower.followPath(bot.pickUpPaths[i], true);
            for (int j = 0; j < 3; j++) {
                while (bot.follower.isBusy() && bot.launchState == Robot.launchStatus.READY) {
                    bot.follower.update();
                }
                bot.launch();
                sleep(2000);
            }
        }

    }
}
