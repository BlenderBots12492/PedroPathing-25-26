package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.field.Line;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.tools.tools;

public class testCameras extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        bot.initilizeCamera();
        bot.initilizePedroPathing(tools.Team.RED);
        while (opModeInInit()) {
            Pose position = bot.getPosition();
            if (position != null) {
                telemetry.addData("x", position.getX());
                telemetry.addData("y", position.getY());
                telemetry.addData("heading", position.getHeading());
                telemetry.update();
            }

        }
        waitForStart();


    }
}
