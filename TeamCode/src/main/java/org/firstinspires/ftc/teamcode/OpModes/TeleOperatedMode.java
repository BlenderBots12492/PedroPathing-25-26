package org.firstinspires.ftc.teamcode.OpModes;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="TeleOp", group="test")
public class TeleOperatedMode extends LinearOpMode {
    ElapsedTime runtime;
    private Robot bot;
    public void runOpMode() {
        bot = new Robot(hardwareMap);
        bot.checkVoltage();
        runtime = new ElapsedTime();
        double gamepad1AbtnCooldown = -1.0;
        double gamepad1XbtnCooldown = -1.0;
        waitForStart();
        runtime.reset();
        bot.block();
        bot.launchAngleServo.setPosition(0.5383);
        while (opModeIsActive()) {
            telemetry.addData("left Vel", bot.launchWheelLeft.getVelocity());
            telemetry.addData("right Vel", bot.launchWheelRight.getVelocity());
            telemetry.addData("launch State", bot.launchState);

            telemetry.update();
            if (gamepad1.right_trigger > 0.5) {
                bot.setPowers(gamepad1.left_stick_y*0.5, gamepad1.left_stick_x*0.5, -gamepad1.right_stick_x*0.5);
            } else {
                bot.setPowers(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }

            if (gamepad2.aWasPressed()) {
                if (bot.launchState == Robot.launchStatus.IDLE) {
                    bot.startWheels();
                    gamepad1AbtnCooldown = runtime.seconds();
                }
            }
            if (gamepad2.bWasReleased()) {
                bot.launch();
            }
            if (gamepad2.xWasReleased()) {
                bot.intake();
            }
            if (abs(gamepad2.left_stick_y) > 0.2) {
                bot.changeAngle(gamepad2.left_stick_y);
            }
            telemetry.addData("launchAngle", bot.launchAngleServo.getPosition());

            if (gamepad2.yWasPressed() || gamepad1.yWasPressed()) {
                bot.stopWheels();
            }
        }
    }
}
