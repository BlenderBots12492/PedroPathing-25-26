package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.visionThread.VisionThread;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Robot;
@TeleOp(name="Tele-Operated Mode", group="test")
public class TeleOperatedMode extends LinearOpMode {
    private Robot bot;
    public void runOpMode() {
        bot = new Robot();
        waitForStart();
        while (opModeIsActive()) {
            bot.setPowers(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            if (gamepad1.a) {
                if (bot.launchState == Robot.launchStatus.IDLE) {
                    bot.startWheels();
                } else {
                    bot.stopWheels();
                }
            }
            if (gamepad1.b) {
                bot.launch();
            }
        }
    }
}
