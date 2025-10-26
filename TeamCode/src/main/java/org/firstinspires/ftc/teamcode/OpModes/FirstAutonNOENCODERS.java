package org.firstinspires.ftc.teamcode.OpModes;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name="Auton SIMPLE", group="test")
public class FirstAutonNOENCODERS extends LinearOpMode {
    ElapsedTime runtime;
    private Robot bot;
    public void runOpMode() {
        bot = new Robot(hardwareMap);
        bot.checkVoltage();
        runtime = new ElapsedTime();
        waitForStart();
        runtime.reset();
        bot.setPowers(-0.75, 0, 0);
        sleep(7000);
        bot.setPowers(0, 0, 0);
    }
}
