package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous
public class Auto extends LinearOpMode {
    private Robot bot;
    @Override
    public void runOpMode() {
        bot = new Robot(hardwareMap);
        bot.goTo();
    }
}
