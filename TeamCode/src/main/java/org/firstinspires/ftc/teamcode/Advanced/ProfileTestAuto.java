package org.firstinspires.ftc.teamcode.Advanced;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous(group = "test")
public class ProfileTestAuto extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, false);

        AutoManager a1 = new AutoManager(this)
                //.AddPoint(new Vector2d(1,0))
                .AddPoint(new Vector2d(0, 1))
                //.AddPoint(new Vector2d(0, 3))
                //.AddPoint(new Vector2d(1, 3))
                //.AddPoint(new Vector2d(1, 2))
                .Build();

        waitForStart();

        a1.Follow(robot);
    }
}
