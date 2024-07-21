package org.firstinspires.ftc.teamcode.Advanced;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous(name = "Auto_InfoEdu", group = "test")
public class Auto_InfoEdu extends LinearOpMode {
    private Robot robot;
    ColorSensor color;
    @Override
    public void runOpMode() throws InterruptedException {
        color = hardwareMap.get(ColorSensor.class, "Color");
        waitForStart();
            robot = new Robot(this, false);

            AutoManager auto1 = new AutoManager(this)
                    .AddPoint(new Vector2d(0,1))
                    .Build();

            auto1.Follow(robot);
            int redValue = color.red();
            int greenValue = color.green();
            int blueValue = color.blue();

            if (redValue > greenValue && redValue > blueValue) {
                AutoManager auto2 = new AutoManager(this)
                        .AddPoint(new Vector2d(0,0))
                        .AddPoint(new Vector2d(-2, -1))
                        .AddPoint(new Vector2d(-2.5, -1.5))
                        .AddPoint(new Vector2d(-1.5, -2))
                        .AddPoint(new Vector2d(-1.5, -3))
                        .AddPoint(new Vector2d(-2.5, -3))
                        .Build();
                auto2.Follow(robot);
                telemetry.addData("Color", "Red");
            }
            // Check if the sensor detects green
            else if (greenValue > redValue && greenValue > blueValue) {
                AutoManager auto2 = new AutoManager(this)
                        .AddPoint(new Vector2d(0,-1))
                        .AddPoint(new Vector2d(0.5, -1.5))
                        .AddPoint(new Vector2d(-0.5, -2))
                        .AddPoint(new Vector2d(-0.5, -3))
                        .AddPoint(new Vector2d(0.5, -3))
                        .Build();
                auto2.Follow(robot);
                telemetry.addData("Color", "Green");
            }
            // Check if the sensor detects blue
            else if (blueValue > redValue && blueValue > greenValue) {
                AutoManager auto2 = new AutoManager(this)
                        .AddPoint(new Vector2d(0,0))
                        .AddPoint(new Vector2d(2, -1))
                        .AddPoint(new Vector2d(2.5, -1.5))
                        .AddPoint(new Vector2d(1.5, -2))
                        .AddPoint(new Vector2d(1.5, -3))
                        .AddPoint(new Vector2d(2.5, -3))
                        .Build();
                auto2.Follow(robot);
                telemetry.addData("Color", "Blue");
            } else {
                telemetry.addData("Color", "Unknown");
            }
            telemetry.update();

            sleep(100);


    }
}
