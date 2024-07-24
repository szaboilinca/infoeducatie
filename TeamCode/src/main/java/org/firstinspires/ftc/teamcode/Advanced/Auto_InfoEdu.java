package org.firstinspires.ftc.teamcode.Advanced;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous(name = "Auto_InfoEdu", group = "test")
public class Auto_InfoEdu extends LinearOpMode {

    private Robot robot;
    ColorSensor color;

    DcMotorEx LeftSlider,RightSlider;//extension motion
    AnalogInput servoport;
    Servo LeftLift,RightLift;

    DistanceSensor LeftDistance, RightDistance;
    TouchSensor SliderLimit,Palet;

    double extension;
    double retraction;
    boolean auto;

    @Override
    public void runOpMode() throws InterruptedException {

        color = hardwareMap.get(ColorSensor.class, "Color");

        //HARDWARE MAP
        LeftSlider=hardwareMap.get(DcMotorEx.class, "LeftSlider");
        RightSlider=hardwareMap.get(DcMotorEx.class, "RightSlider");
        RightLift=hardwareMap.get(Servo.class, "RightLift");
        LeftLift=hardwareMap.get(Servo.class, "LeftLift");
        servoport=hardwareMap.get(AnalogInput.class,"servoport");
        LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");
        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");
        SliderLimit = hardwareMap.get(TouchSensor.class, "SliderLimit");
        Palet = hardwareMap.get(TouchSensor.class, "Palet");

        //SET DIRECTION
        LeftSlider.setDirection(DcMotorSimple.Direction.FORWARD);
        RightSlider.setDirection(DcMotorSimple.Direction.FORWARD);


        LeftSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //ENCODER
        LeftSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //BRAKE
        LeftSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        robot = new Robot(this, false);

        AutoManager auto1 = new AutoManager(this)
                    .AddPoint(new Vector2d(0,1))
                    .Build();


        auto1.Follow(robot);
        int redValue = color.red();
        int greenValue = color.green();
        int blueValue = color.blue();

        //extindere si ridicare palet
        while (LeftSlider.getCurrentPosition() < 4000) {
            LeftSlider.setPower(1);
            RightSlider.setPower(1);
        }

        // lift up
        LeftLift.setPosition(0.6);
        RightLift.setPosition(0.4);
        Thread.sleep(300);
        LeftLift.setPosition(0.5);
        RightLift.setPosition(0.5);

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
        } else { telemetry.addData("Color", "Unknown"); }

        telemetry.update();

        //lasat jos

        // lift up
        LeftLift.setPosition(0.4);
        RightLift.setPosition(0.6);
        Thread.sleep(300);
        LeftLift.setPosition(0.5);
        RightLift.setPosition(0.5);
        //extindere si ridicare palet
        while (LeftSlider.getCurrentPosition() >0) {
            LeftSlider.setPower(-1);
            RightSlider.setPower(-1);
        }



        sleep(100);

    }
}
