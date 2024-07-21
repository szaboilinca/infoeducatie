package org.firstinspires.ftc.teamcode.Advanced;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
        (name = "Diff Swerve TeleOp", group = "TeleOp")
public class TeleOpExpansion extends LinearOpMode {

    DcMotorEx LeftSlider,RightSlider;//extension motion
    AnalogInput servoport;
    Servo LeftLift,RightLift;

    DistanceSensor LeftDistance, RightDistance;
    TouchSensor SliderLimit,Palet;

    double extension;
    double retraction;
    boolean auto;

    @Override
    public void runOpMode() throws InterruptedException{

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
        while (opModeIsActive() && !isStopRequested()){
            //BASIC MOTION FUNCTIONS
            double pivot = -gamepad1.left_stick_y *0.5;
            double vertical = gamepad1.left_stick_x *0.5;
            double turn = gamepad1.right_stick_x *0.5;


            //SLIDERS
            if(LeftSlider.getCurrentPosition() < 4000 && RightSlider.getCurrentPosition() > -4000)
                extension = gamepad1.right_trigger;
            else
                extension = 0;

            if(LeftSlider.getCurrentPosition() > 0 && RightSlider.getCurrentPosition() < 0)
                retraction = gamepad1.left_trigger;
            else
                retraction = 0;



            //SLIDER POWER
            LeftSlider.setPower(extension - retraction);
            RightSlider.setPower(-extension + retraction);


            //LIFT
            boolean uplift = gamepad1.right_bumper;
            boolean downlift = gamepad1.left_bumper;
            if(uplift){
                LeftLift.setPosition(0.6);
                RightLift.setPosition(0.4);
                Thread.sleep(300);
                LeftLift.setPosition(0.5);
                RightLift.setPosition(0.5);
            }
            if(downlift){
                LeftLift.setPosition(0.4);
                RightLift.setPosition(0.6);
                Thread.sleep(300);
                LeftLift.setPosition(0.5);
                RightLift.setPosition(0.5);
            }

            if(SliderLimit.isPressed()){
                LeftSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RightSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LeftSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RightSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if(Palet.isPressed()) {
                auto = true;
            }
            if(auto){
                LeftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftSlider.setPower(0.5);
                LeftSlider.setTargetPosition(4000);
                RightSlider.setPower(0.5);
                RightSlider.setTargetPosition(-4000);
                if(LeftSlider.getCurrentPosition()>=4000 && RightSlider.getCurrentPosition()<=-4000){
                    LeftSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    RightSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    gamepad1.rumble(1, 1,500);
                    auto=false;
                }

            }



            if(LeftDistance.getDistance(DistanceUnit.CM) < 5)
                gamepad1.rumble(1,0,500);

            if(RightDistance.getDistance(DistanceUnit.CM) < 5)
                gamepad1.rumble(0,1,500);

            //TELEMETRY
            telemetry.addData("Left Slider Position: ", LeftSlider.getCurrentPosition());
            telemetry.addData("Right Slider Position ", RightSlider.getCurrentPosition());
            telemetry.addData("Left Distance", LeftDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Distance", RightDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Slider Limit is pressed: ", SliderLimit.isPressed());

            telemetry.update();
        }
    }
}