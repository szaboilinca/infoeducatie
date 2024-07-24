package org.firstinspires.ftc.teamcode.Advanced;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Diff Swerve TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    Robot robot;

    DcMotorEx LeftSlider,RightSlider;//extension motion
    AnalogInput servoport;
    Servo LeftLift,RightLift;

    DistanceSensor LeftDistance, RightDistance;
    TouchSensor SliderLimit,Palet;

    double extension;
    double retraction;
    boolean auto;

    //deadband for joysticks
    public double DEADBAND_MAG = 0.05;
    public Vector2d DEADBAND_VEC = new Vector2d(DEADBAND_MAG, DEADBAND_MAG);
    private GamepadEx g1;

    public boolean willResetIMU = true;
    private boolean fieldCentric = false;

    public static double p = 0.01, i = 0, d = 0;

    public void init() {
        robot = new Robot(this, false);
        g1 = new GamepadEx(gamepad1);
    }


    public void init_loop() {
        if (g1.wasJustPressed(GamepadKeys.Button.Y)) {
            willResetIMU = false;
        }
    }
    public void start () {
        if (willResetIMU) robot.initIMU();
    }


    public void loop() {
        g1.readButtons();

        Vector2d joystick1 = new Vector2d(g1.getLeftX(), g1.getLeftY()).exponential(3);
        Vector2d joystick2 = new Vector2d(g1.getRightX(), g1.getRightY()).exponential(3);

        robot.driveController.setPID(p, i, d);
        robot.driveController.updateUsingJoysticks(checkDeadband(joystick1), checkDeadband(joystick2), fieldCentric);

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
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            LeftLift.setPosition(0.5);
            RightLift.setPosition(0.5);
        }
        if(downlift){
            LeftLift.setPosition(0.4);
            RightLift.setPosition(0.6);
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
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


    public Vector2d checkDeadband(Vector2d joystick) {
        if (Math.abs(joystick.getX()) > DEADBAND_VEC.getX() || Math.abs(joystick.getY()) > DEADBAND_VEC.getY())
            return joystick;
        return Vector2d.ZERO;
    }




}