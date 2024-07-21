package org.firstinspires.ftc.teamcode.Advanced;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Diff Swerve TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    Robot robot;

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


    }


    public Vector2d checkDeadband(Vector2d joystick) {
        if (Math.abs(joystick.getX()) > DEADBAND_VEC.getX() || Math.abs(joystick.getY()) > DEADBAND_VEC.getY())
            return joystick;
        return Vector2d.ZERO;
    }




}