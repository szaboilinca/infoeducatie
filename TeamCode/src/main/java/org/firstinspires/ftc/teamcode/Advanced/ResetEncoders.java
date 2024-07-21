package org.firstinspires.ftc.teamcode.Advanced;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;



@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Reset Encoders", group = "Utilities")
public class ResetEncoders extends OpMode {
    Robot robot;

    public void init () {
        robot = new Robot(this, false);
    }

    public void loop () {
        telemetry.addData("LEFT Module Orientation: ", robot.driveController.moduleLeft.getCurrentOrientation().getAngle());
        telemetry.addData("RIGHT Module Orientation: ", robot.driveController.moduleRight.getCurrentOrientation().getAngle());
        telemetry.update();

        if (gamepad1.y) {
            robot.driveController.resetEncoders();
        }
    }
}
