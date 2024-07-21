package org.firstinspires.ftc.teamcode.Advanced;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

enum ModuleSide {LEFT, RIGHT}

public class DriveController {
    Robot robot;

    DriveModule moduleLeft;
    DriveModule moduleRight;

    double robotDistanceTraveled = 0;
    double previousRobotDistanceTraveled = 0;
    double moduleLeftLastDistance;
    double moduleRightLastDistance;

    //tolerante
    public final double ALLOWED_MODULE_ROT_ERROR = 5;

    public final double START_DRIVE_SLOWDOWN_AT_CM = 15;

    //nr max incercari corectare
    public final int MAX_ITERATIONS_ROBOT_ROTATE = 2;


    public final double ROBOT_ROTATION_SCALE_FACTOR = 0.7;

    public DriveController(Robot robot) {
        this.robot = robot;
        moduleLeft = new DriveModule(robot, ModuleSide.LEFT);
        moduleRight = new DriveModule(robot, ModuleSide.RIGHT);

        moduleLeftLastDistance = moduleLeft.getDistanceTraveled();
        moduleRightLastDistance = moduleRight.getDistanceTraveled();
    }


    public void updateUsingJoysticks(Vector2d joystick1, Vector2d joystick2, boolean fieldCentric) {
        update(joystick1, -joystick2.getX() * ROBOT_ROTATION_SCALE_FACTOR, fieldCentric);
    }


    public void update(Vector2d translationVector, double rotationMagnitude, boolean fieldCentric) {
        moduleLeft.updateTarget(translationVector, rotationMagnitude, fieldCentric);
        moduleRight.updateTarget(translationVector, rotationMagnitude, fieldCentric);
    }


    public void drive(Vector2d direction, double cmDistance, double speed, LinearOpMode linearOpMode) {

        resetDistanceTraveled();
        while (getDistanceTraveled() < cmDistance && linearOpMode.opModeIsActive()) {

            if (cmDistance - getDistanceTraveled() < START_DRIVE_SLOWDOWN_AT_CM) {

            }
            updateTracking();
            update(direction.normalize(speed), 0, true);

            linearOpMode.telemetry.addData("Driving robot", "");
            linearOpMode.telemetry.update();
        }
        update(Vector2d.ZERO, 0, true);
    }

    public void rotateRobot(Angle targetAngle, LinearOpMode linearOpMode) {

        int iterations = 0;
        boolean isNegativeRotation = robot.getRobotHeading().directionTo(targetAngle) == Angle.Direction.CLOCKWISE;

        double absHeadingDiff = robot.getRobotHeading().getDifference(targetAngle);
        while (absHeadingDiff > ALLOWED_MODULE_ROT_ERROR && linearOpMode.opModeIsActive() && iterations < MAX_ITERATIONS_ROBOT_ROTATE /*&& System.currentTimeMillis() - startTime < ROTATE_ROBOT_TIMEOUT*/) {
            absHeadingDiff = robot.getRobotHeading().getDifference(targetAngle);
            double rotMag = RobotUtil.scaleVal(absHeadingDiff, 0, 25, 0, 0.7);

            if (robot.getRobotHeading().directionTo(targetAngle) == Angle.Direction.CLOCKWISE) {
                update(Vector2d.ZERO, -rotMag, true);
                if (!isNegativeRotation) iterations++;
            } else {
                update(Vector2d.ZERO, rotMag, true);
                if (isNegativeRotation) iterations++;
            }
            linearOpMode.telemetry.addData("Rotating ROBOT", "");
            linearOpMode.telemetry.update();
        }
        update(Vector2d.ZERO, 0, true);
    }


    public void rotateModules(Vector2d direction, boolean fieldCentric, double timemoutMS, LinearOpMode linearOpMode) {
        //TODO: check if this will work with reversed modules
        double moduleLeftDifference, moduleRightDifference;
        double startTime = System.currentTimeMillis();
        do {
            moduleLeftDifference = moduleLeft.getCurrentOrientation().getDifference(direction.getAngle());
            moduleRightDifference = moduleRight.getCurrentOrientation().getDifference(direction.getAngle());
            moduleLeft.rotateModule(direction, fieldCentric);
            moduleLeft.rotateModule(direction, fieldCentric);
            moduleRight.rotateModule(direction, fieldCentric);

            linearOpMode.telemetry.addData("Rotating MODULES", "");
            linearOpMode.telemetry.update();
        } while ((moduleLeftDifference > ALLOWED_MODULE_ROT_ERROR || moduleRightDifference > ALLOWED_MODULE_ROT_ERROR) && linearOpMode.opModeIsActive() && System.currentTimeMillis() < startTime + timemoutMS);
        update(Vector2d.ZERO, 0, true);
    }




    public void resetDistanceTraveled() {
        previousRobotDistanceTraveled = robotDistanceTraveled;
        robotDistanceTraveled = 0;

        moduleRight.resetDistanceTraveled();
        moduleLeft.resetDistanceTraveled();
    }

    public void updateTracking() {
        moduleRight.updateTracking();
        moduleLeft.updateTracking();

        double moduleLeftChange = moduleLeft.getDistanceTraveled() - moduleLeftLastDistance;
        double moduleRightChange = moduleRight.getDistanceTraveled() - moduleRightLastDistance;
        robotDistanceTraveled += (moduleLeftChange + moduleRightChange) / 2;

        moduleLeftLastDistance = moduleLeft.getDistanceTraveled();
        moduleRightLastDistance = moduleRight.getDistanceTraveled();
    }

    public void setPID(double p, double i, double d) {
        moduleLeft.setPID(p, i, d);
        moduleRight.setPID(p, i, d);
    }

    public double getDistanceTraveled() {
        return Math.abs(robotDistanceTraveled);
    }

    public void resetEncoders() {
        moduleRight.resetEncoders();
        moduleLeft.resetEncoders();
    }
}