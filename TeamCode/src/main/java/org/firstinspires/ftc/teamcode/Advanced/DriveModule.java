package org.firstinspires.ftc.teamcode.Advanced;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveModule {
    Robot robot;


    DcMotor motor1; //top motor
    DcMotor motor2; //bottom motor

    public final ModuleSide moduleSide;
    public final Vector2d positionVector;


    public boolean takingShortestPath = false;
    public boolean reversed = false;


    public final double TICKS_PER_MODULE_REV = 2800; //ticks per MODULE revolution
    public final double DEGREES_PER_TICK = 360/TICKS_PER_MODULE_REV;

    public final double TICKS_PER_WHEEL_REV = 537.7 * (double)(27)/70 * (double)(60)/20; //ticks per WHEEL revolution

    public final double CM_WHEEL_DIAMETER = 11;
    public final double CM_PER_WHEEL_REV = CM_WHEEL_DIAMETER * Math.PI;
    public final double CM_PER_TICK = CM_PER_WHEEL_REV/TICKS_PER_WHEEL_REV;


    public final double ANGLE_OF_MAX_MODULE_ROTATION_POWER = 40;


    public final double ALLOWED_MODULE_ORIENTATION_ERROR = 5;


    public double ROT_ADVANTAGE = 2;


    public double MAX_MOTOR_POWER = 0.3;


    public final Vector2d MOTOR_1_VECTOR = new Vector2d(1/Math.sqrt(2), 1/Math.sqrt(2));
    public final Vector2d MOTOR_2_VECTOR = new Vector2d(-1/Math.sqrt(2), 1/Math.sqrt(2));


    private double distanceTraveled;
    private double lastMotor1Encoder;
    private double lastMotor2Encoder;

    private double target_heading = 0;
    private PIDController headController;
    private Telemetry dashboardTelemetry;

    private final double p = 0.05, i = 0, d = 0;


    public DriveModule(Robot robot, ModuleSide moduleSide) {
        this.robot = robot;
        this.moduleSide = moduleSide;
        if (moduleSide == ModuleSide.RIGHT) {
            motor1 = robot.hardwareMap.dcMotor.get(HardwareNames.RightUp);
            motor2 = robot.hardwareMap.dcMotor.get(HardwareNames.RightDown);
            positionVector = new Vector2d((double)18/2, 0); //points from robot center to right module
        } else {
            motor1 = robot.hardwareMap.dcMotor.get(HardwareNames.LeftUp);
            motor2 = robot.hardwareMap.dcMotor.get(HardwareNames.LeftDown);
            positionVector = new Vector2d((double)-18/2, 0); //points from robot center to left module
        }

        headController = new PIDController(p, i, d);
        dashboardTelemetry = robot.telemetry;

        lastMotor1Encoder = motor1.getCurrentPosition();
        lastMotor2Encoder = motor2.getCurrentPosition();


        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPID(double p, double i, double d) { headController.setPID(p, i, d); }


    public void updateTarget (Vector2d transVec, double rotMag, boolean fieldCentric) {

        double current_heading = robot.getRobotHeading().getAngle(Angle.AngleType.ZERO_TO_360_HEADING);

        if (rotMag != 0)
            target_heading = current_heading;

        double error = target_heading - current_heading;
        double error_abs = Math.abs(error);
        double correction = 0;

        // find shortest path
        if (error_abs <= 360 - error_abs) error = -error;
        else error = Math.signum(error) * 360 - error;

        if (Math.abs(error) > 2)
            correction = headController.calculate(error);

        rotMag += correction;

        if (fieldCentric)
            transVec = transVec.rotateBy(current_heading, Angle.Direction.COUNTER_CLOCKWISE);


        Vector2d rotVec = positionVector.normalize(rotMag).rotateBy(90, Angle.Direction.COUNTER_CLOCKWISE); //theoretically this should be rotated 90, not sure sure it doesn't need to be


        Vector2d targetVector = transVec.add(rotVec);


        int directionMultiplier = -1;
        if (reversed) {
            targetVector = targetVector.reflect();
            directionMultiplier = 1;
        }


        goToTarget(targetVector, directionMultiplier);

        dashboardTelemetry.addData("Target heading: ", target_heading);
        dashboardTelemetry.addData("Current heading: ", current_heading);
        dashboardTelemetry.addData("error: ", error);
        dashboardTelemetry.addData("PID correction: ", correction);
        dashboardTelemetry.update();

    }


    //sets motor powers for robot to best approach given target vector
    public void goToTarget (Vector2d targetVector, int directionMultiplier) {
        //how much the module needs to translate (and in which direction)
        double moveComponent = targetVector.getMagnitude() * directionMultiplier;

        //how much the module needs to pivot (change its orientation)
        double pivotComponent;
        if (targetVector.getMagnitude() != 0) {
            pivotComponent = getPivotComponent(targetVector, getCurrentOrientation());
        } else {
            //if target vector is zero (joystick is within deadband) don't pivot modules
            pivotComponent = 0;
        }

        //vector in an (invented) coordinate system that represents desired (relative) module translation and module rotation
        Vector2d powerVector = new Vector2d(moveComponent, pivotComponent); //order very important here
        setMotorPowers(powerVector);

    }



    public double getPivotComponent (Vector2d targetVector, Angle currentAngle) {
        Angle targetAngle = targetVector.getAngle();
        double angleDiff = targetAngle.getDifference(currentAngle); //number from 0 to 180 (always positive)


        if (Math.abs(angleDiff) > 110) { //was 90
            if (!takingShortestPath) {
                reversed = !reversed; //reverse translation direction bc module is newly reversed
            }
            takingShortestPath = true;
        } else {
            takingShortestPath = false;
        }

        Angle.Direction direction = currentAngle.directionTo(targetAngle);


        if (angleDiff < ALLOWED_MODULE_ORIENTATION_ERROR) {

            return 0;
        } else if (angleDiff > ANGLE_OF_MAX_MODULE_ROTATION_POWER) {

            if (direction == Angle.Direction.CLOCKWISE) return ROT_ADVANTAGE;
            else return -1 * ROT_ADVANTAGE;
        } else {

            if (direction == Angle.Direction.CLOCKWISE) return angleDiff / ANGLE_OF_MAX_MODULE_ROTATION_POWER * ROT_ADVANTAGE;
            else return -1 * angleDiff / ANGLE_OF_MAX_MODULE_ROTATION_POWER * ROT_ADVANTAGE;
        }
    }



    public void setMotorPowers (Vector2d powerVector) {


        Vector2d motor1Unscaled = powerVector.projection(MOTOR_1_VECTOR);
        Vector2d motor2Unscaled = powerVector.projection(MOTOR_2_VECTOR);


        Vector2d[] motorPowersScaled = Vector2d.batchNormalize(MAX_MOTOR_POWER, motor1Unscaled, motor2Unscaled);
        double motor1power = motorPowersScaled[0].getMagnitude();
        double motor2power = motorPowersScaled[1].getMagnitude();

        //this is to add sign to magnitude, which returns an absolute value
        if (motorPowersScaled[0].getAngleDouble(Angle.AngleType.NEG_180_TO_180_CARTESIAN) != MOTOR_1_VECTOR.getAngleDouble(Angle.AngleType.NEG_180_TO_180_CARTESIAN)) {
            motor1power *= -1;
        }
        if (motorPowersScaled[1].getAngleDouble(Angle.AngleType.NEG_180_TO_180_CARTESIAN) != MOTOR_2_VECTOR.getAngleDouble(Angle.AngleType.NEG_180_TO_180_CARTESIAN)) {
            motor2power *= -1;
        }


        motor1.setPower(motor1power);
        motor2.setPower(motor2power);
    }



    public void rotateModule (Vector2d direction, boolean fieldCentric) {

        Angle convertedRobotHeading = robot.getRobotHeading().convertAngle(Angle.AngleType.NEG_180_TO_180_CARTESIAN);


        Vector2d directionFC = direction.rotateTo(robot.getRobotHeading()); //was converted robot heading


        if (reversed) { //reverse direction of translation because module is reversed
            directionFC = directionFC.reflect();
            direction = direction.reflect();
        }

        Vector2d powerVector;
        if (fieldCentric) {
            powerVector = new Vector2d(0, getPivotComponent(directionFC, getCurrentOrientation())); //order important here
        } else {
            powerVector = new Vector2d(0, getPivotComponent(direction, getCurrentOrientation())); //order important here
        }
        setMotorPowers(powerVector);

    }


    public void resetEncoders () {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public Angle getCurrentOrientation() {
        robot.telemetry.addData(moduleSide + "Motor 1 Encoder", motor1.getCurrentPosition());
        robot.telemetry.addData(moduleSide + "Motor 2 Encoder", motor2.getCurrentPosition());
        double rawAngle = (double)(motor2.getCurrentPosition() + motor1.getCurrentPosition())* DEGREES_PER_TICK; //motor2-motor1 makes ccw positive (?)
        return new Angle(rawAngle, Angle.AngleType.ZERO_TO_360_HEADING);
    }


    //TRACKING METHODS
    //used for straight line distance tracking

    public void updateTracking () {

        double currentMotor1Encoder = motor1.getCurrentPosition();
        double currentMotor2Encoder = motor2.getCurrentPosition();

        double motor1Change = currentMotor1Encoder - lastMotor1Encoder;
        double motor2Change = currentMotor2Encoder - lastMotor2Encoder;


        if (reversed) {
            distanceTraveled -= (motor1Change - motor2Change)/2.0 * CM_PER_TICK;
        } else {
            distanceTraveled += (motor1Change - motor2Change)/2.0 * CM_PER_TICK;
        }

        lastMotor1Encoder = currentMotor1Encoder;
        lastMotor2Encoder = currentMotor2Encoder;


        robot.telemetry.update();
    }

    public void resetDistanceTraveled () {
        distanceTraveled = 0;
        lastMotor1Encoder = motor1.getCurrentPosition();
        lastMotor2Encoder = motor2.getCurrentPosition();
    }

    //returns distance (in cm) traveled since distance was last reset
    public double getDistanceTraveled () {
        return distanceTraveled;
    }
}
