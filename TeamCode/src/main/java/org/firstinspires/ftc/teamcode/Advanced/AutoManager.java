package org.firstinspires.ftc.teamcode.Advanced;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class AutoManager {
    List<Vector2d> WayPoints = new ArrayList<>();
    List<MotionProfiles> Profiles = new ArrayList<>();
    List <Double> Angles = new ArrayList<>();

    private final LinearOpMode opMode;
    private final double meter_offset_scale = 1;

    public AutoManager(LinearOpMode opMode){
        this.opMode = opMode;
        WayPoints.add(new Vector2d());
    }

    public AutoManager AddPoint(Vector2d p){
        WayPoints.add(p);
        return this;
    }

    public AutoManager Build(){

        for(int i = 0; i < WayPoints.size() - 1; i++) {
            Vector2d point1 = WayPoints.get(i);
            Vector2d point2 = WayPoints.get(i + 1);

            double distance= Math.hypot(- point1.getX() + point2.getX(),
                                        - point1.getY() + point2.getY()) * meter_offset_scale;

            Profiles.add(new MotionProfiles(distance));

            double angle = Math.atan2(- point1.getY() + point2.getY(),
                                      - point1.getX() + point2.getX());

            Angles.add(angle);

        }
        return this;
    }
    public void Follow(Robot robot){
        ElapsedTime Timer = new ElapsedTime();

        for(int i = 0; i < Profiles.size() && opMode.opModeIsActive(); i++) {
            MotionProfiles Profile = Profiles.get(i);
            double Angle = Angles.get(i);

            Timer.reset();

            double t = 0;
            while (t <= Profile.t_total && opMode.opModeIsActive()) {
                t = Timer.time(TimeUnit.SECONDS);
                double velocity = Profile.GetSpeed(t) / Profile.MAX_VEL;

                double x = Math.cos(Angle) * velocity;
                double y = Math.sin(Angle) * velocity;

                robot.driveController.updateUsingJoysticks(new Vector2d(x, y), new Vector2d(), true);

                robot.telemetry.addData("Angle: ", Math.toDegrees(Angle));
                robot.telemetry.addData("x vel: ", x);
                robot.telemetry.addData("y vel: ", y);
                robot.telemetry.addData("total time: ", Profile.t_total);
                robot.telemetry.addData("current time: ", t);
                robot.telemetry.addData("distance: ", Profile.GetDistance(t));

                robot.telemetry.update();
            }
        }

        robot.driveController.updateUsingJoysticks(new Vector2d(), new Vector2d(), true);

    }
}
