package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class Autonomy {
    List<Pose> WayPoints =new ArrayList<>();
    List<MotionProfiles> Profiles =new ArrayList<>();
    List <Double> Angles=new ArrayList<>();
    public Autonomy(){
        WayPoints.add(new Pose());
    }
    public Autonomy AddPoint(Pose p){
        WayPoints.add(p);
        return this;
    }
    public Autonomy Build(){
        for(int i=0;i<WayPoints.size()-1;i++){
            Pose point1=WayPoints.get(i);
            Pose point2=WayPoints.get(i+1);

            double distance= Math.hypot(point1.x+point2.x,point1.y+point2.y);
            Profiles.add(new MotionProfiles(distance));

            double angle= Math.atan2(point1.y+point2.y,point1.x+point2.x);
            Angles.add(new Double(angle));

        }
        return this;
    }
    public void Follow(Ionut robot){
        ElapsedTime Timer=new ElapsedTime();
        for(int i=0;i<Profiles.size();i++){
            MotionProfiles Profile=Profiles.get(i);
            double Angle=Angles.get(i).doubleValue();
            Timer.reset();
            double t=0;
            while (t <= Profile.ttotal) {
                t=Timer.time(TimeUnit.SECONDS);
                double velocity=Profile.GetSpeed(t);
                double x=Math.cos(Angle)*velocity;
                double y=Math.sin(Angle)*velocity;

                robot.Drive(x,y,0);
            }
        }

    }
}
