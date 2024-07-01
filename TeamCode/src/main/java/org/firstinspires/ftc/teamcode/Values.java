package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class Values extends LinearOpMode {
    public Ionut robot;
    public void runOpMode (){

        robot=new Ionut(this);

        Autonomy a1=new Autonomy()
                .AddPoint(new Pose(2,1,0))
                .Build();

        waitForStart();

        a1.Follow(robot);
    }

}
