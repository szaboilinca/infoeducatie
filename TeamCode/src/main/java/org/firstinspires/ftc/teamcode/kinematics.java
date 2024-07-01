package org.firstinspires.ftc.teamcode;
import java.util.ArrayList;
import java.util.List;

public class kinematics {
    static double l;//dist in m
    public static List<ModuleState> InverseKinematics (double x,double y,double w){
        double LeftX=x;//roata stanga
        double RightX=x;
        double LeftY=y+l*w/2;//roti diametral opuse=>schimbare de semn
        double RightY=y-l*w/2;
        ModuleState Left=new ModuleState(Math.hypot(LeftX,LeftY), Math.atan2(LeftY,LeftX));
        ModuleState Right=new ModuleState(Math.hypot(RightX,RightY), Math.atan2(RightY,RightX));
        List<ModuleState> output=new ArrayList<>();
        output.add (Left);
        output.add (Right);
        return output;
    }
}
