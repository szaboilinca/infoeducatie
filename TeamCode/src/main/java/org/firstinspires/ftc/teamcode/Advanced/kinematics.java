package org.firstinspires.ftc.teamcode.Advanced;

import java.util.ArrayList;
import java.util.List;

public class kinematics {
    private static double TRACK_WIDTH = 0.5;//dist in m
    private static double R = TRACK_WIDTH / 2;

    public static List<ModuleState> InverseKinematics (double x, double y, double w) { // cm, cm, rad
        double LeftX = x; //roata stanga
        double RightX = x;
        double LeftY = y + R * w; //roti diametral opuse=>schimbare de semn
        double RightY = y - R * w;

        ModuleState Left = new ModuleState(Math.hypot(LeftX,LeftY), Math.atan2(LeftY,LeftX));
        ModuleState Right = new ModuleState(Math.hypot(RightX,RightY), Math.atan2(RightY,RightX));

        List<ModuleState> output = new ArrayList<>();

        output.add(Left);
        output.add(Right);

        return output;
    }

    public static class ModuleState {
        public double speed;
        public double angle;

        public ModuleState(double speed, double angle) {
            this.speed = speed;
            this.angle = angle;
        }

        public ModuleState() {
            this.speed = 0;
            this.angle = 0;
        }
    }
}


