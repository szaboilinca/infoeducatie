package org.firstinspires.ftc.teamcode.Advanced;

public class MotionProfiles {
    private final double robot_vel_scale = 0.3;
    private double acc = 0.5 * robot_vel_scale;
    public double MAX_VEL = 1.5 * robot_vel_scale;

    private double d;
    private double ta, tf, tc;
    private double da, df, dc;

    double t_total;

    public MotionProfiles(double distance) { // profil de miscare propriu zis
        this.d = distance;

        ta = MAX_VEL / acc;
        tf = ta;                   //profil simetric

        da = ta * MAX_VEL / 2;
        df = tf * MAX_VEL / 2;

        if(da + df <= d) {         //profil trapezoidal (exista dc si tc)
            tc = d / MAX_VEL - ta;
            dc = MAX_VEL * tc;
        } else {
            MAX_VEL = Math.sqrt(acc * d);
            tc = 0;
            dc = 0;

            ta = MAX_VEL / acc;      //se recalculeaza noua viteza maxima posibila
            tf = ta;

            da = ta * MAX_VEL / 2;
            df = tf * MAX_VEL / 2;
        }

        t_total = ta + tc + tf;     //timpul total al miscarii
    }
    public double GetDistance(double t){//localizare in fct de timp
        if(t <= ta) return acc * (t*t) /2;

        if(t <= ta + tc){
            double relative_time = t - ta;
            return da + MAX_VEL * relative_time;
        }

        if(t<=ta+tc+tf) {
            double relative_time = t - ta - tc;
            return da + dc + MAX_VEL * relative_time - acc * Math.pow(relative_time, 2) / 2;
        }

        return 0;
    }
    public double GetSpeed(double t){
        if(t <= ta) return acc * t;
        if(t <= ta + tc) return MAX_VEL;
        if(t <= ta + tc + tf) return MAX_VEL - acc * (t - ta - tc);

        return 0;
    }
}
