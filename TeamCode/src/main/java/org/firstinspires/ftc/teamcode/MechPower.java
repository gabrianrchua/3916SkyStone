package org.firstinspires.ftc.teamcode;

public class MechPower {
    public double leftFront;
    public double leftBack;
    public double rightFront;
    public double rightBack;
    public MechPower(double lf, double lb, double rf, double rb) {
        leftBack = lb;
        leftFront = lf;
        rightBack = rb;
        rightFront = rf;
    }
    public MechPower interpolate(MechPower mp, double interpolationValue) {
        double lf = lerp(leftFront, mp.leftFront, interpolationValue);
        double lb = lerp(leftBack, mp.leftBack, interpolationValue);
        double rf = lerp(rightFront, mp.rightFront, interpolationValue);
        double rb = lerp(rightBack, mp.rightBack, interpolationValue);
        return new MechPower(lf, lb, rf, rb);
    }
    public double lerp(double a, double b, double f)
    {
        if (b > a)
            return a + f * (b - a);
        else
            return b + f * (a - b);
    }
    @Override
    public String toString() {
        return "LF:" + leftFront + " LB:" + leftBack + " RF:" + rightFront + " RB:" + rightBack;
    }
}
