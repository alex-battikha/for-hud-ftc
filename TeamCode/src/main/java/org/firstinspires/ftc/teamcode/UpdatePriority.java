package org.firstinspires.ftc.teamcode;

public class UpdatePriority {
    public double basePriority;
    public double priorirtyScale;
    public double lastPower = 0;
    public double power = 0;
    long lastUpdateTime;

    public UpdatePriority(double basePriority, double priorityScale) {
        this.basePriority = basePriority;
        this.priorirtyScale = priorityScale;
        lastUpdateTime = System.currentTimeMillis();
    }

    public void setTargetPower(double targetPower) {
            power = targetPower;
    }

    public double getPriority() { //get or set means priority
        if(power - lastPower == 0) {
            lastUpdateTime = System.currentTimeMillis();
            return 0;
        }
        return basePriority + Math.abs(power - lastPower) * (System.currentTimeMillis() - lastUpdateTime) * priorirtyScale;
    }

    public void update() {
        lastUpdateTime = System.currentTimeMillis();
        lastPower = power;
    }
}
