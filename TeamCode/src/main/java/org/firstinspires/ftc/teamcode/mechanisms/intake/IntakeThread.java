package org.firstinspires.ftc.teamcode.mechanisms.intake;

//Eric Debug Thanks
public class IntakeThread {
    public static boolean useSensors;
    private boolean[] slots;
    private final IntakeArtifactDetector ball1Distance;
    private final IntakeArtifactDetector ball2Distance;
    private final IntakeArtifactDetector ball3Distance;


    public IntakeThread(IntakeArtifactDetector ball1, IntakeArtifactDetector ball2, IntakeArtifactDetector ball3) {
        useSensors = true;
        slots = new boolean[] {false,false,false};
        ball1Distance = ball1;
        ball2Distance = ball2;
        ball3Distance = ball3;
    }

    public void update() {
        if (!useSensors) return;
        if (ball3Distance.hasArtifact()) {
            fillSlots(ball1Distance.getReading(), ball2Distance.getReading(), ball3Distance.getReading());
        }
    }

    public int getNumBalls() {
        int balls = 0;
        for (boolean slot : slots)
            if (slot)
                balls++;
        return balls;
    }


    public void reset() {
        stop();
        fillSlots(false,false,false);
        start();
    }

    public void start() {
        ball1Distance.start();
        ball2Distance.start();
        ball3Distance.start();
    }

    public void stop(){
        ball1Distance.stop();
        ball2Distance.stop();
        ball3Distance.stop();
    }

    public void fillSlots(boolean ball1, boolean ball2, boolean ball3){
        slots[0] = ball1;
        slots[1] = ball2;
        slots[2] = ball3;
    }

    public boolean[] getSlots(){
        return slots;
    }
}
