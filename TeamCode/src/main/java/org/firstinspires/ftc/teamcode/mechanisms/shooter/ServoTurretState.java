package org.firstinspires.ftc.teamcode.mechanisms.shooter;

public sealed interface ServoTurretState permits ServoTurretState.AutoTrack, ServoTurretState.Custom, ServoTurretState.PresetState {
    Turret.MoveState.AutoTrack AUTO_TRACK = new Turret.MoveState.AutoTrack();

    enum PresetState implements ServoTurretState {
        LEFT_135,
        LEFT_90,
        LEFT_45,
        REST,
        RIGHT_45,
        RIGHT_90,
        RIGHT_135;

        public double targetPos() {
            return (ordinal() - REST.ordinal()) * ServoTurret.ticksPerRotation() / 8 + ServoTurretMTI.REST;
        }

        public PresetState next() {
            if (this == RIGHT_135) return RIGHT_135;
            return values()[ordinal() + 1];
        }

        public PresetState previous() {
            if (this == LEFT_135) return LEFT_135;
            return values()[ordinal() - 1];
        }
    }

    double targetPos();

    record AutoTrack(double targetPos) implements ServoTurretState { }

    record Custom(double targetPos) implements ServoTurretState { }
}
