package org.firstinspires.ftc.teamcode.utils.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Supplier;

public interface Encoder {
    int getPosition();
    double getVelocity();

    static Encoder fromLambdas(Supplier<Integer> position, Supplier<Double> velocity) {
        return new Encoder() {
            @Override
            public int getPosition() {
                return position.get();
            }

            @Override
            public double getVelocity() {
                return velocity.get();
            }
        };
    }

    static Encoder external(DcMotorEx otherMotor) {
        return new Encoder() {
            @Override
            public int getPosition() {
                return otherMotor.getCurrentPosition();
            }

            @Override
            public double getVelocity() {
                return otherMotor.getVelocity();
            }
        };
    }

    default Encoder reverse() {
        return new Encoder() {
            @Override
            public int getPosition() {
                return -Encoder.this.getPosition();
            }

            @Override
            public double getVelocity() {
                return -Encoder.this.getVelocity();
            }
        };
    }
}