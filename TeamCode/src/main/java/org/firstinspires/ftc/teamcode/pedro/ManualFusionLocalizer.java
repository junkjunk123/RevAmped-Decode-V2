package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.utils.math.calc.Vector2D;
import org.firstinspires.ftc.teamcode.utils.math.calc.Vector3D;

public class ManualFusionLocalizer implements Localizer {
    private final Localizer deadReckoning;
    private final Pose landmark;       // landmark position for bearing measurement
    private Pose state;            // x, y, theta
    private Pose velocity;         // world-frame twist
    private Matrix P;               // state covariance
    private final Matrix diagQ;      // body-frame process noise diagonal
    private final double R;            // measurement noise (bearing variance)
    private final Vector3D initialP;   // initial covariance diagonal
    private long lastUpdateTime = -1;

    public ManualFusionLocalizer(
            Localizer deadReckoning,
            Pose landmark,
            Pose startPose,
            Vector3D diagQ,
            double R,
            Vector3D initialP   // initial uncertainty in x, y, theta
    ) {
        this.deadReckoning = deadReckoning;
        this.landmark = landmark;
        this.state = startPose.copy();
        this.velocity = new Pose();
        this.diagQ = Matrix.diag(diagQ.getValues());
        this.R = R;
        this.initialP = initialP.copy();

        // Initialize covariance to the provided initial diagonal
        P = new Matrix(new double[][] {
                {initialP.getX(), 0, 0},
                {0, initialP.getY(), 0},
                {0, 0, initialP.getZ()},
            }
        );

        deadReckoning.setStartPose(new Pose(startPose.getX(), startPose.getY(), startPose.getHeading()));
    }

    @Override
    public void update() {
        deadReckoning.update();
        long now = System.nanoTime();
        double dt = lastUpdateTime < 0 ? 0 : (now - lastUpdateTime)/1e9;
        lastUpdateTime = now;

        // World-frame velocity from dead reckoning
        Pose twistPose = deadReckoning.getVelocity();
        velocity = new Pose(twistPose.getX(), twistPose.getY(), twistPose.getHeading());

        // Forward-Euler integration for mean
        state = integrate(state, velocity, dt);

        // Propagate covariance
        P = propagateCovariance(P, state.getHeading(), diagQ, dt);
    }

    private static Pose integrate(Pose state, Pose vel, double dt) {
        return state.plus(vel.scale(dt));
    }

    private static Matrix propagateCovariance(Matrix P, double theta, Matrix diagQ, double dt) {
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);

        // Jacobian G from body-frame twist noise to world-frame state
        Matrix G = new Matrix(new double[][] {
                {dt*cos, -dt*sin, 0},
                {dt *sin,  dt*cos, 0},
                {0, 0, dt}
        });

        // Q_state = G * diag(diagQ) * G^T

        Matrix Q = G.multiply(diagQ).multiply(G.transposed());

        return P.plus(Q);
    }

    /**
     * Bearing measurement update:
     * Sensor measures radians = atan2(y_l - y, x_l - x) - theta + noise
     */
    public void addTurretEstimate(double radians) {
        Vector2D delta = Vector2D.fromPose(landmark.minus(state));
        double predictedBearing = delta.getTheta() - state.getHeading();
        predictedBearing = MathFunctions.normalizeAngle(predictedBearing);

        // Innovation
        double y = MathFunctions.normalizeAngle(radians - predictedBearing);

        // Measurement Jacobian H
        double r2 = delta.magSquared();
        Vector3D H = new Vector3D(delta.getY() / r2, -delta.getX() / r2, -1);
        double S = H.quadraticForm(P) + R;  // Innovation covariance
        Vector3D K = H.transform(P).dividedBy(S);  // Kalman gain

        // State update
        state = state.plus(K.toPose().scale(y));

        // Joseph-form covariance update (scalar-expanded)
        Matrix temp = Matrix.identity(3).minus(K.tensorProduct(H));
        P = temp.multiply(P).multiply(temp.transposed()).plus(K.tensorProduct(K).times(R));
    }

    @Override
    public Pose getPose() {
        return new Pose(state.getX(), state.getY(), state.getHeading());
    }

    @Override
    public Vector getVelocityVector() { return velocity.getAsVector(); }

    @Override
    public Pose getVelocity() {
        return velocity.copy();
    }

    @Override
    public void setStartPose(Pose startPose) {
        state = new Pose(startPose.getX(), startPose.getY(), startPose.getHeading());
        deadReckoning.setStartPose(startPose);

        // Reset covariance to initial values
        P = new Matrix(new double[][] {
                {initialP.getX(), 0, 0},
                {0, initialP.getY(), 0},
                {0, 0, initialP.getZ()},
        });
    }

    @Override
    public void setPose(Pose setPose) {
        state = new Pose(setPose.getX(), setPose.getY(), setPose.getHeading());
        deadReckoning.setPose(setPose);
    }

    @Override
    public double getTotalHeading() { return deadReckoning.getTotalHeading(); }

    @Override
    public double getForwardMultiplier() { return deadReckoning.getForwardMultiplier(); }

    @Override
    public double getLateralMultiplier() { return deadReckoning.getLateralMultiplier(); }

    @Override
    public double getTurningMultiplier() { return deadReckoning.getTurningMultiplier(); }

    @Override
    public void resetIMU() throws InterruptedException { deadReckoning.resetIMU(); }

    @Override
    public double getIMUHeading() { return deadReckoning.getIMUHeading(); }

    @Override
    public boolean isNAN() {
        return Double.isNaN(state.getX()) || Double.isNaN(state.getY()) || Double.isNaN(state.getHeading());
    }
}
