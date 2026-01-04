package org.firstinspires.ftc.teamcode.pedro;

public class FusionConstants {
    public double pinpointXCovariance;
    public double pinpointYCovariance;
    public double pinpointHeadingCovariance;
    public double externalSensorXCovariance;
    public double externalSensorYCovariance;
    public double externalSensorHeadingCovariance;
    public double processXCovariance;
    public double processYCovariance;
    public double processHeadingCovariance;

    public FusionConstants pinpointTranslationalCov(double cov) {
        pinpointXCovariance = cov;
        pinpointYCovariance = cov;
        return this;
    }

    public FusionConstants pinpointHeadingCov(double cov) {
        pinpointHeadingCovariance = cov;
        return this;
    }

    public FusionConstants externalSensorXCov(double cov) {
        externalSensorXCovariance = cov;
        return this;
    }

    public FusionConstants externalSensorYCov(double cov) {
        externalSensorYCovariance = cov;
        return this;
    }

    public FusionConstants externalSensorHeadingCov(double cov) {
        externalSensorHeadingCovariance = cov;
        return this;
    }

    public FusionConstants processXCov(double cov) {
        processXCovariance = cov;
        return this;
    }

    public FusionConstants processYCov(double cov) {
        processYCovariance = cov;
        return this;
    }

    public FusionConstants processHeadingCov(double cov) {
        processHeadingCovariance = cov;
        return this;
    }
}
