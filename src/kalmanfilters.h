#pragma once
#include "kalmanfilter.h"
#include <iostream>

class LinearKalmanFilter : public KalmanFilterBase{
    public:
        LinearKalmanFilter(){}
        ~LinearKalmanFilter(){}

        VehicleState getVehicleState() override;
        Matrix2d getVehicleStatePositionCovariance() override;

        void predictionStep(double dt) override;
        void predictionStep(GyroMeasurement gyro, double dt) override;
        void handleLidarMeasurements(const std::vector<LidarMeasurement>& meas, const BeaconMap& map) override;
        void handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map) override;
        void handleGPSMeasurement(GPSMeasurement meas) override;
};

class ExtendedKalmanFilter : public KalmanFilterBase{
    public:
        ExtendedKalmanFilter(){}
        ~ExtendedKalmanFilter(){}

        VehicleState getVehicleState() override;
        Matrix2d getVehicleStatePositionCovariance() override;

        void predictionStep(double dt) override;
        void predictionStep(GyroMeasurement gyro, double dt) override;
        void handleLidarMeasurements(const std::vector<LidarMeasurement>& meas, const BeaconMap& map) override;
        void handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map) override;
        void handleGPSMeasurement(GPSMeasurement meas) override;
};

class UnscentedKalmanFilter : public KalmanFilterBase{
    public:
        UnscentedKalmanFilter(){}
        ~UnscentedKalmanFilter(){}

        VehicleState getVehicleState() override;
        Matrix2d getVehicleStatePositionCovariance() override;

        void predictionStep(double dt) override;
        void predictionStep(GyroMeasurement gyro, double dt) override;
        void handleLidarMeasurements(const std::vector<LidarMeasurement>& meas, const BeaconMap& map) override;
        void handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map) override;
        void handleGPSMeasurement(GPSMeasurement meas) override;
};