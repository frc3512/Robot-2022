// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "controllers/DrivetrainTurningController.hpp"

#include <algorithm>
#include <cmath>

#include <frc/MathUtil.h>
#include <frc/fmt/Eigen.h>
#include <frc/system/plant/LinearSystemId.h>

using namespace frc3512;

const frc::LinearSystem<2, 2, 2> DrivetrainTurningController::kPlant{
    GetPlant()};

DrivetrainTurningController::DrivetrainTurningController() {}

void DrivetrainTurningController::SetGoal(
    units::radian_t angle, units::radians_per_second_t angularVelocity) {
    units::radian_t ref{m_r[State::kHeading]};
    m_hasNewGoal = (angle != ref) ? true : false;
    m_goal = {frc::AngleModulus(angle), angularVelocity};
}

void DrivetrainTurningController::SetReferences(
    units::radian_t angle, units::radians_per_second_t angularVelocity) {
    auto vLeft = -angularVelocity * DrivetrainConstants::kWidth / 2;
    auto vRight = angularVelocity * DrivetrainConstants::kWidth / 2;
    m_nextR =
        Eigen::Vector<double, 3>{angle.value(), vLeft.value(), vRight.value()};
}

bool DrivetrainTurningController::HaveHeadingGoal() const {
    return m_hasNewGoal;
}

void DrivetrainTurningController::AbortTurnInPlace() { m_hasNewGoal = false; }

bool DrivetrainTurningController::AtHeading() const {
    frc::TrapezoidProfile<units::radian>::State ref{
        units::radian_t{m_nextR(0)}, units::radians_per_second_t{m_nextR(1)}};
    return m_goal == ref && m_atReferences;
}

Eigen::Vector<double, 2> DrivetrainTurningController::Calculate(
    const Eigen::Vector<double, 3>& x) {
    // angular velocity = (right velocity - left velocity) / width
    auto angularVelocity =
        (m_nextR[State::kRightVelocity] - m_nextR[State::kLeftVelocity]) /
        DrivetrainConstants::kWidth.value();

    // Calculate profiled references to the goal
    frc::TrapezoidProfile<units::radian>::State references{
        units::radian_t{m_nextR(State::kHeading)},
        units::radians_per_second_t{angularVelocity}};
    frc::TrapezoidProfile<units::radian> profile{m_constraints, m_goal,
                                                 references};
    auto profiledReference = profile.Calculate(Constants::kControllerPeriod);
    SetReferences(profiledReference.position, profiledReference.velocity);

    m_u = m_lqr.Calculate(x, m_r) + m_ff.Calculate(m_nextR);
    m_u = frc::DesaturateInputVector<2>(m_u, 12.0);

    Eigen::Vector<double, 3> error = m_nextR - x;
    error(0) = frc::AngleModulus(units::radian_t{error(0)}).value();
    UpdateAtReferences(error);

    m_r = m_nextR;

    if (AtHeading() && HaveHeadingGoal()) {
        m_hasNewGoal = false;
        m_goal = {frc::AngleModulus(units::radian_t{m_r(State::kHeading)}),
                  units::radians_per_second_t{0}};
    }

    return m_u;
}

frc::LinearSystem<2, 2, 2> DrivetrainTurningController::GetPlant() {
    return frc::LinearSystemId::IdentifyDrivetrainSystem(
        DrivetrainConstants::Meters::kLinearV,
        DrivetrainConstants::Meters::kLinearA,
        DrivetrainConstants::Meters::kAngularV,
        DrivetrainConstants::Meters::kAngularA);
}

frc::LinearSystem<3, 2, 3> DrivetrainTurningController::TurningDynamics() {
    Eigen::Matrix<double, 3, 3> A;
    A.setZero();
    A(0, 1) = -1.0 / DrivetrainConstants::kWidth.value();
    A(0, 2) = 1.0 / DrivetrainConstants::kWidth.value();
    A.block<2, 2>(1, 1) = kPlant.A();
    Eigen::Matrix<double, 3, 2> B;
    B.setZero();
    B.block<2, 2>(1, 0) = kPlant.B();
    Eigen::Matrix<double, 3, 3> C;
    C.setIdentity();
    Eigen::Matrix<double, 3, 2> D;
    D.setZero();

    return frc::LinearSystem<3, 2, 3>{A, B, C, D};
}

void DrivetrainTurningController::Reset(units::radian_t heading) {
    Eigen::Vector<double, 3> xhat{heading.value(), 0.0, 0.0};

    m_ff.Reset(xhat);
    m_r = xhat;
    m_nextR = xhat;
    m_hasNewGoal = false;

    UpdateAtReferences(Eigen::Vector<double, 3>::Zero());
}

void DrivetrainTurningController::UpdateAtReferences(
    const Eigen::Vector<double, 3>& error) {
    m_atReferences = std::abs(error(0)) < kAngleTolerance &&
                     std::abs(error(1)) < kVelocityTolerance &&
                     std::abs(error(2)) < kVelocityTolerance;
}
