// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include <cmath>

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/StateSpaceUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/MathExtras.h>
#include <wpi/numbers>

#include "CANSparkMaxUtil.hpp"
#include "HWConfig.hpp"

using namespace frc3512;
using namespace frc3512::HWConfig::Climber;

Climber::Climber() {
    SetCANSparkMaxBusUsage(m_leftArmMotor, Usage::kPositionOnly);
    m_leftArmMotor.SetSmartCurrentLimit(40);
    SetCANSparkMaxBusUsage(m_rightArmMotor, Usage::kPositionOnly);
    m_rightArmMotor.SetSmartCurrentLimit(40);
}

void Climber::DeployClimbers() { m_solenoid.Set(false); }

void Climber::StowClimbers() { m_solenoid.Set(true); }

bool Climber::IsClimberDeployed() { return !m_solenoid.Get(); }

bool Climber::HasReachedUpperLimit() { return m_extendedSensor.Get(); }

bool Climber::HasReachedBottomLimit() { return m_retractedSensor.Get(); }

bool Climber::HasReachedFullUpperLimit() {
    return m_extendedSensor.Get() && m_retractedSensor.Get();
}

void Climber::RobotPeriodic() {
    frc::SmartDashboard::PutData("Climber", &m_climberSim);

    m_climberSolenoidEntry.SetBoolean(IsClimberDeployed());
    m_upperSensorEntry.SetBoolean(HasReachedUpperLimit());
    m_lowerSensorEntry.SetBoolean(HasReachedBottomLimit());
}

void Climber::TeleopPeriodic() {
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};

    double y = appendageStick2.GetRawAxis(1) * 0.75;

    /**
     * Climber Teleop Logic
     *
     * If the climber is deployed, it is allowed to fully extend as it won't go
     * passed any extention limits. If the climber is stowed then it has to stop
     * extending when it the top IR Beam Break sensor reads true. When the lower
     * sensor reads true, then the climber can no longer descend. When both the
     * top and bottom sensors are allowed to read true, then the climber is
     * fully extended and can't go much farther without damaging itself.
     *
     * Sensor Logic Table
     *
     * Lower sensor (Detects when the climber is all the way down)
     * Upper Sensor (Detects when the climber is all the way up)
     *
     * Lower Sensor | true  |  false  |    true     |
     *              =================================
     * Upper Sensor | false |   true  |    true     |
     *              =================================
     *              climber | climber | climber up  |
     *               down      up       && deployed
     */

    if (HasReachedUpperLimit() ||
        (HasReachedFullUpperLimit() && IsClimberDeployed())) {
        if (y < 0.0) {
            SetClimber(y);
        } else {
            SetClimber(0.0);
        }
    } else if (HasReachedBottomLimit()) {
        if (y > 0.0) {
            SetClimber(y);
        } else {
            SetClimber(0.0);
        }
    }

    if (appendageStick2.GetRawButtonPressed(11)) {
        DeployClimbers();
    }
    if (appendageStick2.GetRawButtonPressed(12)) {
        StowClimbers();
    }
}

void Climber::TestPeriodic() {}

void Climber::SimulationPeriodic() {
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};

    if (!HasReachedBottomLimit()) {
        m_climberPartSim->SetAngle(-180_deg);
    } else {
        m_climberPartSim->SetAngle(-90_deg);
    }

    double extension = appendageStick2.GetY();
    double extensionLength = m_extensionBase->GetLength();

    extension += extensionLength;

    if (extension < -40) {
        extension = -40;
    }

    if (extension > -20) {
        extension = -20;
    }

    m_extensionBase->SetLength(extension);
}

void Climber::SetClimber(double speed) { m_climber.Set(speed); }
