package lib.motors

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.Units.RotationsPerSecondPerSecond
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import lib.math.controllers.gains.FeedforwardGains
import lib.math.controllers.gains.PIDGains

fun configuredTalonFX(
    id: Int,
    canbus: String = "rio",
    inverted: InvertedValue = InvertedValue.CounterClockwise_Positive,
    motionMagicVelocity: AngularVelocity = RotationsPerSecond.zero(),
    motionMagicAcceleration: AngularAcceleration = RotationsPerSecondPerSecond.zero(),
    pidGains: PIDGains,
    kG: Double,
    ffGains: FeedforwardGains,
    gravityTypeValue: GravityTypeValue = GravityTypeValue.Elevator_Static
): TalonFX {
    val motor = TalonFX(id, canbus)
    val config = TalonFXConfiguration()
    
    config.MotorOutput.Inverted = inverted
    config.MotionMagic.withMotionMagicCruiseVelocity(motionMagicVelocity)
    config.MotionMagic.withMotionMagicAcceleration(motionMagicAcceleration)
    
    config.Slot0.kP = pidGains.kP
    config.Slot0.kI = pidGains.kI
    config.Slot0.kD = pidGains.kD
    config.Slot0.kV = ffGains.kV
    config.Slot0.kA = ffGains.kA
    config.Slot0.kS = ffGains.kS
    config.Slot0.kG = kG
    config.Slot0.GravityType = gravityTypeValue
    
    config.CurrentLimits.StatorCurrentLimit = 80.0
    config.CurrentLimits.StatorCurrentLimitEnable = true
    config.CurrentLimits.SupplyCurrentLimit = 40.0
    config.CurrentLimits.SupplyCurrentLimitEnable = true
    
    motor.configurator.apply(config)
    return motor
}