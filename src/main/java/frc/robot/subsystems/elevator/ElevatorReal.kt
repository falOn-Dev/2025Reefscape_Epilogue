package frc.robot.subsystems.elevator

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.epilogue.NotLogged
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.RadiansPerSecondPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MutCurrent
import edu.wpi.first.units.measure.MutDistance
import edu.wpi.first.units.measure.MutLinearVelocity
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import lib.math.controllers.gains.FeedforwardGains
import lib.math.controllers.gains.PIDGains
import lib.math.units.into
import lib.math.units.times
import lib.motors.configuredTalonFX

@Logged
class ElevatorReal : Elevator() {
    
    @NotLogged
    private val leftMotor: TalonFX = configuredTalonFX(
        id = leftID,
        inverted = InvertedValue.CounterClockwise_Positive,
        motionMagicVelocity = RadiansPerSecond.of(1.0 / drumRadius.into(Meters)), // 1.0 Meter per second
        motionMagicAcceleration = RadiansPerSecondPerSecond.of(1.0 / drumRadius.into(Meters)), // 1.0 Meter per second squared
        pidGains = PIDGains(),
        kG = 0.0,
        ffGains = FeedforwardGains(),
    )
    
    @NotLogged
    private val rightMotor: TalonFX = configuredTalonFX(
        id = rightID,
        inverted = InvertedValue.Clockwise_Positive,
        motionMagicVelocity = RadiansPerSecond.of(1.0 / drumRadius.into(Meters)), // 1.0 Meter per second
        motionMagicAcceleration = RadiansPerSecondPerSecond.of(1.0 / drumRadius.into(Meters)), // 1.0 Meter per second squared
        pidGains = PIDGains(),
        kG = 0.0,
        ffGains = FeedforwardGains(),
    )
    
    override val height: MutDistance = Meters.zero().mutableCopy()
        @NotLogged get() = field.mut_replace(
            leftMotor.position.valueAsDouble * 2 * Math.PI * drumRadius.into(Meters),
            Meters
        )
    override val velocity: MutLinearVelocity = MetersPerSecond.zero().mutableCopy()
        @NotLogged get() = field.mut_replace(
            leftMotor.velocity.valueAsDouble * 2 * Math.PI * drumRadius.into(Meters),
            MetersPerSecond
        )
    override val leftMotorVoltage: MutVoltage = Volts.zero().mutableCopy()
        @NotLogged get() = field.mut_replace(leftMotor.motorVoltage.value)
    override val leftMotorCurrent: MutCurrent = Amps.zero().mutableCopy()
        @NotLogged get() = field.mut_replace(leftMotor.statorCurrent.value)
    override val rightMotorVoltage: MutVoltage = Volts.zero().mutableCopy()
        @NotLogged get() = field.mut_replace(rightMotor.motorVoltage.value)
    override val rightMotorCurrent: MutCurrent = Amps.zero().mutableCopy()
        @NotLogged get() = field.mut_replace(rightMotor.statorCurrent.value)
    
    private val followerRequest: Follower = Follower(leftID, true)
    private val positionRequest: MotionMagicVoltage = MotionMagicVoltage(0.0).withEnableFOC(true)
    private val voltageRequest: VoltageOut = VoltageOut(0.0).withEnableFOC(true)
    
    override fun setTargetHeight(height: Distance) {
        val targetRotations = height.into(Meters) / drumRadius.into(Meters) / (2 * Math.PI)
        leftMotor.setControl(positionRequest.withPosition(targetRotations))
        rightMotor.setControl(followerRequest)
    }
    
    override fun setRawVoltage(voltage: Voltage) {
        leftMotor.setControl(voltageRequest.withOutput(voltage))
        rightMotor.setControl(followerRequest)
    }
}