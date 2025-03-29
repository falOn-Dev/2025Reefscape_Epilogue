package frc.robot.subsystems.elevator

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MutCurrent
import edu.wpi.first.units.measure.MutDistance
import edu.wpi.first.units.measure.MutLinearVelocity
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import lib.math.units.epsilonEquals
import lib.math.units.inches

@Logged
abstract class Elevator : SubsystemBase("elevator") {
    open val height: MutDistance = Meters.zero().mutableCopy()
    open val targetHeight: MutDistance = Meters.zero().mutableCopy()
    open val velocity: MutLinearVelocity = MetersPerSecond.zero().mutableCopy()
    open val leftMotorVoltage: MutVoltage = Volts.zero().mutableCopy()
    open val rightMotorVoltage: MutVoltage = Volts.zero().mutableCopy()
    open val leftMotorCurrent: MutCurrent = Amps.zero().mutableCopy()
    open val rightMotorCurrent: MutCurrent = Amps.zero().mutableCopy()
    
    private var positionToleranceMap: MutableMap<Distance, Trigger> = mutableMapOf()
    
    abstract fun setTargetHeight(height: Distance)
    abstract fun setRawVoltage(voltage: Voltage)
    
    fun getPositionInToleranceTrigger(tolerance: Distance): Trigger {
        return positionToleranceMap.getOrPut(tolerance) { Trigger { height.epsilonEquals(targetHeight, tolerance) } }
    }
    
    companion object Constants {
        const val leftID: Int = 0
        const val rightID: Int = 1
        val drumRadius: Distance = (1.76/2).inches
        val maxHeight: Distance = 52.0.inches
    }
}