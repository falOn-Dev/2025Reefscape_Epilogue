package frc.robot.subsystems.elevator

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
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
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import lib.math.units.into

@Logged
class ElevatorSim: Elevator() {
    private val elevatorSim: ElevatorSim = ElevatorSim(
        DCMotor.getKrakenX60Foc(2),
        5.0,
        Units.lbsToKilograms(15.0),
        drumRadius.into(Meters),
        0.0,
        maxHeight.into(Meters),
        true,
        0.0,
    )
    
    private var isClosedLoop = false
    
    private val pidController = PIDController(100.0, 0.0, 0.0)
    
    override val height: MutDistance = Meters.zero().mutableCopy()
        get() = field.mut_replace(elevatorSim.positionMeters, Meters)
    override val velocity: MutLinearVelocity = MetersPerSecond.zero().mutableCopy()
        get() = field.mut_replace(elevatorSim.velocityMetersPerSecond, MetersPerSecond)
    override val leftMotorCurrent: MutCurrent = Amps.zero().mutableCopy()
        get() = field.mut_replace(elevatorSim.currentDrawAmps, Amps)
    override val rightMotorCurrent: MutCurrent = Amps.zero().mutableCopy()
        get() = field.mut_replace(elevatorSim.currentDrawAmps, Amps)
    
    override fun setTargetHeight(height: Distance) {
        this.targetHeight.mut_replace(height)
        pidController.setpoint = height into Meters
        isClosedLoop = true
    }
    
    override fun setRawVoltage(voltage: Voltage) {
        setSimInputVoltage((voltage into Volts))
        isClosedLoop = false
    }
    
    private fun setSimInputVoltage(voltage: Double) {
        elevatorSim.setInputVoltage(voltage)
        leftMotorVoltage.mut_replace(voltage, Volts)
        rightMotorVoltage.mut_replace(voltage, Volts)
    }
    
    override fun simulationPeriodic() {
        elevatorSim.update(0.02)
        if(isClosedLoop) {
            println("closed loop")
            val voltage = pidController.calculate(height into Meters)
            setSimInputVoltage(voltage)
        }
    }
}