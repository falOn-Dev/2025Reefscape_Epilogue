package frc.robot

import edu.wpi.first.epilogue.Epilogue
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.epilogue.logging.EpilogueBackend
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.elevator.ElevatorReal
import frc.robot.subsystems.elevator.ElevatorSim
import lib.math.units.inches

/**
 * The functions in this object (which basically functions as a singleton class) are called automatically
 * corresponding to each mode, as described in the TimedRobot documentation.
 * This is written as an object rather than a class since there should only ever be a single instance, and
 * it cannot take any constructor arguments. This makes it a natural fit to be an object in Kotlin.
 *
 * If you change the name of this object or its package after creating this project, you must also update
 * the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when renaming the
 * object or package, it will get changed everywhere.)
 */
@Logged
class Robot : TimedRobot() {
    
    val controller = CommandXboxController(0)
    
    @JvmField
    val elevator: Elevator = if(RobotBase.isReal()) ElevatorReal() else ElevatorSim()
    
    init {
        // Report the use of the Kotlin Language for "FRC Usage Report" statistics.
        // Please retain this line so that Kotlin's growing use by teams is seen by FRC/WPI.
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version)
        
        Epilogue.bind(this)
        
        controller.a().onTrue(elevator.runOnce { elevator.setTargetHeight(12.0.inches)})
        controller.b().onTrue(elevator.runOnce { elevator.setTargetHeight(24.0.inches)})
        controller.x().onTrue(elevator.runOnce { elevator.setTargetHeight(36.0.inches)})
        controller.y().onTrue(elevator.runOnce { elevator.setTargetHeight(48.0.inches)})
    }
    
    /**
     * This method is called every 20 ms, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run()
    }
    
    /** This method is called once each time the robot enters Disabled mode.  */
    override fun disabledInit() {
    
    }
    
    override fun disabledPeriodic() {
    
    }
    
    /** This autonomous runs the autonomous command selected by your [RobotContainer] class.  */
    override fun autonomousInit() {
    }
    
    /** This method is called periodically during autonomous.  */
    override fun autonomousPeriodic() {
    }
    
    override fun teleopInit() {
    
    }
    
    /** This method is called periodically during operator control.  */
    override fun teleopPeriodic() {
    
    }
    
    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }
    
    /** This method is called periodically during test mode.  */
    override fun testPeriodic() {
    
    }
    
    /** This method is called once when the robot is first started up.  */
    override fun simulationInit() {
    
    }
    
    /** This method is called periodically whilst in simulation.  */
    override fun simulationPeriodic() {
    
    }
}