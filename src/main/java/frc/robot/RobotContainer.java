// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.PneumaticsCTR;
import frc.robot.subsystems.PneumaticsREV;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.Date;






/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final PneumaticsREV m_pneumatics = new PneumaticsREV();
  private final Camera m_camera = new Camera();
  private final NavX m_gyro = new NavX();
  private SendableChooser<Integer> m_chooser = new SendableChooser<Integer>(); 


//   private final Pixy2 m_pixy = new Pixy2();
  Thread m_visionThread;

  // The driver's controller
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  //Axis[0] - left(-)/right(+)
  //Axis[1] - forward(-)/back(+)
  //Axis[2] - rotation (right is +)
  //Axis[3] altitude? up(-)/down(+)
  GenericHID m_driverController = new GenericHID(OIConstants.kDriverControllerPort);
  GenericHID m_driverController2 = new GenericHID(OIConstants.kDriverControllerPort2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // m_pixy.init();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getRawAxis(1), 0.2),
                MathUtil.applyDeadband(-m_driverController.getRawAxis(0), 0.2),
                MathUtil.applyDeadband(-m_driverController.getRawAxis(2), 0.2),
                false, (m_driverController.getRawAxis(3)+1)/2),
            m_robotDrive)
            );
        
        m_elevator.setDefaultCommand(
            new RunCommand(
                () -> m_elevator.drive(MathUtil.applyDeadband(-m_driverController2.getRawAxis(1), 0.2)), m_elevator)
                );
                
        m_gyro.setDefaultCommand(
            new RunCommand(
            () -> m_gyro.putGyro()    
            , m_gyro)
        );
        m_camera.startCamera();

        //armStage
    m_pneumatics.setDefaultCommand(
        new RunCommand(
            () -> m_pneumatics.setArmStage(m_driverController2.getRawAxis(3)), m_pneumatics
        ));
        m_visionThread =
        new Thread(
            () -> {
                m_camera.visionSystem();
            });
            m_visionThread.setDaemon(true);
            m_visionThread.start();
        }
        
    



  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    //IDK what this is
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));


    new JoystickButton(m_driverController2, 1).whileTrue(
        new InstantCommand(
            () -> m_pneumatics.toggleClaw(), 
            m_pneumatics));

    m_chooser.setDefaultOption("Drop and leave", 1);
    m_chooser.addOption("Drop and balance", 2);
    SmartDashboard.putData(m_chooser);
    
    // new JoystickButton(m_driverController, 2).whileTrue(
    //     new InstantCommand(
    //         () -> m_pixy.setLamp(1, 1), 
    //         m_pixy));

    // new JoystickButton(m_driverController, 3).whileTrue(
    //     new InstantCommand(
    //         () -> m_pixy.setLamp(0, 0), 
    //         m_pixy));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // RunCommand auto = new RunCommand(new Autonomous(m_robotDrive, m_gyro));
    // return auto;
    // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     m_robotDrive::setModuleStates,
    //     m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, (m_driverController.getRawAxis(3)+1)/2 ));
   //}

  public void teloPeriodic(){
  }

  int state = 0;
  double startTime = System.currentTimeMillis();
  private RunCommand m_forward = new RunCommand(() -> m_robotDrive.drive(.6, 0, 0, false, 0.3), m_robotDrive);
  private RunCommand m_slow1 = new RunCommand(() -> m_robotDrive.drive(.15, 0, 0, false, 0.1), m_robotDrive);
  private RunCommand m_slow2 = new RunCommand(() -> m_robotDrive.drive(-.15, 0, 0, false, 0.1), m_robotDrive);
  private RunCommand m_backward = new RunCommand(() -> m_robotDrive.drive(-.6, 0, 0, false, 0.3), m_robotDrive);
  private RunCommand m_elevatorDown = new RunCommand(() -> m_elevator.drive(.6), m_elevator);
 
  public void autonomousPeriodic(){
    AHRS gyro = m_gyro.gyro();
    
    //commands
    //RunCommand forward = new RunCommand(() -> m_robotDrive.drive(.6, 0, 0, false, 0.3), m_robotDrive);
    //RunCommand slow1 = new RunCommand(() -> m_robotDrive.drive(.15, 0, 0, false, 0.1), m_robotDrive);
    //RunCommand slow2 = new RunCommand(() -> m_robotDrive.drive(-.15, 0, 0, false, 0.1), m_robotDrive);
    //RunCommand backward = new RunCommand(() -> m_robotDrive.drive(-.6, 0, 0, false, 0.3), m_robotDrive);
    //RunCommand elevatorDown = new RunCommand(() -> m_elevator.drive(.6), m_elevator);
    
    switch(state){
        case(0):
            startTime = System.currentTimeMillis();
            state = 1;
        case(1):
            m_elevatorDown.schedule();
            System.out.println(System.currentTimeMillis() - startTime);
            if(System.currentTimeMillis() - startTime >= 3000){
                CommandScheduler.getInstance().cancel(m_elevatorDown);
                startTime = System.currentTimeMillis();
                state = 2;
            }
            break;
        case(2):
            m_forward.schedule();
            if(System.currentTimeMillis() - startTime >= 3000){
                CommandScheduler.getInstance().cancel(m_forward);
                m_pneumatics.toggleClaw();
                gyro.resetDisplacement();
                startTime = System.currentTimeMillis();
                state = 3;
            }
            break;
        case(3):
            m_backward.schedule();
            if(m_chooser.getSelected() == 2) {
                // Do balance
                state = 4;
            }
            else {
                // Just back up.
                if(System.currentTimeMillis() - startTime >= 3000){
                    CommandScheduler.getInstance().cancel(m_backward);
                }
            }      
            break;
        case(4):
            if(Math.abs(m_gyro.pitch()) > 10){
                state = 5;
                CommandScheduler.getInstance().cancel(m_backward);
            }
            break;
        case(5):
            if(m_gyro.pitch() > 3){
                CommandScheduler.getInstance().cancel(m_slow2);
                m_slow1.schedule();
            }
            else if(m_gyro.pitch() < -3){
                CommandScheduler.getInstance().cancel(m_slow1);
                m_slow2.schedule();
            } else {
                CommandScheduler.getInstance().cancel(m_backward);
                CommandScheduler.getInstance().cancel(m_slow2);
                CommandScheduler.getInstance().cancel(m_slow1);
            }
            break;
    }
  }
}


