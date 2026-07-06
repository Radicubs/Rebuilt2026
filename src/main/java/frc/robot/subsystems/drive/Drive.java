package frc.robot.subsystems.drive;

import frc.robot.constants.DriveConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.module.Module;
import frc.robot.subsystems.gyro.Gyro;

import java.util.Optional;

public class Drive extends SubsystemBase {

    private static Drive instance;

    private final Module[] modules;
    private final Gyro gyro;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final DriveLogger logger;

    private RobotConfig config;

    private double simHeadingRad = 0.0;

    private SwerveModuleState[] desiredModuleStates = new SwerveModuleState[]{
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
    private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();

    public static Drive getInstance() {
        if (instance == null) {instance = new Drive();}
        return instance;
    }

    private Drive() {
        boolean sim = RobotBase.isSimulation();

        gyro = Gyro.getInstance();

        modules = new Module[]{
                Module.create(0, DriveConstants.Mod0.constants, sim),
                Module.create(1, DriveConstants.Mod1.constants, sim),
                Module.create(2, DriveConstants.Mod2.constants, sim),
                Module.create(3, DriveConstants.Mod3.constants, sim)
        };

        // prime inputs before seeding the pose estimator
        for (Module module : modules) {module.periodic();}
        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.swerveKinematics, getGyroYaw(), getModulePositions(), new Pose2d());

        logger = new DriveLogger(this);

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(3.0, 0.0, 0.0)
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this
        );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds speeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getHeading())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        SwerveModuleState[] swerveModuleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.maxSpeed);
        desiredChassisSpeeds = speeds;
        desiredModuleStates = swerveModuleStates;
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(swerveModuleStates[i], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeed);
        desiredModuleStates = desiredStates;
        desiredChassisSpeeds = DriveConstants.swerveKinematics.toChassisSpeeds(desiredStates);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(desiredStates[i], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond, false, false);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    SwerveModuleState[] getDesiredModuleStates() { return desiredModuleStates; }
    ChassisSpeeds getDesiredChassisSpeeds() { return desiredChassisSpeeds; }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return gyro.getYaw();
    }

    public void resetModulesToAbsolute() {
        for (Module module : modules) {
            module.resetToAbsolute();
        }
    }

    public void addVisionMeasurement(Pose2d estimatedPose, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(estimatedPose, timestampSeconds);
    }

    public Optional<Rotation2d> getHeadingToHub() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return Optional.empty();
        }
        Pose2d hub = alliance.get() == DriverStation.Alliance.Red
                ? FieldConstants.hubCenterRed
                : FieldConstants.hubCenterBlue;
        Pose2d pose = getPose();
        Translation2d toHub = new Translation2d(hub.getX() - pose.getX(), hub.getY() - pose.getY());
        return Optional.of(toHub.getAngle().plus(Rotation2d.k180deg));
    }

    @Override
    public void periodic() {
        for (Module module : modules) {
            module.periodic();
        }

        if (RobotBase.isSimulation()) {
            simHeadingRad += getRobotRelativeSpeeds().omegaRadiansPerSecond * 0.02;
            gyro.setSimYaw(Rotation2d.fromRadians(simHeadingRad));
        }

        poseEstimator.update(getGyroYaw(), getModulePositions());
        logger.log(this);
    }
}
