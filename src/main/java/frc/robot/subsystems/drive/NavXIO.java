package frc.robot.subsystems.drive;

// 1. CHANGE THIS: The package name is now com.studica!
import com.studica.frc.AHRS; 
import edu.wpi.first.math.geometry.Rotation2d;

public class NavXIO implements GyroIO {
  // 2. CHANGE THIS: The 2026 constructor requires the new NavXComType enum
  private final AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI);

  public NavXIO() {
    navX.reset();
  }

  public Rotation2d getYawRotation() {
    // This part stays exactly the same!
    return Rotation2d.fromDegrees(-navX.getYaw());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = getYawRotation();
    inputs.yawVelocityRadPerSec = Math.toRadians(-navX.getRate());
  }
}