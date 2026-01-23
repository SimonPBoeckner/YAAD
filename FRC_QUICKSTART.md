# FRC Quick Start Guide

Get your AprilTag vision system running on your robot in 15 minutes.

## Step 1: Hardware Setup (5 minutes)

### Required Hardware
- Raspberry Pi 4 (4GB+ recommended) or similar coprocessor
- USB camera (Logitech C920 or similar)
- MicroSD card (16GB+)
- Ethernet cable to robot

### Connect Hardware
1. Connect camera to Raspberry Pi USB port
2. Connect Pi to robot via Ethernet (not WiFi for reliability)
3. Power Pi from robot PDP/PDH

## Step 2: Software Installation (5 minutes)

### On Raspberry Pi

```bash
# Install dependencies
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake \
    libopencv-dev \
    libeigen3-dev \
    nlohmann-json3-dev \
    apriltag-dev

# Install WPILib ntcore
wget https://github.com/wpilibsuite/allwpilib/releases/download/v2024.3.2/ntcore-cpp-*-linuxarm64.zip
unzip ntcore-cpp-*.zip -d /usr/local
sudo ldconfig

# Copy your vision system
scp -r vision_system/ pi@raspberrypi.local:~/

# Compile
cd ~/vision_system
g++ -std=c++17 *.cpp -o vision_system \
    -lopencv_core -lopencv_imgproc -lopencv_calib3d -lopencv_imgcodecs \
    -lapriltag -lntcore -lwpiutil -lpthread
```

## Step 3: Configuration (3 minutes)

Create `config_advanced.json`:

```json
{
  "camera": {
    "index": 0,
    "width": 640,
    "height": 480,
    "fps": 60
  },
  "paths": {
    "field_layout": "/home/pi/vision_system/2025-field.json",
    "camera_calibration": "",
    "log_file": "vision.log"
  },
  "detection": {
    "quad_decimate": 1.0,
    "num_threads": 4,
    "refine_edges": true
  },
  "tag": {
    "size": 0.1651
  },
  "networktables": {
    "enabled": true,
    "team_number": "254",
    "is_server": false
  }
}
```

**Important**: Replace `"254"` with YOUR team number!

## Step 4: Robot Code (2 minutes)

### Copy VisionSubsystem

Copy `VisionSubsystem.java` to `src/main/java/frc/robot/subsystems/`

### Add to RobotContainer

```java
public class RobotContainer {
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    
    public RobotContainer() {
        // Vision updates pose estimator automatically
        driveSubsystem.setPoseEstimatorCallback(
            () -> visionSubsystem.updatePoseEstimator(
                driveSubsystem.getPoseEstimator()
            )
        );
    }
}
```

### Or in Drivetrain Periodic

```java
@Override
public void periodic() {
    // ... existing code ...
    
    // Update pose with vision
    visionSubsystem.updatePoseEstimator(poseEstimator);
}
```

## Step 5: Test! (1 minute)

### Start Vision System

```bash
./vision_system config_advanced.json
```

You should see:
```
[INFO] Starting AprilTag Multi-Camera Vision System
[INFO] NetworkTables client connecting to team 254
[INFO] All cameras started
[INFO] System running. Press Ctrl+C to exit.
```

### Check from Driver Station

1. Open OutlineViewer or Glass
2. Connect to robot
3. Look for `Vision` table
4. Verify `Vision/active = true`
5. Point camera at AprilTag
6. Watch `Vision/Fused/pose` update

## Verify It's Working

### ✅ Checklist

- [ ] Vision system starts without errors
- [ ] `Vision/active` is `true` in NetworkTables
- [ ] Camera LED is on
- [ ] When pointing at AprilTag, `Vision/Fused/confidence` > 0.5
- [ ] `Vision/Fused/pose` shows reasonable X, Y values
- [ ] Robot pose on Field2d widget updates from vision

### 🎯 Quick Test

```java
// Add this to your teleopPeriodic for testing
public void teleopPeriodic() {
    if (visionSubsystem.canSeeAnyTags()) {
        System.out.println("Vision sees tags! Confidence: " + 
                          visionSubsystem.getConfidence());
    }
}
```

## Troubleshooting

### "NetworkTables client connecting..." but never connects

**Problem**: Can't connect to robot
**Fix**: 
1. Check team number in config: `"team_number": "YOUR_TEAM"`
2. Verify Pi is on robot network: `ping 10.TE.AM.2`
3. Check robot code is running NetworkTables server

### No camera found

**Problem**: `/dev/video0` doesn't exist
**Fix**:
```bash
ls /dev/video*  # Find your camera
# Update config: "index": 2  (if camera is /dev/video2)
```

### Can see tags but pose is wrong

**Problem**: Incorrect field layout or tag size
**Fix**:
1. Verify tag size: Measure actual tag (usually 6.5" = 0.1651m)
2. Update config: `"tag": {"size": 0.1651}`
3. Check field layout JSON matches game

### Vision pose jumps around

**Problem**: Poor calibration or lighting
**Fix**:
1. Run camera calibration (see main README)
2. Add more cameras
3. Increase `min_cameras_required: 2`
4. Improve lighting

## Auto-Start on Boot

Create `/etc/systemd/system/vision.service`:

```ini
[Unit]
Description=AprilTag Vision System
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/vision_system
ExecStart=/home/pi/vision_system/vision_system /home/pi/vision_system/config_advanced.json
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable it:
```bash
sudo systemctl enable vision.service
sudo systemctl start vision.service
sudo systemctl status vision.service
```

## Competition Checklist

Before Competition:
- [ ] Camera calibration completed
- [ ] Field layout JSON matches game
- [ ] Tag size verified (measure!)
- [ ] System auto-starts on boot
- [ ] Tested with 2+ tags visible
- [ ] Tested in competition lighting
- [ ] NetworkTables connection solid
- [ ] Robot code handles vision dropouts gracefully

During Competition:
- [ ] Check camera isn't blocked
- [ ] Verify `Vision/active` in driver station
- [ ] Monitor confidence levels
- [ ] Have backup auto without vision

## Common Use Cases

### Use Case 1: Auto Alignment

```java
public Command alignToTag(int tagId) {
    return new ParallelRaceGroup(
        new WaitUntilCommand(() -> 
            Math.abs(getAngleToTag(tagId)) < 2.0
        ),
        new RunCommand(() -> {
            double angle = getAngleToTag(tagId);
            drive.drive(0, 0, angle * 0.1);  // P controller
        })
    ).withTimeout(3.0);
}
```

### Use Case 2: Scoring Position Check

```java
public boolean isInScoringPosition() {
    return visionSubsystem.canSeeTag(SCORING_TAG_ID) &&
           visionSubsystem.getConfidence() > 0.7 &&
           getDistanceToTag(SCORING_TAG_ID) < 1.5;
}
```

### Use Case 3: Vision-Based Auto

```java
public Command autoScore() {
    return sequence(
        new WaitUntilCommand(() -> visionSubsystem.isVisionActive()),
        new DriveToTagCommand(visionSubsystem, SCORING_TAG),
        new ScoreCommand()
    );
}
```

## Performance Tips

### For Fastest Updates (Lowest Latency)
```json
{
  "camera": {"fps": 90, "width": 640, "height": 480},
  "detection": {"quad_decimate": 2.0, "num_threads": 4}
}
```
Trade-off: Lower accuracy

### For Highest Accuracy
```json
{
  "camera": {"fps": 60, "width": 1280, "height": 720},
  "detection": {"quad_decimate": 1.0, "refine_edges": true}
}
```
Trade-off: Higher latency

### Balanced (Recommended)
```json
{
  "camera": {"fps": 60, "width": 640, "height": 480},
  "detection": {"quad_decimate": 1.0, "num_threads": 4}
}
```

## Support Resources

- **Full Documentation**: See README.md
- **FRC Integration Guide**: See FRC_INTEGRATION.md
- **Robot Code Example**: See VisionSubsystem.java
- **WPILib Docs**: https://docs.wpilib.org
- **Chief Delphi**: Post in Vision category

## Quick Reference

### NetworkTables Values

| Path | Type | Description |
|------|------|-------------|
| `Vision/active` | boolean | System running |
| `Vision/Fused/pose` | double[6] | [x,y,z,roll,pitch,yaw] |
| `Vision/Fused/confidence` | double | 0.0 to 1.0 |
| `Vision/Cameras/*/tag_ids` | int[] | Detected tags |

### Default Ports

| Service | Port |
|---------|------|
| NetworkTables | 5810 |
| Web Dashboard | 8080 |
| MJPEG Stream | 8081+ |
| UDP Data | 5800 |

### Typical Latency

| Component | Time |
|-----------|------|
| Camera capture | 16ms |
| Detection | 15ms |
| NetworkTables | 5ms |
| **Total** | **~35ms** |

## Next Steps

1. ✅ Get basic vision working
2. 📸 Camera calibration for accuracy
3. 🎯 Add second camera for redundancy
4. 🎮 Integrate with auto commands
5. 🧪 Test in competition conditions
6. 🏆 Win competitions!

Good luck with your season! 🤖