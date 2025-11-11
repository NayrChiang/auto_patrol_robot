# Media Folder

This folder contains images captured by the autonomous patrol robot during waypoint navigation.

## File Naming Convention

Images are automatically saved with the following naming format:
```
time_{relative_time}s_point_{point_index}.png
```

Where:
- `relative_time`: Time in seconds since patrol started (integer, no decimals)
- `point_index`: The waypoint number (0, 1, 2, ...)

## Example Files
- `time_12s_point_0.png` - Image captured at waypoint 0, 12 seconds after start
- `time_45s_point_1.png` - Image captured at waypoint 1, 45 seconds after start
- `time_127s_point_3.png` - Image captured at waypoint 3, 127 seconds after start

## Note
Image files (*.png) in this directory are ignored by Git to keep the repository size manageable. Only the folder structure is tracked via `.gitkeep`.

