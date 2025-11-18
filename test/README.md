# DS Node Test Suite

This directory contains the test suite for the ds_node ROS2 package.

## Test Structure

The test suite is organized into several categories:

### Unit Tests
- **test_ds_convert.cpp**: Tests for coordinate conversion functions
  - ECEF to geodetic coordinate conversion
  - Message conversion utilities (CompactNav to NavSatFix, Imu, PoseStamped)
  - Time conversion functions
  - Covariance assignment templates

- **test_ds_nmea.cpp**: Tests for NMEA message parsing
  - Checksum validation
  - GGA message parsing
  - Coordinate parsing and hemisphere handling
  - Fix quality and satellite count validation

### Integration Tests
- **test_integration.cpp**: Tests for ROS2 node integration
  - Node initialization and parameter handling
  - Message publishing and subscription
  - Coordinate frame transformations
  - Time synchronization

### Test Utilities
- **test_utils.hpp**: Helper functions and utilities
  - Random data generators for test messages
  - Distance calculations
  - Validation functions
  - NMEA sentence generators

### Package Validation
- **test_package.py**: Python script to validate package setup
  - Verify all required files exist
  - Check dependencies in package.xml
  - Validate CMakeLists.txt configuration

## Running Tests

### Using colcon
```bash
# Build and run all tests
colcon build --packages-select ds_node
colcon test --packages-select ds_node

# Run specific test executable
./build/ds_node/test_ds_convert
./build/ds_node/test_ds_nmea
./build/ds_node/test_integration
```

### Using the launch file
```bash
# Run all tests
ros2 launch ds_node test_launch.py

# Run specific test types
ros2 launch ds_node test_launch.py test_type:=unit
ros2 launch ds_node test_launch.py test_type:=integration
ros2 launch ds_node test_launch.py test_type:=convert
ros2 launch ds_node test_launch.py test_type:=nmea

# Run with verbose output
ros2 launch ds_node test_launch.py verbose:=true
```

### Using Python validation script
```bash
# Validate package setup
python3 test/test_package.py
```

## Test Coverage

The test suite covers:
- ✅ Coordinate conversion functions
- ✅ NMEA message parsing
- ✅ Message type conversions
- ✅ ROS2 node integration
- ✅ Parameter validation
- ✅ Error handling
- ✅ Edge cases and boundary conditions

## Adding New Tests

To add new tests:

1. **Unit Tests**: Add test cases to the appropriate existing test file or create a new one
2. **Integration Tests**: Add scenarios to `test_integration.cpp`
3. **Utilities**: Add helper functions to `test_utils.hpp`
4. **Update CMakeLists.txt**: Add the new test executable if creating a new test file

## Test Data

The test suite uses both:
- **Fixed test data**: Known good values for validation
- **Random test data**: Generated using the utilities in `test_utils.hpp`

## Dependencies

The test suite requires:
- Google Test framework (ament_cmake_gtest)
- ROS2 core packages (rclcpp, std_msgs, geometry_msgs, sensor_msgs, nav_msgs)
- Custom message types from ds_node package

## Continuous Integration

The tests are designed to run in CI/CD environments and produce:
- XML test results compatible with JUnit format
- Console output for debugging
- Exit codes for pass/fail status

## Troubleshooting

If tests fail:

1. Check that all dependencies are installed
2. Verify ROS2 environment is sourced
3. Check test output for specific error messages
4. Run tests with verbose output for detailed information
5. Check system requirements (sufficient memory, disk space)

## License

These tests are part of the ds_node package and are licensed under Apache License 2.0.