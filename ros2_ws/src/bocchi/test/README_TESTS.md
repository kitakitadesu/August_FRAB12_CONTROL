# ROS2 WebSocket Test Suite Documentation

## Overview

This directory contains comprehensive tests for the bocchi robot controller's WebSocket functionality, fully integrated with ROS2 testing infrastructure. All tests are compatible with `colcon test` and `ros2 test` commands.

## Test Structure

### Core Test Files

#### 1. `test_websocket_comprehensive.py`
**Primary ROS2-integrated test suite**
- ✅ ROS2 node integration testing
- ✅ Twist message publishing (`/cmd_vel` topic)
- ✅ Servo control publishing (`/servo_position` topic)
- ✅ WASD key mapping validation
- ✅ F key servo toggle testing
- ✅ Message format compliance
- ✅ colcon test compatibility

#### 2. `test_websocket_standalone.py`
**Mock-based testing without external dependencies**
- ✅ WebSocket manager functionality
- ✅ Key state management
- ✅ Message broadcasting
- ✅ Performance testing
- ✅ Thread safety validation

#### 3. `test_websocket_functionality.py`
**Core WebSocket protocol testing**
- ✅ Connection lifecycle management
- ✅ Message format validation
- ✅ Broadcasting mechanisms
- ✅ Error handling
- ✅ Performance benchmarks

#### 4. `test_websocket_ros2_integration.py`
**ROS2-specific integration testing**
- ✅ Topic publishing integration
- ✅ Robot control command generation
- ✅ Real-time message processing
- ✅ Multi-client support

#### 5. `test_websocket_security.py`
**Security and protection testing**
- ✅ Input validation
- ✅ Rate limiting
- ✅ DoS protection
- ✅ Connection management
- ✅ Message integrity

#### 6. `test_websocket_real_server.py`
**Real networking and server testing**
- ✅ Actual WebSocket connections
- ✅ Network protocol validation
- ✅ Latency measurement
- ✅ Concurrent client handling

## Running Tests

### Using ROS2 Test Infrastructure

#### Run All Tests
```bash
# From workspace root
cd /workdir/ros2_ws
colcon test --packages-select bocchi
```

#### Run Specific Test Categories
```bash
# Run only WebSocket functionality tests
colcon test --packages-select bocchi --pytest-args="test/test_websocket_functionality.py"

# Run ROS2 integration tests
colcon test --packages-select bocchi --pytest-args="test/test_websocket_comprehensive.py"

# Run security tests
colcon test --packages-select bocchi --pytest-args="test/test_websocket_security.py"
```

#### View Test Results
```bash
# View test results
colcon test-result --all --verbose

# View specific test output
colcon test-result --test-result-base build/bocchi
```

### Using pytest Directly

#### Run All Tests
```bash
cd /workdir/ros2_ws/src/bocchi
python -m pytest test/ -v
```

#### Run with Coverage
```bash
python -m pytest test/ --cov=bocchi --cov-report=html
```

#### Run Specific Test Markers
```bash
# Run only unit tests
python -m pytest test/ -m unit

# Run WebSocket tests
python -m pytest test/ -m websocket

# Run ROS2 integration tests
python -m pytest test/ -m ros2_integration

# Run performance tests
python -m pytest test/ -m performance
```

### Individual Test Execution

#### Comprehensive ROS2 Tests
```bash
python test/test_websocket_comprehensive.py
```

#### Standalone Tests (no external dependencies)
```bash
python test/test_websocket_standalone.py
```

#### Real Server Tests (requires websockets)
```bash
python test/test_websocket_real_server.py
```

## Test Categories and Markers

### Test Markers

Tests are organized using pytest markers for selective execution:

- `@pytest.mark.websocket` - WebSocket functionality tests
- `@pytest.mark.ros2_integration` - ROS2 integration tests  
- `@pytest.mark.security` - Security validation tests
- `@pytest.mark.performance` - Performance and load tests
- `@pytest.mark.unit` - Unit tests
- `@pytest.mark.integration` - Integration tests
- `@pytest.mark.slow` - Long-running tests
- `@pytest.mark.network` - Tests requiring network access

### Test Categories

#### 1. Unit Tests
- WebSocket manager operations
- Key state management
- Message format validation
- Mock-based functionality testing

#### 2. Integration Tests
- ROS2 topic publishing
- WebSocket to robot command flow
- Multi-component interaction
- End-to-end message processing

#### 3. Performance Tests
- Message throughput (target: >50 msg/sec)
- Connection latency (target: <50ms)
- Concurrent client handling
- Memory usage validation

#### 4. Security Tests
- Input validation and sanitization
- Rate limiting and flood protection
- Connection security
- DoS attack protection

## Dependencies

### Required Dependencies
- `rclpy` - ROS2 Python client library
- `geometry_msgs` - For Twist messages
- `std_msgs` - For basic message types
- `pytest` - Test framework
- `unittest` - Python testing framework

### Optional Dependencies
- `websockets` - For real WebSocket testing
- `coverage` - For test coverage analysis
- `pytest-asyncio` - For async test support

### Installing Dependencies
```bash
# Install using rosdep
rosdep install --from-paths . --ignore-src -r -y

# Install Python dependencies
pip install websockets pytest-cov pytest-asyncio
```

## Configuration Files

### `pytest.ini`
Configures pytest behavior for ROS2 compatibility:
- Test discovery patterns
- Markers definition
- Warning filters
- Output formatting

### `package.xml`
Declares test dependencies:
```xml
<test_depend>python3-pytest</test_depend>
<test_depend>python3-websockets</test_depend>
<exec_depend>geometry_msgs</exec_depend>
```

### `setup.py`
Includes test requirements:
```python
tests_require=['pytest', 'websockets']
```

## Continuous Integration

### GitHub Actions Integration
```yaml
- name: Run ROS2 Tests
  run: |
    source /opt/ros/$ROS_DISTRO/setup.bash
    colcon test --packages-select bocchi
    colcon test-result --verbose
```

### Expected Test Results

#### Minimum Requirements
- ✅ **Unit Tests**: 100% pass rate
- ✅ **Integration Tests**: 95%+ pass rate  
- ✅ **Performance Tests**: Meet latency/throughput targets
- ✅ **Security Tests**: All validation tests pass

#### Performance Benchmarks
- **Message Latency**: < 50ms average
- **Throughput**: > 50 messages/second sustained
- **Connection Setup**: < 100ms
- **Memory Usage**: < 50MB for 10 concurrent clients

## Troubleshooting

### Common Issues

#### 1. ROS2 Environment Not Found
```bash
# Solution: Source ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash
```

#### 2. WebSocket Library Missing
```bash
# Solution: Install websockets
pip install websockets==11.0.3
```

#### 3. Port Already in Use
```bash
# Solution: Check for running processes
lsof -i :8765
kill <pid>
```

#### 4. Test Discovery Issues
```bash
# Solution: Run from correct directory
cd /workdir/ros2_ws/src/bocchi
python -m pytest test/ --collect-only
```

### Debugging Tests

#### Verbose Output
```bash
colcon test --packages-select bocchi --event-handlers console_direct+
```

#### Individual Test Debugging
```bash
python -m pytest test/test_websocket_comprehensive.py::TestWebSocketROS2Integration::test_wasd_to_twist_mapping -v -s
```

#### Log Analysis
```bash
# View colcon test logs
tail -f log/latest_test/bocchi/stdout_stderr.log
```

## Test Coverage

### Generating Coverage Reports
```bash
# Run tests with coverage
python -m pytest test/ --cov=bocchi --cov-report=html --cov-report=term

# View HTML report
open htmlcov/index.html
```

### Coverage Targets
- **Overall Coverage**: > 90%
- **WebSocket Manager**: > 95%
- **Key State Management**: > 95%  
- **Message Processing**: > 90%
- **Error Handling**: > 85%

## Development Workflow

### Adding New Tests

1. **Create Test File**
   ```bash
   touch test/test_new_feature.py
   ```

2. **Follow Naming Convention**
   - File: `test_*.py`
   - Class: `Test*`
   - Method: `test_*`

3. **Add Appropriate Markers**
   ```python
   @pytest.mark.websocket
   @pytest.mark.unit
   def test_new_functionality(self):
       pass
   ```

4. **Run Tests**
   ```bash
   colcon test --packages-select bocchi
   ```

### Test-Driven Development

1. Write failing test first
2. Implement minimal code to pass
3. Refactor while keeping tests green
4. Add integration tests
5. Validate with full test suite

## Integration with ROS2 Ecosystem

### colcon Integration
- Tests automatically discovered by colcon
- Results integrated with workspace testing
- Compatible with CI/CD pipelines

### ament_python Compatibility
- Follows ament_python package structure
- Test dependencies properly declared
- Compatible with ROS2 build system

### ROS2 Testing Best Practices
- ✅ Mock external dependencies
- ✅ Test ROS2 message types
- ✅ Validate topic publishing
- ✅ Test node lifecycle
- ✅ Handle async operations properly

## Future Enhancements

### Planned Improvements
1. **Load Testing**: Stress tests with 100+ concurrent clients
2. **Security Hardening**: Additional penetration testing
3. **Performance Optimization**: Sub-10ms latency targets
4. **Monitoring Integration**: Metrics collection during tests
5. **Multi-Robot Testing**: Testing with multiple robot instances

### Contributing

When contributing new tests:
1. Follow ROS2 testing conventions
2. Add appropriate documentation
3. Include performance benchmarks
4. Ensure colcon compatibility
5. Update this README

## References

- [ROS2 Testing Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html)
- [colcon Testing](https://colcon.readthedocs.io/en/released/user/how-to/test-workspace.html)
- [pytest Documentation](https://docs.pytest.org/)
- [WebSocket Protocol RFC](https://tools.ietf.org/html/rfc6455)