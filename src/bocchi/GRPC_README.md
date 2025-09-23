# ROS2 gRPC Keyboard Publisher

This project has been converted from WebSocket to gRPC for better performance, type safety, and cross-platform compatibility.

## Overview

The ROS2 keyboard publisher now uses gRPC instead of WebSocket to receive keyboard input and publish it to the `/keyboard` topic. This conversion provides:

- **Better Performance**: gRPC uses HTTP/2 and Protocol Buffers for efficient binary serialization
- **Type Safety**: Strongly typed message definitions with automatic code generation
- **Cross-Platform**: Works seamlessly across different languages and platforms
- **Streaming Support**: Bidirectional streaming for real-time key input
- **Built-in Error Handling**: Robust error handling and status codes

## Architecture

```
┌─────────────────┐    gRPC     ┌─────────────────┐    ROS2    ┌─────────────────┐
│   gRPC Client   │ ──────────► │  ROS2 Publisher │ ──────────► │   /keyboard     │
│  (Your App)     │             │   (gRPC Server) │             │     Topic       │
└─────────────────┘             └─────────────────┘             └─────────────────┘
```

## Quick Start

### 1. Install Dependencies

```bash
# Install gRPC dependencies
pip3 install grpcio grpcio-tools protobuf

# Or run the setup script
chmod +x setup_grpc.sh
./setup_grpc.sh
```

### 2. Generate gRPC Files (if needed)

```bash
cd bocchi/proto
python3 -m grpc_tools.protoc \
    --proto_path=. \
    --python_out=. \
    --grpc_python_out=. \
    keyboard.proto
```

### 3. Build and Run

```bash
# Build the ROS2 package
cd /path/to/ros2_ws
colcon build --packages-select bocchi

# Source the workspace
source install/setup.bash

# Run the gRPC publisher
ros2 run bocchi publisher
```

### 4. Test with Example Client

```bash
# Run the example Python client
python3 src/bocchi/bocchi/grpc_client_example.py

# Or use grpcurl for quick testing
grpcurl -plaintext localhost:50051 keyboard.KeyboardService/GetStatus
```

## gRPC Service Definition

The keyboard service provides three main methods:

### 1. SendKey (Unary)
Send a single key press to the ROS2 publisher.

```protobuf
rpc SendKey (KeyRequest) returns (KeyResponse);
```

### 2. StreamKeys (Bidirectional Streaming)
Stream key presses in real-time with bidirectional communication.

```protobuf
rpc StreamKeys (stream KeyRequest) returns (stream KeyResponse);
```

### 3. GetStatus (Unary)
Get server status information.

```protobuf
rpc GetStatus (StatusRequest) returns (StatusResponse);
```

## Message Types

### KeyRequest
```protobuf
message KeyRequest {
  string key = 1;           // Key string (e.g., "a", "SPACE", "ENTER")
  int32 key_code = 2;       // Key code (ASCII or other)
  int64 timestamp = 3;      // Unix timestamp in milliseconds
}
```

### KeyResponse
```protobuf
message KeyResponse {
  bool success = 1;         // Whether the key was processed successfully
  string message = 2;       // Optional message (error or confirmation)
  int64 timestamp = 3;      // Server timestamp
}
```

### StatusResponse
```protobuf
message StatusResponse {
  bool server_running = 1;  // Whether the server is running
  int32 connected_clients = 2; // Number of connected clients
  string version = 3;       // Server version
  int64 uptime = 4;        // Server uptime in seconds
}
```

## Client Examples

### Python Client

```python
import grpc
from bocchi.proto import keyboard_pb2, keyboard_pb2_grpc

# Create channel and stub
channel = grpc.insecure_channel('localhost:50051')
stub = keyboard_pb2_grpc.KeyboardServiceStub(channel)

# Send a key
request = keyboard_pb2.KeyRequest(
    key='a', 
    key_code=97, 
    timestamp=int(time.time() * 1000)
)
response = stub.SendKey(request)
print(f"Success: {response.success}, Message: {response.message}")

# Get status
status_request = keyboard_pb2.StatusRequest()
status = stub.GetStatus(status_request)
print(f"Server running: {status.server_running}")
```

### Command Line (grpcurl)

```bash
# Get server status
grpcurl -plaintext localhost:50051 keyboard.KeyboardService/GetStatus

# Send a key press
grpcurl -plaintext \
  -d '{"key": "a", "key_code": 97, "timestamp": 1234567890}' \
  localhost:50051 keyboard.KeyboardService/SendKey

# List available services
grpcurl -plaintext localhost:50051 list

# Describe the service
grpcurl -plaintext localhost:50051 describe keyboard.KeyboardService
```

### Node.js Client

```javascript
const grpc = require('@grpc/grpc-js');
const protoLoader = require('@grpc/proto-loader');

// Load proto
const packageDefinition = protoLoader.loadSync('keyboard.proto');
const keyboard = grpc.loadPackageDefinition(packageDefinition).keyboard;

// Create client
const client = new keyboard.KeyboardService('localhost:50051', 
  grpc.credentials.createInsecure());

// Send a key
const request = { key: 'a', key_code: 97, timestamp: Date.now() };
client.sendKey(request, (error, response) => {
  if (error) {
    console.error('Error:', error);
  } else {
    console.log('Response:', response);
  }
});
```

## Configuration

### Server Configuration
- **Port**: 50051 (configurable in code)
- **Interface**: 0.0.0.0 (listens on all interfaces)
- **Max Workers**: 10 (ThreadPoolExecutor)

### Client Configuration
- **Timeout**: 5 seconds for unary calls
- **Retry**: Implement your own retry logic as needed
- **TLS**: Currently uses insecure connections (can be upgraded)

## Migration from WebSocket

### Key Differences

| Feature | WebSocket | gRPC |
|---------|-----------|------|
| Protocol | HTTP/1.1 Upgrade | HTTP/2 |
| Data Format | JSON | Protocol Buffers |
| Type Safety | Runtime validation | Compile-time validation |
| Streaming | Full duplex | Bidirectional streaming |
| Error Handling | Custom implementation | Built-in status codes |
| Browser Support | Native | Requires gr