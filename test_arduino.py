#!/usr/bin/env python3
"""
Test script to check if Arduino is accessible from within the devcontainer.
Usage: python test_arduino.py [device_path]
Default device: /dev/ttyACM0 (Linux) or COM3 (Windows)
Supports both Linux (/dev/tty*) and Windows (COM*) ports.
"""

import sys
import os
import platform
import serial
import serial.tools.list_ports

def test_device_exists(device_path):
    """Check if the device file exists (Linux) or port is available (Windows)"""
    print(f"Checking if {device_path} exists...")
    
    # On Windows, COM ports don't exist as files, check via pyserial instead
    if platform.system() == "Windows" and device_path.upper().startswith("COM"):
        # Check if port is in available ports list
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if device_path.upper() in [p.upper() for p in ports]:
            print(f"✓ COM port {device_path} is available")
            return True
        else:
            print(f"✗ COM port {device_path} is not available")
            return False
    
    # On Linux, check if device file exists
    if os.path.exists(device_path):
        print(f"✓ Device {device_path} exists")
        
        # Check permissions
        stat_info = os.stat(device_path)
        print(f"  Permissions: {oct(stat_info.st_mode)}")
        print(f"  Owner UID: {stat_info.st_uid}, GID: {stat_info.st_gid}")
        
        # Check if readable/writable
        if os.access(device_path, os.R_OK):
            print(f"  ✓ Readable")
        else:
            print(f"  ✗ Not readable")
            
        if os.access(device_path, os.W_OK):
            print(f"  ✓ Writable")
        else:
            print(f"  ✗ Not writable")
            
        return True
    else:
        print(f"✗ Device {device_path} does not exist")
        return False

def test_serial_connection(device_path):
    """Try to open a serial connection"""
    print(f"\nAttempting to open serial connection to {device_path}...")
    try:
        ser = serial.Serial(
            device_path,
            115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1.0
        )
        print(f"✓ Successfully opened serial connection")
        print(f"  Port: {ser.port}")
        print(f"  Baudrate: {ser.baudrate}")
        print(f"  Is open: {ser.is_open}")
        
        # Try to read (non-blocking)
        print(f"\nAttempting to read from device (timeout: 1s)...")
        try:
            data = ser.read(1)
            if data:
                print(f"  ✓ Received data: {data.hex()}")
            else:
                print(f"  ⚠ No data received (device may be idle)")
        except Exception as e:
            print(f"  ⚠ Read error: {e}")
        
        ser.close()
        print(f"\n✓ Serial connection test completed successfully")
        return True
        
    except serial.SerialException as e:
        print(f"✗ Serial connection failed: {e}")
        return False
    except PermissionError as e:
        print(f"✗ Permission denied: {e}")
        print(f"  Try adding your user to the 'dialout' group:")
        print(f"    sudo usermod -aG dialout $USER")
        return False
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return False

def list_available_ports():
    """List all available serial ports"""
    print("\n" + "="*60)
    print("Available serial ports:")
    print("="*60)
    ports = serial.tools.list_ports.comports()
    if ports:
        for port in ports:
            print(f"  {port.device}")
            print(f"    Description: {port.description}")
            print(f"    Hardware ID: {port.hwid}")
            print()
    else:
        print("  No serial ports found")
    print("="*60)

if __name__ == "__main__":
    # Default device based on platform
    if platform.system() == "Windows":
        default_device = "COM3"
    else:
        default_device = "/dev/ttyACM0"
    
    device_path = sys.argv[1] if len(sys.argv) > 1 else default_device
    
    print("="*60)
    print("Arduino Access Test")
    print("="*60)
    print(f"Platform: {platform.system()}")
    print(f"Testing device: {device_path}\n")
    
    # List available ports first
    list_available_ports()
    
    # Test device existence
    exists = test_device_exists(device_path)
    
    if exists:
        # Test serial connection
        test_serial_connection(device_path)
    else:
        print("\n" + "="*60)
        print("Troubleshooting:")
        print("="*60)
        print("1. Make sure your Arduino is connected via USB")
        print("2. Check if the device appears in the list above")
        print("3. If using a devcontainer:")
        if platform.system() == "Windows":
            print("   - Docker Desktop should handle COM ports automatically")
            print("   - Ensure Docker Desktop has access to USB devices")
            print("   - Try running: docker run --privileged -it <image>")
        else:
            print("   - Ensure 'runArgs': ['--privileged'] is in devcontainer.json")
            print("   - Rebuild container: 'Dev Containers: Rebuild Container'")
        print("4. If permission denied (Linux):")
        print("   - Add user to dialout group: sudo usermod -aG dialout $USER")
        print("   - Log out and back in, or restart container")
        print("="*60)


