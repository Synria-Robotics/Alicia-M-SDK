#!/usr/bin/env python3
# Copyright (c) 2025 Synria Robotics Co., Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
Benchmark: Read joint angles frequency

Features:
- Benchmark real read frequency (with serial communication wait)
- Measure single call latency (serial communication response time)
- Statistics: average, min, max, std deviation
"""

import time
import argparse
import alicia_m_sdk
from alicia_m_sdk.hardware import ServoDriver
import numpy as np
import statistics

def main(args):
    """Benchmark joint reading frequency with real serial communication.
    
    :param args: Command line arguments
    """
    # Initialize robot instance
    robot = alicia_m_sdk.create_robot(
        port=args.port,
        gripper_type=args.gripper_type,
        robot_version=args.robot_version,
        base_link=args.base_link,
        end_link=args.end_link,
        control_aim=args.control_aim,
        control_mode=args.control_mode,
        baudrate=args.baudrate
    )
    
    try:
        print(f"\n{'='*60}")
        print(f"Read Frequency Benchmark (Real Serial Communication)")
        print(f"{'='*60}")
        print(f"Duration: {args.duration}s")
        print(f"Control Mode: {args.control_mode}")
        print(f"Control Aim: {args.control_aim}")
        print(f"Baudrate: {args.baudrate}")
        print(f"{'='*60}\n")
        
        # Pause background update thread to force real serial communication
        print("Pausing background update thread to test real read frequency...")
        robot.servo_driver._pause_update.set()
        time.sleep(0.1)  # Wait for current update to finish
        
        print("Starting benchmark...")
        print("(Each call will wait for serial response)")
        
        # Statistics collection
        call_times = []
        success_count = 0
        fail_count = 0
        
        # Warm-up: send a few requests to stabilize
        print("Warming up...")
        for _ in range(3):
            try:
                robot.get_robot_state("joint", timeout=1.0)
            except:
                pass
        time.sleep(0.1)
        
        # Main benchmark loop
        start_time = time.perf_counter()
        end_time = start_time + args.duration
        
        iteration = 0
        while time.perf_counter() < end_time:
            iteration += 1
            call_start = time.perf_counter()
            
            try:
                joints = robot.get_robot_state("joint", timeout=1.0)
                call_end = time.perf_counter()
                call_duration = call_end - call_start
                call_times.append(call_duration * 1000)  # Convert to ms
                
                if joints is not None:
                    success_count += 1
                else:
                    fail_count += 1
                    
            except Exception as e:
                call_end = time.perf_counter()
                call_duration = call_end - call_start
                call_times.append(call_duration * 1000)
                fail_count += 1
                if args.verbose:
                    print(f"Error at iteration {iteration}: {e}")
        
        actual_duration = time.perf_counter() - start_time
        total_calls = len(call_times)
        
        # Restore background thread
        robot.servo_driver._pause_update.clear()
        
        # Calculate statistics
        if total_calls > 0:
            avg_time_ms = np.mean(call_times)
            min_time_ms = np.min(call_times)
            max_time_ms = np.max(call_times)
            median_time_ms = np.median(call_times)
            
            if len(call_times) > 1:
                std_time_ms = statistics.stdev(call_times)
                p95_time_ms = np.percentile(call_times, 95)
                p99_time_ms = np.percentile(call_times, 99)
            else:
                std_time_ms = 0.0
                p95_time_ms = avg_time_ms
                p99_time_ms = avg_time_ms
            
            # Calculate frequencies
            avg_freq = 1000.0 / avg_time_ms if avg_time_ms > 0 else 0.0
            max_freq = 1000.0 / min_time_ms if min_time_ms > 0 else 0.0
            min_freq = 1000.0 / max_time_ms if max_time_ms > 0 else 0.0
            actual_freq = total_calls / actual_duration
            
            # Print results
            print(f"\n{'='*60}")
            print(f"Results ({actual_duration:.2f}s, {total_calls} calls)")
            print(f"{'='*60}")
            
            print(f"\n📊 Read Frequency Statistics:")
            print(f"  Actual Read Frequency:     {actual_freq:10.2f} Hz")
            print(f"  Average Frequency:         {avg_freq:10.2f} Hz")
            print(f"  Maximum Frequency:         {max_freq:10.2f} Hz")
            print(f"  Minimum Frequency:         {min_freq:10.2f} Hz")
            
            print(f"\n⏱️  Latency Statistics (per call):")
            print(f"  Average Latency:           {avg_time_ms:10.3f} ms")
            print(f"  Median Latency:            {median_time_ms:10.3f} ms")
            print(f"  Minimum Latency:           {min_time_ms:10.3f} ms")
            print(f"  Maximum Latency:           {max_time_ms:10.3f} ms")
            print(f"  Std Deviation:             {std_time_ms:10.3f} ms")
            print(f"  95th Percentile:           {p95_time_ms:10.3f} ms")
            print(f"  99th Percentile:           {p99_time_ms:10.3f} ms")
            
            print(f"\n✅ Success Rate:")
            print(f"  Successful Calls:          {success_count:10d} ({success_count/total_calls*100:.1f}%)")
            print(f"  Failed Calls:              {fail_count:10d} ({fail_count/total_calls*100:.1f}%)")
            
            # Performance analysis
            print(f"\n📈 Performance Analysis:")
            if avg_time_ms < 2.0:
                print(f"  ⚡ Excellent: Average latency < 2ms (theoretical max > 500Hz)")
            elif avg_time_ms < 5.0:
                print(f"  ✅ Good: Average latency < 5ms (theoretical max > 200Hz)")
            elif avg_time_ms < 10.0:
                print(f"  ⚠️  Moderate: Average latency < 10ms (theoretical max > 100Hz)")
            else:
                print(f"  ⚠️  Slow: Average latency >= 10ms (theoretical max < 100Hz)")
            
            if std_time_ms / avg_time_ms > 0.3:
                print(f"  ⚠️  Warning: High latency variance (std/avg = {std_time_ms/avg_time_ms:.2f})")
                print(f"      This may indicate unstable serial communication.")
            
            print(f"\n{'='*60}")
            print(f"💡 Notes:")
            print(f"  - This benchmark tests REAL serial communication (background thread paused)")
            print(f"  - Each call waits for serial response (one-request-one-response mechanism)")
            print(f"  - Theoretical max frequency = 1000 / average_latency_ms Hz")
            print(f"  - If latency is high, check:")
            print(f"    * Serial port baudrate (current: {args.baudrate})")
            print(f"    * USB cable quality")
            print(f"    * System load")
            print(f"{'='*60}\n")
        else:
            print("Error: No calls completed!")
            
    except KeyboardInterrupt:
        print("\n✗ Benchmark interrupted")
        # Restore background thread if it was paused
        if hasattr(robot, 'servo_driver') and hasattr(robot.servo_driver, '_pause_update'):
            robot.servo_driver._pause_update.clear()
    
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        # Restore background thread if it was paused
        if hasattr(robot, 'servo_driver') and hasattr(robot.servo_driver, '_pause_update'):
            robot.servo_driver._pause_update.clear()
    
    finally:
        # Ensure background thread is restored
        if hasattr(robot, 'servo_driver') and hasattr(robot.servo_driver, '_pause_update'):
            robot.servo_driver._pause_update.clear()
        robot.disconnect()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Benchmark robot joint reading frequency (real serial communication)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic benchmark (5 seconds, default settings)
  python 14_benchmark_read_joints.py

  # Extended benchmark (10 seconds)
  python 14_benchmark_read_joints.py --duration 10

  # Verbose output (show errors)
  python 14_benchmark_read_joints.py --verbose

  # Specify serial port and baudrate
  python 14_benchmark_read_joints.py --port /dev/ttyUSB0 --baudrate 1000000
        """
    )
    
    # Serial port settings
    parser.add_argument('--port', type=str, default="", 
                       help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--gripper_type', type=str, default="100mm", 
                       help="夹爪型号 (默认: 100mm)")
    parser.add_argument('--robot_version', type=str, default="v1_0", 
                       help="机械臂版本")
    parser.add_argument('--base_link', type=str, default="base_link", 
                       help="基座链路名称")
    parser.add_argument('--end_link', type=str, default="tool0", 
                       help="末端执行器链路名称")
    parser.add_argument('--baudrate', type=int, default=1000000,
                       help="串口波特率 (默认: 1000000)")
    
    # Control settings
    parser.add_argument('--control-aim', type=str, default='teach', 
                       choices=['teach', 'operation'],
                       help='Control aim: teach or operation (motor-specific, auto-detected if not specified)')
    parser.add_argument('--control-mode', type=str, default='pv', 
                       choices=['pv', 'pvt', 'v', 'mit', 'mit_position', 'mit_speed', 'mit_torque'],
                       help='Control mode: pv, pvt, v, mit, mit_position, mit_speed, mit_torque')
    
    # Benchmark settings
    parser.add_argument('--duration', type=float, default=5.0, 
                       help="Benchmark duration in seconds (default: 5.0)")
    parser.add_argument('--verbose', action='store_true',
                       help="Show verbose output including errors")
    
    args = parser.parse_args()
    main(args)
