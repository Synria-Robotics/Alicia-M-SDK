"""15_demo_sparkvis.py — SparkVis 可视化

演示通过 WebSocket 实时推送关节状态到 SparkVis 可视化界面。
按 Ctrl+C 退出。

依赖: pip install websockets (可选依赖)

使用方式:
  1. 启动 SparkVis 后端:  cd SparkVis && python backend_server.py
  2. 启动 SparkVis 前端:  cd SparkVis && python -m http.server 8080
  3. 运行本脚本:          python 12_demo_sparkvis.py
  4. 浏览器打开:          http://localhost:8080
"""

import json
import math
import time
import asyncio
import alicia_m_sdk
from robocore.utils.beauty_logger import beauty_print, beauty_print_array

# WebSocket 服务配置
WS_HOST = "localhost"
WS_PORT = 8765
PUSH_RATE_HZ = 50  # 推送频率 (Hz)


async def _ws_push_loop(robot, host: str, port: int, rate_hz: float):
    """WebSocket 推送循环：持续发送关节状态

    Args:
        robot: 机器人实例
        host: WebSocket 主机地址
        port: WebSocket 端口
        rate_hz: 推送频率 (Hz)
    """
    try:
        import websockets
    except ImportError:
        beauty_print("缺少 websockets 依赖，请安装: pip install websockets", type="warning")
        return

    interval = 1.0 / rate_hz

    beauty_print(f"启动 WebSocket 服务: ws://{host}:{port}", type="info")
    beauty_print(f"推送频率: {rate_hz} Hz", type="info")
    beauty_print("等待 SparkVis 连接...", type="info")

    # 已连接的客户端集合
    connected_clients = set()

    async def _handler(websocket):
        """处理 WebSocket 客户端连接"""
        connected_clients.add(websocket)
        client_addr = websocket.remote_address
        beauty_print(f"SparkVis 客户端已连接: {client_addr}", type="success")
        try:
            # 保持连接，接收客户端消息（如果有）
            async for message in websocket:
                pass  # SparkVis -> Robot 的消息可在此处理
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            connected_clients.discard(websocket)
            beauty_print(f"SparkVis 客户端已断开: {client_addr}", type="warning")

    async def _broadcast_loop():
        """广播关节状态到所有已连接的客户端"""
        while True:
            if connected_clients:
                # 读取当前关节状态
                state = robot.get_robot_state("all")
                if state is not None:
                    # 构建推送数据
                    angles_deg = [a * 180.0 / math.pi for a in state.angles]
                    data = {
                        "type": "joint_state",
                        "timestamp": time.time(),
                        "joints_rad": state.angles,
                        "joints_deg": angles_deg,
                        "gripper": state.gripper,
                    }
                    # 添加速度和力矩（如果可用）
                    if state.velocities is not None:
                        data["velocities"] = state.velocities
                    if state.torques is not None:
                        data["torques"] = state.torques

                    message = json.dumps(data)

                    # 广播到所有客户端
                    disconnected = set()
                    for ws in connected_clients:
                        try:
                            await ws.send(message)
                        except websockets.exceptions.ConnectionClosed:
                            disconnected.add(ws)
                    connected_clients.difference_update(disconnected)

            await asyncio.sleep(interval)

    # 启动 WebSocket 服务和广播循环
    async with websockets.serve(_handler, host, port):
        beauty_print(f"WebSocket 服务已启动: ws://{host}:{port}", type="success")
        beauty_print("按 Ctrl+C 停止", type="info")
        await _broadcast_loop()


def main():
    beauty_print("Demo: SparkVis 可视化 (WebSocket)", type="module")

    # 创建并连接机器人
    robot = alicia_m_sdk.create_robot()
    beauty_print("机器人连接成功", type="success")

    try:
        # 启动 WebSocket 推送
        asyncio.run(_ws_push_loop(robot, WS_HOST, WS_PORT, PUSH_RATE_HZ))

    except KeyboardInterrupt:
        beauty_print("\n用户中断，停止 WebSocket 服务", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
