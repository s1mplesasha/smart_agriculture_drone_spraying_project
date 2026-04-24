#!/usr/bin/env python3
"""
telemetry_dashboard – Lightweight HTTP + SSE server for live drone telemetry.

Serves dashboard.html on http://localhost:8080 and pushes /drone/telemetry
JSON to connected browsers via Server-Sent Events. Zero external dependencies.
"""

import json
import os
import threading
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler
from io import BytesIO

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class SSEHandler(SimpleHTTPRequestHandler):
    """HTTP handler: serves dashboard.html and /events SSE stream."""

    server_version = "DroneTelemetryDashboard/1.0"
    latest_telemetry = '{}'
    _lock = threading.Lock()

    @classmethod
    def update_telemetry(cls, data: str):
        with cls._lock:
            cls.latest_telemetry = data

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            self._serve_dashboard()
        elif self.path == '/events':
            self._serve_sse()
        else:
            self.send_error(404)

    def _serve_dashboard(self):
        try:
            pkg = get_package_share_directory('drone_sprayer')
            html_path = os.path.join(pkg, 'dashboard.html')
            with open(html_path, 'rb') as f:
                content = f.read()
        except Exception:
            # Fallback: try relative to this file
            here = os.path.dirname(os.path.abspath(__file__))
            html_path = os.path.join(here, 'dashboard.html')
            with open(html_path, 'rb') as f:
                content = f.read()

        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def _serve_sse(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        try:
            while True:
                with self._lock:
                    data = self.latest_telemetry
                self.wfile.write(f'data: {data}\n\n'.encode('utf-8'))
                self.wfile.flush()
                time.sleep(0.5)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def log_message(self, format, *args):
        pass  # Suppress HTTP logs


class TelemetryDashboard(Node):

    def __init__(self):
        super().__init__('telemetry_dashboard')
        self.declare_parameter('http_port', 8080)
        self.port = int(self.get_parameter('http_port').value)

        self.create_subscription(String, '/drone/telemetry', self._on_telemetry, 10)

        # Start HTTP server in background thread
        self.httpd = HTTPServer(('0.0.0.0', self.port), SSEHandler)
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()

        self.get_logger().info(f'📊 Dashboard live at http://localhost:{self.port}')

    def _on_telemetry(self, msg: String):
        SSEHandler.update_telemetry(msg.data)

    def destroy_node(self):
        self.httpd.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
