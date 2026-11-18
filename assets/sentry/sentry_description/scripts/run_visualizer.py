#!/usr/bin/env python3
"""
Sentry Mk2 Robot 3D Visualizer HTTP Server Launcher.

Serves the repository directory so that visualizer/index.html and
urdf/meshes/visual/*.stl can be loaded without CORS or file:// limitations.
Automatically detects free ports, opens the default browser, and handles graceful shutdown.
"""

import os
import sys
import argparse
import socket
import webbrowser
import threading
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler

# Repository root directory
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class QuietHTTPRequestHandler(SimpleHTTPRequestHandler):
    """HTTP request handler with directory set to REPO_ROOT and quiet logging."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=REPO_ROOT, **kwargs)

    def log_message(self, format, *args):
        # Only log errors (4xx, 5xx) to keep terminal clean
        if len(args) >= 2 and str(args[1]).startswith(("4", "5")):
            sys.stderr.write(f"[HTTP] {self.address_string()} - {format % args}\n")

    def end_headers(self):
        # Enable CORS headers so local WebGL canvas and web workers can load STL assets
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "X-Requested-With, Content-Type")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        super().end_headers()


def find_available_port(start_port=8000, max_attempts=50, host="127.0.0.1"):
    """Find the first available TCP port starting from start_port."""
    for port in range(start_port, start_port + max_attempts):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.bind((host, port))
                return port
            except OSError:
                continue
    raise RuntimeError(f"Could not find an available port in range {start_port}..{start_port + max_attempts}")


def open_browser_delayed(url, delay_sec=0.5):
    """Open web browser after a short delay to allow server initialization."""
    def _open():
        time.sleep(delay_sec)
        try:
            webbrowser.open(url)
        except Exception as e:
            print(f"[Warning] Could not automatically open browser: {e}")

    thread = threading.Thread(target=_open, daemon=True)
    thread.start()


def main():
    parser = argparse.ArgumentParser(description="Sentry Mk2 Robot 3D Visualizer Launcher")
    parser.add_argument("--host", default="127.0.0.1", help="Host interface to bind (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8000, help="Initial port to bind (default: 8000)")
    parser.add_argument("--no-browser", action="store_true", help="Do not automatically launch web browser")
    parser.add_argument("--timeout", type=float, default=None, help="Automatic shutdown timeout in seconds (optional)")
    args = parser.parse_args()

    port = find_available_port(args.port, host=args.host)
    server_address = (args.host, port)

    try:
        httpd = HTTPServer(server_address, QuietHTTPRequestHandler)
    except Exception as e:
        print(f"[Error] Failed to start HTTP server on {args.host}:{port} - {e}", file=sys.stderr)
        sys.exit(1)

    vis_url = f"http://{args.host}:{port}/visualizer/index.html"

    print("=" * 76)
    print("  SENTRY MK2 ROBOT 3D VISUALIZER SERVER")
    print("=" * 76)
    print(f"  Visualizer URL : {vis_url}")
    print(f"  Root Directory : {REPO_ROOT}")
    print(f"  Server Host    : {args.host}")
    print(f"  Server Port    : {port}")
    print("-" * 76)
    print("  Features available in visualizer:")
    print("    * 7-DOF Joint Articulation Sliders (4 wheels, major yaw, minor yaw, minor pitch)")
    print("    * Display Modes: Coarse Visual Meshes / Analytical Primitives / Both")
    print("    * Preset Viewpoints (Isometric, Front, Side, Top, Rear)")
    print("    * Coordinate Frames, Joint Axes, and Ground Grid Inspection Overlays")
    print("    * Joint Reset (q = 0) & Automated Motion Preview Animation")
    print("=" * 76)
    print("  Press Ctrl+C to terminate server.\n")

    if not args.no_browser:
        print(f"  Launching web browser: {vis_url}...")
        open_browser_delayed(vis_url, delay_sec=0.5)

    if args.timeout:
        timer = threading.Timer(args.timeout, lambda: httpd.shutdown())
        timer.start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[Shutdown] KeyboardInterrupt received. Stopping visualizer server...")
    finally:
        httpd.server_close()
        print("[Shutdown] Visualizer server stopped successfully.")


if __name__ == "__main__":
    main()
