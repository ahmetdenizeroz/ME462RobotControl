#!/usr/bin/env python3
import cv2
import numpy as np
import json
import argparse
import math
import sys
import socket

# Canvas dimensions: 1 pixel = 1 millimeter
# Physical Whiteboard is 1.00m x 0.35m -> 1000mm x 350mm
WIDTH = 1000
HEIGHT = 350
WINDOW_NAME = "Delete Solver Simulation"

# Global persistent canvas so we don't wipe the board between network jobs!
global_canvas = np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255

def simulate_job(job, speed):
    global global_canvas
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1200, int(1200 * (HEIGHT / WIDTH)))

    actions = job.get("actions", [])
    if not actions:
        print("No actions found in JSON.")
        return
    
    print(f"Simulating {len(actions)} actions...")

    for action in actions:
        atype = action.get("type", "draw")
        if atype == "same":
            continue # Do not redraw untouched strokes
            
        points = action.get("points", [])
        if len(points) < 2:
            continue

        color = (0, 0, 0) # Black for draw
        thickness = 5 # 5mm pen tip
        is_erase = False

        if "erase_squeegee" in atype:
            color = (255, 255, 255) # White eraser trail
            thickness = 150 # 15cm = 150mm squeegee width
            is_erase = True
        elif "erase_finger" in atype:
            color = (255, 255, 255) # White eraser trail
            thickness = 20 # 2cm = 20mm finger footprint
            is_erase = True

        print(f"[{atype.upper()}] Tracing {len(points)} waypoints...")

        for i in range(1, len(points)):
            pt = points[i]
            x = int(pt[0] * WIDTH)
            y = int((1.0 - pt[1]) * HEIGHT)

            prev_pt = points[i-1]
            px = int(prev_pt[0] * WIDTH)
            py = int((1.0 - prev_pt[1]) * HEIGHT)
            
            # Smooth Animation Interpolation
            dist = math.hypot(x - px, y - py)
            if is_erase and "finger" in atype:
                # Extremely slow cleaning animation for finger raking (0.1 pixels per frame)
                steps = max(1, int(dist / 0.1))
            elif is_erase:
                # Slow animation for squeegee
                steps = max(1, int(dist / 0.5))
            else:
                # Fast drawing animation (5 pixels per frame)
                steps = max(1, int(dist / 5.0))
                # Fast drawing animation (5 pixels per frame)
                steps = max(1, int(dist / 5.0))
            
            for step in range(1, steps + 1):
                inter_x = int(px + (x - px) * (step / steps))
                inter_y = int(py + (y - py) * (step / steps))
                
                prev_inter_x = int(px + (x - px) * ((step - 1) / steps))
                prev_inter_y = int(py + (y - py) * ((step - 1) / steps))
                
                if is_erase and "squeegee" in atype:
                    # Draw the exact geometric rectangle to avoid rounded-cap over-erasing!
                    dx_inter = inter_x - prev_inter_x
                    dy_inter = inter_y - prev_inter_y
                    if dx_inter != 0 or dy_inter != 0:
                        angle = math.atan2(dy_inter, dx_inter)
                        perp_angle = angle + math.pi / 2
                        hl = thickness / 2.0
                        ht = 20 / 2.0 # 2cm thickness
                        v_px, v_py = math.cos(perp_angle)*hl, math.sin(perp_angle)*hl
                        v_nx, v_ny = math.cos(angle)*ht, math.sin(angle)*ht
                        
                        # Rectangle corners
                        rect_pts = np.array([[
                            [inter_x + v_px + v_nx, inter_y + v_py + v_ny],
                            [inter_x - v_px + v_nx, inter_y - v_py + v_ny],
                            [inter_x - v_px - v_nx, inter_y - v_py - v_ny],
                            [inter_x + v_px - v_nx, inter_y + v_py - v_ny]
                        ]], dtype=np.int32)
                        cv2.fillPoly(global_canvas, rect_pts, color)
                elif is_erase and "finger" in atype:
                    dx_inter = inter_x - prev_inter_x
                    dy_inter = inter_y - prev_inter_y
                    if dx_inter != 0 or dy_inter != 0:
                        angle = math.atan2(dy_inter, dx_inter)
                        perp_angle = angle + math.pi / 2
                        hl = thickness / 2.0 # 20mm / 2 = 10mm
                        ht = 5 / 2.0 # 5mm thickness / 2 = 2.5mm
                        v_px, v_py = math.cos(perp_angle)*hl, math.sin(perp_angle)*hl
                        v_nx, v_ny = math.cos(angle)*ht, math.sin(angle)*ht
                        
                        rect_pts = np.array([[
                            [inter_x + v_px + v_nx, inter_y + v_py + v_ny],
                            [inter_x - v_px + v_nx, inter_y - v_py + v_ny],
                            [inter_x - v_px - v_nx, inter_y - v_py - v_ny],
                            [inter_x + v_px - v_nx, inter_y + v_py - v_ny]
                        ]], dtype=np.int32)
                        cv2.fillPoly(global_canvas, rect_pts, color)
                else:
                    cv2.line(global_canvas, (prev_inter_x, prev_inter_y), (inter_x, inter_y), color, thickness)
                
                # Create a display copy so we can draw the "Red Tool Head" without leaving a red trail
                display = global_canvas.copy()
                
                # Draw the Tool Head
                if is_erase:
                    if "squeegee" in atype:
                        # Dynamically calculate the Yaw angle of the stroke direction
                        dx = x - px
                        dy = y - py
                        if dx != 0 or dy != 0:
                            angle = math.atan2(dy, dx)
                            perp_angle = angle + math.pi / 2
                            hl = thickness / 2
                            p1 = (int(inter_x + hl * math.cos(perp_angle)), int(inter_y + hl * math.sin(perp_angle)))
                            p2 = (int(inter_x - hl * math.cos(perp_angle)), int(inter_y - hl * math.sin(perp_angle)))
                            cv2.line(display, p1, p2, (0, 0, 255), 4)
                    else:
                        dx = x - px
                        dy = y - py
                        if dx != 0 or dy != 0:
                            angle = math.atan2(dy, dx)
                            perp_angle = angle + math.pi / 2
                            hl = thickness / 2.0 # 20mm
                            ht = 5 / 2.0 # 5mm
                            v_px, v_py = math.cos(perp_angle)*hl, math.sin(perp_angle)*hl
                            v_nx, v_ny = math.cos(angle)*ht, math.sin(angle)*ht
                            rect_pts_disp = np.array([[
                                [inter_x + v_px + v_nx, inter_y + v_py + v_ny],
                                [inter_x - v_px + v_nx, inter_y - v_py + v_ny],
                                [inter_x - v_px - v_nx, inter_y - v_py - v_ny],
                                [inter_x + v_px - v_nx, inter_y + v_py - v_ny]
                            ]], dtype=np.int32)
                            cv2.polylines(display, [rect_pts_disp], isClosed=True, color=(0, 0, 255), thickness=2)
                else:
                    # Draw a tiny red dot representing the pen tip
                    cv2.circle(display, (inter_x, inter_y), 5, (0, 0, 255), -1)

                cv2.imshow(WINDOW_NAME, display)
                
                # Break if user presses 'q'
                if cv2.waitKey(speed) & 0xFF == ord('q'):
                    print("Simulation aborted by user.")
                    cv2.destroyAllWindows()
                    return

    print("Simulation complete! Waiting for next job...")
    cv2.imshow(WINDOW_NAME, global_canvas)
    cv2.waitKey(500) # show final result briefly

def start_tcp_server(speed):
    HOST = '0.0.0.0'
    PORT = 9090
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print(f"TCP Server listening on {HOST}:{PORT}")
    
    try:
        while True:
            server_socket.settimeout(1.0)
            try:
                conn, addr = server_socket.accept()
            except socket.timeout:
                # Keep OpenCV GUI responsive while waiting
                if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) >= 1:
                    if cv2.waitKey(10) & 0xFF == ord('q'):
                        break
                continue
            except Exception as e:
                print(f"Socket error: {e}")
                break
                
            with conn:
                print(f"Connected by {addr}")
                data = b""
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    data += chunk
                    if b"\n" in chunk:
                        break
                
                if data:
                    try:
                        job = json.loads(data.decode('utf-8', errors='ignore').strip())
                        print("Job received successfully! Starting simulation...")
                        simulate_job(job, speed)
                        conn.sendall(b"SUCCESS\n")
                    except json.JSONDecodeError as e:
                        print(f"Failed to parse JSON: {e}")
                        
    except KeyboardInterrupt:
        print("Server shutting down...")
    finally:
        server_socket.close()
        cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(description="2D Simulation for Delete Solver")
    parser.add_argument("--offline", type=str, default=None, help="Path to JSON job file for offline mode")
    parser.add_argument("--speed", type=int, default=10, help="Playback speed (ms per frame)")
    args = parser.parse_args()

    if args.offline:
        try:
            with open(args.offline, 'r') as f:
                job = json.load(f)
            simulate_job(job, args.speed)
            cv2.waitKey(0) # Keep open until keypress
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"Error loading {args.offline}: {e}")
    else:
        # Default to TCP Server
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 1200, int(1200 * (HEIGHT / WIDTH)))
        start_tcp_server(args.speed)

if __name__ == "__main__":
    main()
