#!/usr/bin/env python3
import curses
import os
import sys

COMMANDS = [
    {
        "title": "Build ROS 2 Workspace",
        "description": "Builds all packages in the workspace and symlinks Python scripts.",
        "cmd": "cd ~/ws_me462 && colcon build --symlink-install && source install/setup.bash",
        "tags": "build ros2 colcon make compile"
    },
    {
        "title": "Start Drawing Job Executor",
        "description": "Starts the TCP server for automated drawings.",
        "cmd": "ros2 run zenpool_draw_letter drawing_job_executor.py",
        "tags": "draw executor server job"
    },
    {
        "title": "Run Full Board Cleaning Sequence",
        "description": "Automatically attaches the squeegee and wipes the entire board.",
        "cmd": "ros2 run zenpool_draw_letter full_cleaning.py",
        "tags": "clean wipe squeegee board full"
    },
    {
        "title": "Move to Absolute Cartesian (cm)",
        "description": "Move TCP to an exact X/Y/Z coordinate (aligned with table edges).",
        "cmd": "ros2 topic pub --once /dashboard/move_absolute_cartesian geometry_msgs/msg/Vector3 \"{x: 29.0, y: 61.7, z: 25.0}\"",
        "tags": "move position absolute cartesian ik dashboard xyz"
    },
    {
        "title": "Move to Relative Cartesian (cm)",
        "description": "Move TCP relative to current position.",
        "cmd": "ros2 topic pub --once /dashboard/move_cartesian geometry_msgs/msg/Vector3 \"{x: 10.0, y: 0.0, z: -5.0}\"",
        "tags": "move position relative cartesian ik dashboard xyz offset"
    },
    {
        "title": "Move Manhattan (Obstacle Dodging)",
        "description": "Move relative, automatically hovers in an L-shape if obstacles exist.",
        "cmd": "ros2 topic pub --once /dashboard/move_manhattan_cartesian geometry_msgs/msg/Vector3 \"{x: 10.0, y: 0.0, z: 0.0}\"",
        "tags": "move position relative cartesian ik manhattan dodge hover obstacle"
    },
    {
        "title": "Re-orient Tool (Degrees)",
        "description": "Change the Pitch/Roll/Yaw of the tool frame. x:0, y:0 is perfectly squared downwards.",
        "cmd": "ros2 topic pub --once /dashboard/reorient geometry_msgs/msg/Vector3 \"{x: 15.0, y: -10.0, z: 0.0}\"",
        "tags": "rotate orientation pitch roll yaw tool dashboard"
    },
    {
        "title": "Adjust Cartesian Speed",
        "description": "Change IK movement speed (0.0 to 1.0). Default is 0.2.",
        "cmd": "ros2 topic pub --once /dashboard/set_speed std_msgs/msg/Float32MultiArray \"{data: [0.5]}\"",
        "tags": "speed velocity fast slow cartesian dashboard"
    },
    {
        "title": "Attach Tool: Tutan-Khamun Gripper",
        "description": "Pick up the active gripper from the toolbox.",
        "cmd": "ros2 topic pub --once /tool_manager/command std_msgs/msg/String \"{data: 'attach tutankhamun'}\"",
        "tags": "tool attach pickup tutan tutankhamun gripper"
    },
    {
        "title": "Detach Tool: Tutan-Khamun Gripper",
        "description": "Drop the active gripper into the toolbox.",
        "cmd": "ros2 topic pub --once /tool_manager/command std_msgs/msg/String \"{data: 'detach tutankhamun'}\"",
        "tags": "tool detach drop tutan tutankhamun gripper"
    },
    {
        "title": "Gripper: Open Fully",
        "description": "Command Tutan-Khamun to open both fingers completely.",
        "cmd": "ros2 topic pub --once /gripper/command std_msgs/msg/Float32 \"{data: 0.0}\"",
        "tags": "gripper open release fingers tutan"
    },
    {
        "title": "Gripper: Close Fully",
        "description": "Command Tutan-Khamun to close both fingers completely.",
        "cmd": "ros2 topic pub --once /gripper/command std_msgs/msg/Float32 \"{data: 1.0}\"",
        "tags": "gripper close grasp fingers tutan"
    },
    {
        "title": "Gripper: Compliant Grasp",
        "description": "Trigger the wheel-mode force-feedback grasp logic.",
        "cmd": "ros2 topic pub --once /gripper/grasp std_msgs/msg/Bool \"{data: true}\"",
        "tags": "gripper close grasp compliant force feedback wheel tutan"
    },
    {
        "title": "Listen to Gripper Telemetry",
        "description": "Echo the 50Hz motor positions and efforts.",
        "cmd": "ros2 topic echo /gripper/joint_states",
        "tags": "gripper telemetry listen echo position effort"
    },
    {
        "title": "Start RViz Teleoperation",
        "description": "Launch the interactive marker teleoperation node.",
        "cmd": "ros2 run zenpool_draw_letter zenpool_teleop_servo",
        "tags": "rviz teleop manual interactive marker drag"
    },
    {
        "title": "Start AI Face Tracking",
        "description": "Launch the visual servoing script for face tracking.",
        "cmd": "ros2 run zenpool_draw_letter face_tracker_servo.py",
        "tags": "ai face track visual servoing camera"
    },
    {
        "title": "Start Docker Container",
        "description": "Start the Docker container in the background.",
        "cmd": "docker compose up -d",
        "tags": "docker compose start background"
    },
    {
        "title": "Enter Docker Container",
        "description": "Enter the interactive ROS 2 terminal inside the container.",
        "cmd": "docker exec -it tutan_gripper_ros bash",
        "tags": "docker exec enter terminal bash"
    },
    {
        "title": "Stop Docker Container",
        "description": "Stop the Docker container.",
        "cmd": "docker compose down",
        "tags": "docker compose stop down"
    },
    {
        "title": "Start Gripper Driver Node",
        "description": "Start the Tutan-Khamun Gripper driver node.",
        "cmd": "ros2 run tutan_gripper gripper_driver_node",
        "tags": "gripper driver start node tutan"
    },
    {
        "title": "Gripper: Close Halfway",
        "description": "Command Tutan-Khamun to close both fingers 50%.",
        "cmd": "ros2 topic pub --once /gripper/command std_msgs/msg/Float32 \"{data: 0.5}\"",
        "tags": "gripper close halfway 50 percent tutan"
    },
    {
        "title": "Gripper: Open Left Finger",
        "description": "Command ONLY the LEFT finger to OPEN fully.",
        "cmd": "ros2 topic pub --once /gripper/command_left std_msgs/msg/Float32 \"{data: 0.0}\"",
        "tags": "gripper open left finger tutan"
    },
    {
        "title": "Gripper: Close Right Finger",
        "description": "Command ONLY the RIGHT finger to CLOSE fully.",
        "cmd": "ros2 topic pub --once /gripper/command_right std_msgs/msg/Float32 \"{data: 1.0}\"",
        "tags": "gripper close right finger tutan"
    },
    {
        "title": "Gripper: Set Hardware Limits",
        "description": "Change safety limits, grasp threshold, speed, and squeeze margin.",
        "cmd": "ros2 topic pub --once /gripper/set_limits std_msgs/msg/Int32MultiArray \"{data: [2630, 1830, 2070, 2370, 200, 300, 50]}\"",
        "tags": "gripper limits safety threshold speed squeeze margin tutan"
    },
    {
        "title": "Listen to Gripper Temperature",
        "description": "Echo the motor temperature telemetry.",
        "cmd": "ros2 topic echo /gripper/temperature",
        "tags": "gripper temperature listen echo heat tutan"
    },
    {
        "title": "Listen to Gripper Current",
        "description": "Echo the motor electrical current telemetry.",
        "cmd": "ros2 topic echo /gripper/current",
        "tags": "gripper current listen echo electrical tutan"
    },
    {
        "title": "Gripper: Set RAW Left Motor",
        "description": "Bypass software limits and send a direct raw hardware position to the LEFT motor.",
        "cmd": "ros2 topic pub --once /gripper/command_raw_left std_msgs/msg/Int32 \"{data: 2150}\"",
        "tags": "gripper raw direct hardware position left motor tutan"
    },
    {
        "title": "Gripper: Set RAW Right Motor",
        "description": "Bypass software limits and send a direct raw hardware position to the RIGHT motor.",
        "cmd": "ros2 topic pub --once /gripper/command_raw_right std_msgs/msg/Int32 \"{data: 2150}\"",
        "tags": "gripper raw direct hardware position right motor tutan"
    },
    {
        "title": "View Live UDP Video Stream",
        "description": "View the live camera stream. Run locally on PC, NOT on Raspberry Pi.",
        "cmd": "python udp_viewer.py",
        "tags": "video stream live udp viewer camera"
    },
    {
        "title": "Start ArUco Native Camera Script",
        "description": "Start camera tracking script. Run on Pi Host terminal, OUTSIDE Docker.",
        "cmd": "python3 ~/generalrobotcontrol/tutan-khamun/STServo_Python/littleDaisies/video/aruco_tracker.py",
        "tags": "aruco track camera native pi host video outside docker"
    },
    {
        "title": "Start Custom MoveIt Dashboard",
        "description": "Background node for kinematics and trajectory commands.",
        "cmd": "ros2 run zenpool_draw_letter custom_dashboard.py",
        "tags": "dashboard kinematics trajectory custom"
    },
    {
        "title": "Dashboard: Move Joints",
        "description": "Move robot to specific joint angles in degrees.",
        "cmd": "ros2 topic pub --once /dashboard/move_joints std_msgs/msg/Float32MultiArray \"{data: [-137.0, -140.0, -50.0, -80.0, 90.0, 178.0]}\"",
        "tags": "dashboard move joints degrees trajectory"
    },
    {
        "title": "Dashboard: Move Relative Cartesian",
        "description": "Translate X/Y/Z relative to current position in cm.",
        "cmd": "ros2 topic pub --once /dashboard/move_cartesian geometry_msgs/msg/Vector3 \"{x: 10.0, y: 0.0, z: -5.0}\"",
        "tags": "dashboard move relative cartesian translate"
    },
    {
        "title": "Dashboard: Move Absolute Cartesian",
        "description": "Move to exact X/Y/Z coordinates on the table in cm.",
        "cmd": "ros2 topic pub --once /dashboard/move_absolute_cartesian geometry_msgs/msg/Vector3 \"{x: 29.0, y: 61.7, z: 25.0}\"",
        "tags": "dashboard move absolute cartesian table coordinates"
    },
    {
        "title": "Dashboard: Move Relative Manhattan",
        "description": "Relative X/Y/Z translation with smart collision dodging hover path.",
        "cmd": "ros2 topic pub --once /dashboard/move_manhattan_cartesian geometry_msgs/msg/Vector3 \"{x: 10.0, y: 0.0, z: 0.0}\"",
        "tags": "dashboard move relative manhattan dodge hover obstacle"
    },
    {
        "title": "Dashboard: Move Absolute Manhattan",
        "description": "Absolute X/Y/Z translation with smart collision dodging hover path.",
        "cmd": "ros2 topic pub --once /dashboard/move_absolute_manhattan geometry_msgs/msg/Vector3 \"{x: 29.0, y: 61.7, z: 25.0}\"",
        "tags": "dashboard move absolute manhattan dodge hover obstacle"
    },
    {
        "title": "Dashboard: Re-orient Wrist",
        "description": "Rotate wrist (pitch/roll/yaw in degrees) to align with base parallel.",
        "cmd": "ros2 topic pub --once /dashboard/reorient geometry_msgs/msg/Vector3 \"{x: 15.0, y: -10.0, z: 0.0}\"",
        "tags": "dashboard reorient wrist rotation pitch roll yaw parallel"
    },
    {
        "title": "Dashboard: Set Speed",
        "description": "Adjust global cartesian speed multiplier (0.0 to 1.0).",
        "cmd": "ros2 topic pub --once /dashboard/set_speed std_msgs/msg/Float32MultiArray \"{data: [0.5]}\"",
        "tags": "dashboard set speed cartesian velocity"
    },
    {
        "title": "Start Tool Manager",
        "description": "Start background node for spawning CAD tools and collision math.",
        "cmd": "ros2 run zenpool_draw_letter tool_manager.py",
        "tags": "tool manager spawn attach detach collision"
    },
    {
        "title": "Tool Manager: Attach Tutan-Khamun",
        "description": "Virtually attach the gripper to the robot's end effector.",
        "cmd": "ros2 topic pub --once /tool_manager/command std_msgs/msg/String \"{data: 'attach tutankhamun'}\"",
        "tags": "tool manager attach tutankhamun gripper"
    },
    {
        "title": "Tool Manager: Attach Cleaner",
        "description": "Virtually attach the squeegee cleaner to the robot's end effector.",
        "cmd": "ros2 topic pub --once /tool_manager/command std_msgs/msg/String \"{data: 'attach cleaner'}\"",
        "tags": "tool manager attach cleaner squeegee"
    },
    {
        "title": "Tool Manager: Attach Pen",
        "description": "Virtually attach the pen tool to the robot's end effector.",
        "cmd": "ros2 topic pub --once /tool_manager/command std_msgs/msg/String \"{data: 'attach pen'}\"",
        "tags": "tool manager attach pen"
    },
    {
        "title": "Tool Manager: Detach Tutan-Khamun",
        "description": "Virtually drop the gripper into the toolbox.",
        "cmd": "ros2 topic pub --once /tool_manager/command std_msgs/msg/String \"{data: 'detach tutankhamun'}\"",
        "tags": "tool manager detach tutankhamun drop"
    },
    {
        "title": "Tool Manager: Detach Cleaner",
        "description": "Virtually drop the cleaner into the toolbox.",
        "cmd": "ros2 topic pub --once /tool_manager/command std_msgs/msg/String \"{data: 'detach cleaner'}\"",
        "tags": "tool manager detach cleaner drop"
    },
    {
        "title": "Tool Manager: Detach Pen",
        "description": "Virtually drop the pen into the toolbox.",
        "cmd": "ros2 topic pub --once /tool_manager/command std_msgs/msg/String \"{data: 'detach pen'}\"",
        "tags": "tool manager detach pen drop"
    },
    {
        "title": "Start Drawing Job Executor",
        "description": "Start the main TCP server for handling drawing and erasing strokes.",
        "cmd": "ros2 run zenpool_draw_letter drawing_job_executor.py",
        "tags": "drawing job executor tcp strokes"
    },
    {
        "title": "Trigger Full Board Cleaning",
        "description": "Run the automated full squeegee zig-zag sequence.",
        "cmd": "ros2 run zenpool_draw_letter full_cleaning.py",
        "tags": "full board cleaning squeegee automated wipe"
    },
    {
        "title": "Start ArUco Servo Tracker",
        "description": "Start the MoveIt Servo script to visually follow the ArUco marker.",
        "cmd": "ros2 run zenpool_draw_letter aruco_tracker_servo.py",
        "tags": "aruco tracker servo visual follow"
    },
    {
        "title": "Launch RViz Visualization",
        "description": "View the robot's physical state, planned trajectories, and 3D collision models.",
        "cmd": "ros2 launch my_robot_cell_moveit_config moveit_rviz.launch.py rviz_config:=/home/ros/.zenpool.config.rviz",
        "tags": "rviz visualize 3d collision models trajectories ui"
    },
    {
        "title": "Start MoveIt Servo Engine",
        "description": "Start the real-time IK velocity solver.",
        "cmd": "ros2 launch my_robot_cell_moveit_config servo.launch.py",
        "tags": "servo moveit engine real-time ik solver"
    },
    {
        "title": "Start RViz Interactive Teleoperation",
        "description": "Drag a 3D marker in RViz to manually teleoperate the robot.",
        "cmd": "ros2 run zenpool_draw_letter zenpool_teleop_servo",
        "tags": "rviz interactive teleop marker manual drag"
    },
    {
        "title": "Start AI Face Tracking",
        "description": "Automatically track a face using the robot camera.",
        "cmd": "ros2 run zenpool_draw_letter face_tracker_servo.py",
        "tags": "ai face tracking visual servoing camera"
    },
    {
        "title": "Host Machine: Enable Display Forwarding",
        "description": "Allow Docker to display GUI windows on the host machine.",
        "cmd": "xhost +",
        "tags": "xhost display forwarding docker gui host window"
    },
    {
        "title": "Start ZenPool Tmuxinator",
        "description": "Launch the underlying UR driver and ROS 2 controllers using tmux profile.",
        "cmd": "tmuxinator start zenpool",
        "tags": "tmux tmuxinator start zenpool driver launch"
    },
    {
        "title": "Get Cartesian Coordinates of End Effector",
        "description": "Continuously print the exact X,Y,Z translation and XYZ rotation of the tool tip relative to the base.",
        "cmd": "ros2 run tf2_ros tf2_echo ur5e_base_link tc_arm_side",
        "tags": "cartesian coordinate xyz position translation tf tf2_echo tf2 end effector tip"
    },
]

def main(stdscr):
    curses.curs_set(1)  # Show cursor for text input
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_CYAN, -1)
    curses.init_pair(2, curses.COLOR_BLACK, curses.COLOR_WHITE) # Highlight
    curses.init_pair(3, curses.COLOR_YELLOW, -1)
    curses.init_pair(4, curses.COLOR_GREEN, -1)

    search_query = ""
    selected_idx = 0
    filtered_cmds = COMMANDS[:]

    while True:
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        # Filter commands
        q = search_query.lower()
        if q:
            filtered_cmds = []
            for c in COMMANDS:
                if q in c['title'].lower() or q in c['description'].lower() or q in c['tags'].lower() or q in c['cmd'].lower():
                    filtered_cmds.append(c)
        else:
            filtered_cmds = COMMANDS[:]

        # Fix index bounds
        if selected_idx >= len(filtered_cmds):
            selected_idx = max(0, len(filtered_cmds) - 1)

        # Draw Header
        title_str = " === ZENPOOL COMMAND SEARCH === "
        stdscr.addstr(0, max(0, (w - len(title_str)) // 2), title_str, curses.color_pair(1) | curses.A_BOLD)
        
        # Draw Search Bar
        stdscr.addstr(2, 2, "Search: ", curses.A_BOLD)
        stdscr.addstr(2, 10, search_query)
        
        stdscr.addstr(3, 2, "-" * (w - 4))

        # Draw List
        max_items = h - 8
        if max_items < 1: max_items = 1
        
        start_idx = max(0, selected_idx - max_items // 2)
        if start_idx + max_items > len(filtered_cmds):
            start_idx = max(0, len(filtered_cmds) - max_items)
            
        for i in range(max_items):
            idx = start_idx + i
            if idx >= len(filtered_cmds):
                break
                
            cmd_obj = filtered_cmds[idx]
            y = 4 + i
            
            is_selected = (idx == selected_idx)
            attr = curses.color_pair(2) if is_selected else curses.A_NORMAL
            
            # Truncate text to fit
            disp_title = cmd_obj['title'][:max(0, w - 6)]
            stdscr.attron(attr)
            stdscr.addstr(y, 2, f" {'*' if is_selected else ' '} {disp_title} ".ljust(w - 4))
            stdscr.attroff(attr)
            
            # If selected, show details at the bottom
            if is_selected:
                desc = cmd_obj['description'][:max(0, w - 4)]
                cmd_txt = cmd_obj['cmd'][:max(0, w - 4)]
                try:
                    stdscr.addstr(h - 4, 2, "DESCRIPTION:", curses.color_pair(1) | curses.A_BOLD)
                    stdscr.addstr(h - 3, 2, desc, curses.color_pair(3))
                    stdscr.addstr(h - 2, 2, cmd_txt, curses.color_pair(4) | curses.A_BOLD)
                except curses.error:
                    pass

        # Move cursor to end of search query
        try:
            stdscr.move(2, 10 + len(search_query))
        except curses.error:
            pass
            
        stdscr.refresh()

        # Input handling
        try:
            c = stdscr.getch()
        except KeyboardInterrupt:
            return None
            
        if c == 27: # ESC
            return None
        elif c == curses.KEY_UP:
            if selected_idx > 0: selected_idx -= 1
        elif c == curses.KEY_DOWN:
            if selected_idx < len(filtered_cmds) - 1: selected_idx += 1
        elif c in (curses.KEY_BACKSPACE, 127, 8):
            search_query = search_query[:-1]
        elif c in (curses.KEY_ENTER, 10, 13):
            if filtered_cmds:
                return filtered_cmds[selected_idx]['cmd']
        elif c >= 32 and c <= 126:
            search_query += chr(c)

if __name__ == "__main__":
    while True:
        result = curses.wrapper(main)
        if result:
            os.system('clear')
            print("\n\n" + "="*80)
            print("  COMMAND SELECTED")
            print("="*80 + "\n")
            print(f"  {result}\n")
            print("="*80)
            print("  Highlight the text above to copy it, then paste it into any terminal!")
            print("  (Press Ctrl+C to close, or press Enter to return to search)\n\n")
            try:
                input()
            except (KeyboardInterrupt, EOFError):
                break
        else:
            break
