import subprocess
import time

def execute_commands_in_terminal(commands, terminal_name):
    try:
        command_string = " && ".join(commands)
        if terminal_name.lower() == "linux" or terminal_name.lower() == "mac":
            terminal_command = ["gnome-terminal", "--", "bash", "-c", command_string]
        elif terminal_name.lower() == "windows":
            print("Windows OS is not supported for this operation.")
            return
        else:
            print("Unsupported operating system")
            return

        subprocess.Popen(terminal_command)
        print("Commands have been started in a separate terminal.")
    except Exception as e:
        print("Error:", e)

if __name__ == "__main__":
    commands1 = [
        "source ~/AirSim/ros/devel/setup.bash",
        "roscd airsim_tutorial_pkgs",
        "cp settings/front_stereo_and_center_mono.json ~/Documents/AirSim/settings.json",
        "roslaunch airsim_ros_pkgs airsim_node.launch"
    ]

    commands2 = [
        "source ~/AirSim/ros/devel/setup.bash",
        "roslaunch airsim_tutorial_pkgs rviz.launch"
    ]

    execute_commands_in_terminal(commands1, "linux")
    
    time.sleep(2)

    execute_commands_in_terminal(commands2, "linux")

