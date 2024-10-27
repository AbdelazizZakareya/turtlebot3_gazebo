import subprocess
import time
import psutil  # To check if Gazebo is running

# Function to run the shell script for Gazebo
def run_gazebo():
    print("Starting Gazebo...")
    subprocess.Popen(["./scripts/gazrun.sh"], shell=True)  # Start Gazebo as a background process  
    # Wait until Gazebo (gzserver) is running
    wait_for_gazebo()

# Function to wait until Gazebo is fully running
def wait_for_gazebo():
    print("Waiting for Gazebo to start...")
    while True:
        # Check if gzserver process is running using psutil
        if any("gzserver" in p.name() for p in psutil.process_iter()):
            print("Gazebo is running.")
            break
        time.sleep(1)  # Check every second until Gazebo is running

# Function to run the move command
def move(value):
    print(f"Executing Move command with distance {value}")
    subprocess.run(["./scripts/move.sh", str(value)])

# Function to run the rotate command
def rotate(value):
    print(f"Executing Rotate command with angle {value}")
    subprocess.run(["./scripts/rotate.sh", str(value)])

# Function to interpret and execute the plan commands
def execute_plan(plan):
    invalid_commands = []  # To collect any invalid commands
    
    # Loop through each command in the plan
    for command, value in plan:
        if command.lower() == "move":
            move(value)
        elif command.lower() == "rotate":
            rotate(value)
        else:
            # Collect any commands that are not recognized
            invalid_commands.append(command)

    # Print or log invalid commands
    if invalid_commands:
        print("\nInvalid commands encountered:")
        for cmd in invalid_commands:
            print(f" - {cmd}")
    else:
        print("\nNo invalid commands encountered.")

if __name__ == "__main__":
    # Sample plan list containing commands
    plan = [["MOVE", 0.6], ["ROTATE", 90], ["MOVE", 0.6], ["ROTATE", 90], ["MOVE", 0.6], ["ROTATE", 90], ["MOVE", 0.6], ["ROTATE", 90]]

    # Start Gazebo simulation
    run_gazebo()

    # Execute the plan sequentially
    execute_plan(plan)
