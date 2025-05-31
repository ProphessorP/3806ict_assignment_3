#!/usr/bin/env python3
import os
import rospy
from ollama import Client

PROMPT_PLACEHOLDER = "You are a planner agent in a robotics AI system with the goal of rescuing people and bringing them back to base. " \
    "Your job is to, given sensor readings and some state, choose one of 3 options: go back to base (0), rescue person (1), or explore (2)." \
    "You can only hold a maximum of 2 people onboard at a time. Ensure your strategy doesn't cause the robot to go back and forth between goals. " \
    "You can ONLY output one integer: 0, 1, or 2 corresponding to your choice. No other text must be present in your response." \
    "Please choose the plan for the following state: "
home_dir = os.environ["HOME"]
STATE_FILE = f"{home_dir}/catkin_ws/src/3806ict_assignment_3/pat/state.txt"
TASK_FILE = f"{home_dir}/catkin_ws/src/3806ict_assignment_3/pat/task.txt"
current_strategy = "2"

def read_state():
    with open(STATE_FILE, "r") as f:
        content = f.read()
    parts = content.split("; ")
    grid = parts[0].split(": ")[1].strip("[]").split(", ")
    x = int(parts[1].split(": ")[1])
    y = int(parts[2].split(": ")[1])
    saved = int(parts[3].split(": ")[1])
    onboard = int(parts[4].split(": ")[1])
    return grid, x, y, saved, onboard

def query_ollama(client, grid, x, y, saved, onboard):
    prompt = PROMPT_PLACEHOLDER + f"Grid: {grid}, robot at ({x}, {y}), survivors saved: {saved}, people onboard: {onboard}."
    response = client.generate(model="llama2", prompt=prompt)
    task = response["response"].strip().lower()
    if task not in ["0", "1", "2"]:
        rospy.logwarn(f"Invalid LLM response: {task}")
        task = "0" if onboard > 0 else "2"  # State-based fallback
    return task

def write_task(task):
    global current_strategy
    if current_strategy == task:
        return
    with open(TASK_FILE, "w") as f:
        f.write(task)
    current_strategy = task

def planner_node():
    client = Client(host="http://localhost:11434")
    client.generate(model="llama2", prompt="Hello")  # Warm up
    rospy.init_node("planner_node")
    rospy.loginfo("Started planner node")
    rate = rospy.Rate(3)  # Same as Gazebo simulation rate to ensure sync
    while not rospy.is_shutdown():
        rospy.loginfo("Querying LLM for strategy")
        grid, x, y, saved, onboard = read_state()
        task = query_ollama(client, grid, x, y, saved, onboard)
        write_task(task)
        rospy.loginfo(f"Planner decided: {task}")
        rate.sleep()

if __name__ == "__main__":
    try:
        planner_node()
        rospy.loginfo("Finished planner node")
    except rospy.ROSInterruptException:
        pass