#include <ros/ros.h>
#include "assignment_3/UpdateGrid.h"
#include "assignment_3/Sensor.h"
#include "communal_defines.cpp"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <vector>
#include <random>
#include <fstream>
#include <queue>
#include <string>
#include <utility>
#include <sstream>

#define GAZEBO_SIMULATION_RATE 3

#define SubIsHome(sub_x, sub_y) (sub_x == SUB_START_X && sub_y == SUB_START_Y)

#define SURVEY_AREA 0
#define COLLECT_SURVIVORS 1
#define GO_HOME 2

std::string homeDir = getenv("HOME");
#define PAT_EXE_DIR homeDir + "/Desktop/MONO-PAT-v3.6.0/PAT3.Console.exe"
#define PAT_PATH_CSP_EXPLORE_DIR homeDir + "/catkin_ws/src/3806ict_assignment_3/pat/explore.csp"
#define PAT_PATH_CSP_HOME_DIR homeDir + "/catkin_ws/src/3806ict_assignment_3/pat/return_home.csp"
#define PAT_PATH_CSP_COLLECT_SURVIVORS_DIR homeDir + "/catkin_ws/src/3806ict_assignment_3/pat/collect_survivors.csp"
#define PAT_OUTPUT_DIR homeDir + "/catkin_ws/src/3806ict_assignment_3/pat/pat_output.txt"
#define PAT_WORLD_DIR homeDir + "/catkin_ws/src/3806ict_assignment_3/pat/world.csp"
#define MAX_BFS_TIME 10
#define STATE_FILE homeDir + "/catkin_ws/src/3806ict_assignment_3/pat/state.txt"
#define TASK_FILE homeDir + "/catkin_ws/src/3806ict_assignment_3/pat/task.txt"
#define PAT_VALIDATION_CSP_DIR homeDir + "/catkin_ws/src/3806ict_assignment_3/pat/validation.csp"
#define OUTPUT_FILE homeDir + "/catkin_ws/src/3806ict_assignment_3/pat/rl_moves.txt"

std::string PAT_CMD_EXPLORE = "mono " + PAT_EXE_DIR + " " + PAT_PATH_CSP_EXPLORE_DIR + " " + PAT_OUTPUT_DIR;
std::string PAT_CMD_GO_HOME_BFS = "timeout " + std::to_string(MAX_BFS_TIME) + "s mono " + PAT_EXE_DIR + " -engine 1 " + PAT_PATH_CSP_HOME_DIR + " " + PAT_OUTPUT_DIR;
std::string PAT_CMD_GO_HOME_DFS = "mono " + PAT_EXE_DIR + " -engine 1 " + PAT_PATH_CSP_HOME_DIR + " " + PAT_OUTPUT_DIR;
std::string PAT_CMD_COLLECT_SURVIVORS_BFS = "timeout " + std::to_string(MAX_BFS_TIME) + "s mono " + PAT_EXE_DIR + " -engine 1 " + PAT_PATH_CSP_COLLECT_SURVIVORS_DIR + " " + PAT_OUTPUT_DIR;
std::string PAT_CMD_COLLECT_SURVIVORS_DFS = "mono " + PAT_EXE_DIR + " " + PAT_PATH_CSP_COLLECT_SURVIVORS_DIR + " " + PAT_OUTPUT_DIR;

void detect_hostiles(assignment_3::Sensor &hostile_srv, int (&curr_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y);
int detect_survivors(assignment_3::Sensor &survivor_srv, int (&curr_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y);
std::pair<int, int> update_position(std::string &move, int &x, int &y);
void read_moves_from_file(const std::string& filename, std::queue<std::string>& q);
void generate_world(int (&world)[BOARD_H][BOARD_W]);
void generate_known_world(int (&world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, int &onBoard);
std::vector<int> translate_world(int (&world)[BOARD_H][BOARD_W]);
void regenerate_moves(int (&current_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, int &onBoard, std::queue<std::string> &q, int currentPath);
void execute_move(int (&current_world)[BOARD_H][BOARD_W], int (&true_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, std::pair<int, int> &new_coords);
void write_state(int (&curr_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, int &survivors_saved, int &onBoard);
std_msgs::Int32MultiArray createGrid(int (&true_world)[BOARD_H][BOARD_W]);
int count_known_survivors(int (&world)[BOARD_H][BOARD_W]);
int read_task();
void generate_csp(int saved, int onboard, int known_survivors, int unknown_survivors, int task);
bool is_task_valid();
void generate_validation_csp(const std::vector<std::string>& moves, int task);
bool is_path_valid();

// Added function to check for remaining survivors
bool has_known_survivors(int (&world)[BOARD_H][BOARD_W]) {
    for (int i = 0; i < BOARD_H; i++) {
        for (int j = 0; j < BOARD_W; j++) {
            if (world[i][j] == SURVIVOR) {
                return true;
            }
        }
    }
    return false;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "testing");
    ros::NodeHandle n;
    ros::ServiceClient gridClient = n.serviceClient<assignment_3::UpdateGrid>("/update_grid");
    ros::ServiceClient hostileSensorClient = n.serviceClient<assignment_3::Sensor>("/hostile_sensor");
    ros::ServiceClient survivorSensorClient = n.serviceClient<assignment_3::Sensor>("/survivor_sensor");
    assignment_3::UpdateGrid grid_srv;
    assignment_3::Sensor hostile_srv;
    assignment_3::Sensor survivor_srv;
    std_msgs::Int32MultiArray true_grid;

    int survivors_saved = 0;
    int survivors_seen = 0;
    int OnBoard = 0;
    int true_world[BOARD_H][BOARD_W];
    int current_world[BOARD_H][BOARD_W];
    for (int i = 0; i < BOARD_H; i++)
        for (int j = 0; j < BOARD_W; j++)
            current_world[i][j] = EMPTY;
    current_world[SUB_START_X][SUB_START_Y] = VISITED;
    int sub_x = SUB_START_X;
    int sub_y = SUB_START_Y;
    int currentPath = SURVEY_AREA;

    generate_world(true_world);
    true_grid = createGrid(true_world);
    grid_srv.request.grid = true_grid;
    if (!gridClient.call(grid_srv))
    {
        ROS_ERROR("Failed to call update_grid service");
        return EXIT_FAILURE;
    }

    std::queue<std::string> q;
    hostile_srv.request.sensorRange = HOSTILE_DETECTION_RANGE;
    survivor_srv.request.sensorRange = SURVIVOR_DETECTION_RANGE;
    if (!hostileSensorClient.call(hostile_srv) || !survivorSensorClient.call(survivor_srv))
    {
        ROS_ERROR("Failed to call sensor services");
        return EXIT_FAILURE;
    }

    detect_hostiles(hostile_srv, current_world, sub_x, sub_y);
    int newSurvivorsDetected = detect_survivors(survivor_srv, current_world, sub_x, sub_y);
    if (newSurvivorsDetected)
    {
        ROS_INFO("New survivor(s) detected!");
        survivors_seen += newSurvivorsDetected;
        currentPath = COLLECT_SURVIVORS;
    }

    regenerate_moves(current_world, sub_x, sub_y, OnBoard, q, currentPath);
    std::string next_move;
    ros::Rate rate(GAZEBO_SIMULATION_RATE);

    while (ros::ok())
    {
        ROS_INFO("-- Start of move cycle --");

        if (SubIsHome(sub_x, sub_y) && OnBoard)
        {
            survivors_saved += OnBoard;
            std::cout << "Saved " << OnBoard << " survivors. Total survivors now saved: " << survivors_saved << std::endl;
            OnBoard = 0;
        }

        if (q.empty())
        {
            if ((survivors_saved + OnBoard) == SURVIVOR_COUNT)
            {
                if (SubIsHome(sub_x, sub_y))
                {
                    ROS_INFO("Mission successful!\nFinal internal representation of environment:");
                    for (int i = 0; i < BOARD_H; i++)
                    {
                        for (int j = 0; j < BOARD_W; j++)
                        {
                            if (current_world[i][j] != VISITED)
                                std::cout << " ";
                            std::cout << current_world[i][j] << " ";
                        }
                        std::cout << std::endl;
                    }
                    return EXIT_SUCCESS;
                }
                else
                    currentPath = GO_HOME;
            }
            else
            {
                int known_survivors = count_known_survivors(current_world);
                int unknown_survivors = SURVIVOR_COUNT - survivors_saved - OnBoard - known_survivors;
                int task = read_task();
                generate_csp(survivors_saved, OnBoard, known_survivors, unknown_survivors, task);
                std::string pat_cmd = "mono " + PAT_EXE_DIR + " " + PAT_VALIDATION_CSP_DIR + " " + PAT_OUTPUT_DIR;
                std::system(pat_cmd.c_str());
                if (task == -1 || !is_task_valid())
                {
                    currentPath = (OnBoard > 0) ? GO_HOME : SURVEY_AREA;
                }
                else
                {
                    currentPath = task;
                }
            }
            regenerate_moves(current_world, sub_x, sub_y, OnBoard, q, currentPath);
        }

        // Modified move retrieval to prevent segmentation fault
        if (!q.empty()) {
            next_move = q.front();
            q.pop();
            ROS_INFO("Next move is: %s", next_move.c_str());
        } else {
            ROS_WARN("Move queue is empty, regenerating moves");
            regenerate_moves(current_world, sub_x, sub_y, OnBoard, q, currentPath);
            continue; // Skip to next iteration to avoid invalid move
        }

        std::pair<int, int> new_coords = update_position(next_move, sub_x, sub_y);
        int new_x = new_coords.first;
        int new_y = new_coords.second;

        if (current_world[new_x][new_y] == HOSTILE)
        {
            ROS_INFO("About to move into hostile, recalculating directions");
            regenerate_moves(current_world, sub_x, sub_y, OnBoard, q, currentPath);
            rate.sleep();
            continue;
        }

        if (current_world[new_x][new_y] == SURVIVOR)
        {
            ROS_INFO("About to pick up a survivor :) Hooray!");
            OnBoard++;
            std::cout << "Now have " << OnBoard << " survivors onboard" << std::endl;
            // Modified task-switching logic to continue searching for survivors
            if (OnBoard >= SUB_CAP || !has_known_survivors(current_world)) {
                currentPath = GO_HOME;
            } else {
                currentPath = COLLECT_SURVIVORS;
                ROS_INFO("Continuing to search for more survivors");
            }
        }

        execute_move(current_world, true_world, sub_x, sub_y, new_coords);
        true_grid.data = translate_world(true_world);
        grid_srv.request.grid = true_grid;
        if (!gridClient.call(grid_srv))
        {
            ROS_ERROR("Failed to call update_grid service");
            return EXIT_FAILURE;
        }

        sub_x = new_x;
        sub_y = new_y;

        if (!hostileSensorClient.call(hostile_srv) || !survivorSensorClient.call(survivor_srv))
        {
            ROS_ERROR("Failed to call sensor services");
            return EXIT_FAILURE;
        }

        detect_hostiles(hostile_srv, current_world, sub_x, sub_y);
        newSurvivorsDetected = detect_survivors(survivor_srv, current_world, sub_x, sub_y);
        if (newSurvivorsDetected)
        {
            ROS_INFO("New survivor(s) detected!");
            survivors_seen += newSurvivorsDetected;
            currentPath = COLLECT_SURVIVORS;
            regenerate_moves(current_world, sub_x, sub_y, OnBoard, q, currentPath);
        }

        write_state(current_world, sub_x, sub_y, survivors_saved, OnBoard);
        ROS_INFO("-- End of move cycle --\n");
        rate.sleep();
        ros::spinOnce();
    }
}

void detect_hostiles(assignment_3::Sensor &hostile_srv, int (&curr_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y)
{
    if (hostile_srv.response.objectEast)
        for (int i = 0; i < hostile_srv.request.sensorRange; i++)
            if (hostile_srv.response.eastRadar[i])
            {
                curr_world[sub_x][sub_y + 1 + i] = HOSTILE;
                ROS_INFO("Robot has detected a hostile east!");
            }
    if (hostile_srv.response.objectWest)
        for (int i = 0; i < hostile_srv.request.sensorRange; i++)
            if (hostile_srv.response.westRadar[i])
            {
                curr_world[sub_x][sub_y - 1 - i] = HOSTILE;
                ROS_INFO("Robot has detected a hostile west!");
            }
    if (hostile_srv.response.objectNorth)
        for (int i = 0; i < hostile_srv.request.sensorRange; i++)
            if (hostile_srv.response.northRadar[i])
            {
                curr_world[sub_x - 1 - i][sub_y] = HOSTILE;
                ROS_INFO("Robot has detected a hostile north!");
            }
    if (hostile_srv.response.objectSouth)
        for (int i = 0; i < hostile_srv.request.sensorRange; i++)
            if (hostile_srv.response.southRadar[i])
            {
                curr_world[sub_x + 1 + i][sub_y] = HOSTILE;
                ROS_INFO("Robot has detected a hostile south!");
            }
}

int detect_survivors(assignment_3::Sensor &survivor_srv, int (&curr_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y)
{
    int newSurvivorsDetected = 0;
    if (survivor_srv.response.objectEast)
        for (int i = 0; i < survivor_srv.request.sensorRange; i++)
            if (survivor_srv.response.eastRadar[i])
                if (curr_world[sub_x][sub_y + 1 + i] != SURVIVOR)
                {
                    newSurvivorsDetected++;
                    curr_world[sub_x][sub_y + 1 + i] = SURVIVOR;
                    ROS_INFO("Robot has detected a survivor east!");
                }
    if (survivor_srv.response.objectWest)
        for (int i = 0; i < survivor_srv.request.sensorRange; i++)
            if (survivor_srv.response.westRadar[i])
                if (curr_world[sub_x][sub_y - 1 - i] != SURVIVOR)
                {
                    newSurvivorsDetected++;
                    curr_world[sub_x][sub_y - 1 - i] = SURVIVOR;
                    ROS_INFO("Robot has detected a survivor west!");
                }
    if (survivor_srv.response.objectNorth)
        for (int i = 0; i < survivor_srv.request.sensorRange; i++)
            if (survivor_srv.response.northRadar[i])
                if (curr_world[sub_x - 1 - i][sub_y] != SURVIVOR)
                {
                    newSurvivorsDetected++;
                    curr_world[sub_x - 1 - i][sub_y] = SURVIVOR;
                    ROS_INFO("Robot has detected a survivor north!");
                }
    if (survivor_srv.response.objectSouth)
        for (int i = 0; i < survivor_srv.request.sensorRange; i++)
            if (survivor_srv.response.southRadar[i])
                if (curr_world[sub_x + 1 + i][sub_y] != SURVIVOR)
                {
                    newSurvivorsDetected++;
                    curr_world[sub_x + 1 + i][sub_y] = SURVIVOR;
                    ROS_INFO("Robot has detected a survivor south!");
                }
    return newSurvivorsDetected;
}

std::pair<int, int> update_position(std::string &move, int &x, int &y)
{
    if (move == "moveRight")
        return {x, y + 1};
    else if (move == "moveLeft")
        return {x, y - 1};
    else if (move == "moveUp")
        return {x - 1, y};
    else if (move == "moveDown")
        return {x + 1, y};
    std::cerr << "update_position found invalid move: " << move << std::endl;
    return {x, y};
}

void read_moves_from_file(const std::string& filename, std::queue<std::string>& q)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open moves file: " << filename << std::endl;
        exit(1);
    }
    while (!q.empty()) q.pop();
    std::string line;
    if (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string token;
        while (iss >> token)
        {
            if (token == "->" || token == "<init>") continue;
            if (token.find('>') != std::string::npos)
                token = token.substr(0, token.find('>'));
            if (!token.empty() && (token == "moveUp" || token == "moveDown" || token == "moveLeft" || token == "moveRight"))
                q.push(token);
        }
    }
    file.close();
}

void generate_world(int (&world)[BOARD_H][BOARD_W])
{
    for (int i = 0; i < BOARD_H; ++i)
        for (int j = 0; j < BOARD_W; ++j)
            world[i][j] = EMPTY;
    world[SUB_START_X][SUB_START_Y] = SUB;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> rowDist(0, BOARD_H - 1);
    std::uniform_int_distribution<int> colDist(0, BOARD_W - 1);
    int placed = 0;
    while (placed < SURVIVOR_COUNT)
    {
        int rand_row = rowDist(gen);
        int rand_col = colDist(gen);
        if (world[rand_row][rand_col] == EMPTY)
        {
            world[rand_row][rand_col] = SURVIVOR;
            placed++;
        }
    }
    placed = 0;
    while (placed < HOSTILE_COUNT)
    {
        int rand_row = rowDist(gen);
        int rand_col = colDist(gen);
        if (world[rand_row][rand_col] == EMPTY)
        {
            world[rand_row][rand_col] = HOSTILE;
            placed++;
        }
    }
}

void generate_known_world(int (&world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, int &onBoard)
{
    std::ofstream file(PAT_WORLD_DIR);
    if (!file.is_open())
    {
        std::cerr << "Failed to save the current world to world.csp" << std::endl;
        exit(1);
    }
    ROS_INFO("Writing robot's current interpretation to world file.");
    file << "#define Visited " << VISITED << ";\n";
    file << "#define Unvisited " << EMPTY << ";\n";
    file << "#define Sub " << SUB << ";\n";
    file << "#define Hostile " << HOSTILE << ";\n";
    file << "#define Survivor " << SURVIVOR << ";\n\n";
    file << "#define SUB_HOME_X " << SUB_START_X << ";\n";
    file << "#define SUB_HOME_Y " << SUB_START_Y << ";\n";
    file << "#define Rows " << BOARD_H << ";\n";
    file << "#define Cols " << BOARD_W << ";\n";
    file << "#define maxCapacity " << SUB_CAP << ";\n";
    file << "\nvar world[Rows][Cols]:{Visited..Survivor} = [\n";
    for (int i = 0; i < BOARD_H; i++)
    {
        for (int j = 0; j < BOARD_W; j++)
        {
            if (i == BOARD_H - 1 && j == BOARD_W - 1)
                file << world[i][j];
            else
                file << world[i][j] << ", ";
        }
        file << "\n";
    }
    file << "];\n\n";
    file << "var xpos:{0..Rows-1} = " << sub_x << ";\n";
    file << "var ypos:{0..Cols-1} = " << sub_y << ";\n";
    file << "var onBoard:{0..maxCapacity} = " << onBoard << ";\n";
    file.close();
}

std::vector<int> translate_world(int (&world)[BOARD_H][BOARD_W])
{
    std::vector<int> vec(BOARD_W * BOARD_H, EMPTY);
    for (int i = 0; i < BOARD_H; i++)
        for (int j = 0; j < BOARD_W; j++)
            vec[i * BOARD_W + j] = world[i][j];
    return vec;
}

void regenerate_moves(int (&current_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, int &onBoard, std::queue<std::string> &q, int currentPath)
{
    generate_known_world(current_world, sub_x, sub_y, onBoard);
    std::string rl_cmd = "python3 " + homeDir + "/catkin_ws/src/3806ict_assignment_3/src/rl_executor.py " + std::to_string(currentPath);
    ROS_INFO("Calling RL agent to generate moves for task %d", currentPath);
    std::system(rl_cmd.c_str());
    std::vector<std::string> rl_moves;
    std::ifstream rl_moves_file(OUTPUT_FILE);
    if (rl_moves_file.is_open())
    {
        std::string line;
        if (std::getline(rl_moves_file, line))
        {
            std::istringstream iss(line);
            std::string token;
            while (iss >> token)
            {
                if (token == "->" || token == "<init>") continue;
                if (token.find('>') != std::string::npos)
                    token = token.substr(0, token.find('>'));
                if (!token.empty() && (token == "moveUp" || token == "moveDown" || token == "moveLeft" || token == "moveRight"))
                    rl_moves.push_back(token);
            }
        }
        rl_moves_file.close();
    }
    else
    {
        ROS_ERROR("Failed to read RL moves file");
    }

    if (rl_moves.size() > 0)
    {
        generate_validation_csp(rl_moves, currentPath);
        std::string pat_cmd = "mono " + PAT_EXE_DIR + " " + PAT_VALIDATION_CSP_DIR + " " + PAT_OUTPUT_DIR;
        std::system(pat_cmd.c_str());

        if (is_path_valid())
        {
            ROS_INFO("RL path validated by PAT, using RL moves");
            for (const auto& move : rl_moves)
                q.push(move);
            return;
        }
    }

    ROS_INFO("RL path invalid or empty, falling back to PAT path planning");
    if (currentPath == SURVEY_AREA)
    {
        ROS_INFO("Calculating a path to survey remaining area with PAT");
        std::system(PAT_CMD_EXPLORE.c_str());
    }
    else if (currentPath == COLLECT_SURVIVORS)
    {
        ROS_INFO("Calculating a path to collect survivors with PAT");
        int status = std::system(PAT_CMD_COLLECT_SURVIVORS_BFS.c_str());
        if (status < 0)
        {
            std::cout << "Fatal error: " << strerror(errno) << '\n';
            exit(1);
        }
        else if (WIFEXITED(status) && WEXITSTATUS(status) == 124)
        {
            ROS_INFO("BFS took too long, switching to DFS");
            std::system(PAT_CMD_COLLECT_SURVIVORS_DFS.c_str());
        }
    }
    else if (currentPath == GO_HOME)
    {
        ROS_INFO("Calculating a path to go home with PAT");
        int status = std::system(PAT_CMD_GO_HOME_BFS.c_str());
        if (status < 0)
        {
            std::cout << "Fatal error: " << strerror(errno) << '\n';
            exit(1);
        }
        else if (WIFEXITED(status) && WEXITSTATUS(status) == 124)
        {
            ROS_INFO("BFS took too long, switching to DFS");
            std::system(PAT_CMD_GO_HOME_DFS.c_str());
        }
    }
    read_moves_from_file(PAT_OUTPUT_DIR, q);
}

void execute_move(int (&current_world)[BOARD_H][BOARD_W], int (&true_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, std::pair<int, int> &new_coords)
{
    int new_x = new_coords.first;
    int new_y = new_coords.second;
    current_world[new_x][new_y] = VISITED;
    true_world[sub_x][sub_y] = VISITED;
    true_world[new_x][new_y] = SUB;
}

std_msgs::Int32MultiArray createGrid(int (&true_world)[BOARD_H][BOARD_W])
{
    std_msgs::Int32MultiArray true_grid;
    true_grid.layout.dim.push_back(std_msgs::MultiArrayDimension());
    true_grid.layout.dim.push_back(std_msgs::MultiArrayDimension());
    true_grid.layout.dim[0].label = "height";
    true_grid.layout.dim[1].label = "width";
    true_grid.layout.dim[0].size = BOARD_H;
    true_grid.layout.dim[1].size = BOARD_W;
    true_grid.layout.dim[0].stride = BOARD_H * BOARD_W;
    true_grid.layout.dim[1].stride = BOARD_W;
    true_grid.layout.data_offset = 0;
    true_grid.data = translate_world(true_world);
    return true_grid;
}

void write_state(int (&curr_world)[BOARD_H][BOARD_W], int &sub_x, int &sub_y, int &survivors_saved, int &onBoard)
{
    std::ofstream file(STATE_FILE);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to write state to file.");
        return;
    }
    file << "grid: [";
    for (int i = 0; i < BOARD_H; i++)
        for (int j = 0; j < BOARD_W; j++)
            file << curr_world[i][j] << (i == BOARD_H - 1 && j == BOARD_W - 1 ? "" : ", ");
    file << "]; x: " << sub_x << "; y: " << sub_y << "; saved: " << survivors_saved << "; onboard: " << onBoard;
    file.close();
}

int count_known_survivors(int (&world)[BOARD_H][BOARD_W])
{
    int count = 0;
    for (int i = 0; i < BOARD_H; i++)
        for (int j = 0; j < BOARD_W; j++)
            if (world[i][j] == SURVIVOR)
                count++;
    return count;
}

int read_task()
{
    std::ifstream file(TASK_FILE);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open task file");
        return -1;
    }
    std::string task_str;
    std::getline(file, task_str);
    file.close();
    if (task_str == "0") return GO_HOME;
    if (task_str == "1") return COLLECT_SURVIVORS;
    if (task_str == "2") return SURVEY_AREA;
    ROS_ERROR("Invalid task value in file");
    return -1;
}

void generate_csp(int saved, int onboard, int known_survivors, int unknown_survivors, int task)
{
    std::ofstream file(PAT_VALIDATION_CSP_DIR);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open validation CSP file for writing");
        return;
    }
    file << "#define SURVIVOR_COUNT 5;\n";
    file << "#define SUB_CAP 2;\n";
    file << "#define AllSurvivorsSaved (saved == SURVIVOR_COUNT);\n\n";
    file << "var saved:{0..SURVIVOR_COUNT} = " << saved << ";\n";
    file << "var onboard:{0..SUB_CAP} = " << onboard << ";\n";
    file << "var known_survivors:{0..SURVIVOR_COUNT} = " << known_survivors << ";\n";
    file << "var unknown_survivors:{0..SURVIVOR_COUNT} = " << unknown_survivors << ";\n\n";
    file << "Explore() = [unknown_survivors > 0] (\n";
    file << "    find {known_survivors = known_survivors + 1; unknown_survivors = unknown_survivors - 1;} -> Robot()\n";
    file << "    [] nofind -> Robot()\n";
    file << ") [] [unknown_survivors == 0] skip -> Robot();\n\n";
    file << "Rescue() = [known_survivors > 0 && onboard < SUB_CAP] pickup {known_survivors = known_survivors - 1; onboard = onboard + 1;} -> Robot();\n\n";
    file << "ReturnHome() = [onboard > 0] dropoff {saved = saved + onboard; onboard = 0;} -> Robot();\n\n";
    file << "Robot() = (\n";
    file << "    [saved < SURVIVOR_COUNT] (\n";
    file << "        [onboard > 0] ReturnHome()\n";
    file << "        [] [known_survivors > 0 && onboard < SUB_CAP] Rescue()\n";
    file << "        [] [unknown_survivors > 0] Explore()\n";
    file << "    )\n";
    file << ") [] [saved == SURVIVOR_COUNT] skip;\n\n";
    file << "System() = (\n";
    if (task == GO_HOME)
        file << "    ReturnHome() -> Robot()\n";
    else if (task == COLLECT_SURVIVORS)
        file << "    Rescue() -> Robot()\n";
    else if (task == SURVEY_AREA)
        file << "    Explore() -> Robot()\n";
    else
        file << "    Robot()\n";
    file << ");\n\n";
    file << "#assert System() reaches AllSurvivorsSaved;\n";
    file.close();
}

bool is_task_valid()
{
    std::ifstream file(PAT_OUTPUT_DIR);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open PAT output file");
        return false;
    }
    std::string line;
    while (std::getline(file, line))
    {
        if (line.find("The Assertion is VALID") != std::string::npos)
        {
            file.close();
            return true;
        }
    }
    file.close();
    return false;
}

void generate_validation_csp(const std::vector<std::string>& moves, int task)
{
    std::ofstream file(PAT_VALIDATION_CSP_DIR);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open validation CSP file");
        return;
    }
    file << "#include \"world.csp\"\n\n";
    file << "Sequence() = ";
    for (size_t i = 0; i < moves.size(); ++i)
    {
        file << moves[i];
        if (i < moves.size() - 1)
            file << " -> ";
    }
    file << " -> Skip;\n\n";
    std::string goal;
    if (task == GO_HOME)
        goal = "home";
    else if (task == COLLECT_SURVIVORS)
        goal = "noSurvivors";
    else if (task == SURVEY_AREA)
        goal = "goalAreaChecked";
    file << "#assert Sequence() reaches " << goal << ";\n";
    file.close();
}

bool is_path_valid()
{
    std::ifstream file(PAT_OUTPUT_DIR);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open PAT output file for path validation");
        return false;
    }
    std::string line;
    while (std::getline(file, line))
    {
        if (line.find("The Assertion is VALID") != std::string::npos)
        {
            file.close();
            return true;
        }
    }
    file.close();
    return false;
}