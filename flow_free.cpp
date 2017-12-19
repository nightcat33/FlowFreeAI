#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <stack>
#include <queue>
#include <vector>
#include <set>
#include <time.h>
#include <stdio.h>
#include <cstring>

using namespace std;

#define MAX_COLORS  14
#define COOR_MASK   0xFF
#define MAX_SIZE    14

// global variables below
typedef unsigned short uint16_t;

// index based on color sequence
uint16_t starts[MAX_COLORS];
uint16_t goals[MAX_COLORS];

map<int, char> color_map;
int board_size = 0;
int color_size = 0;
int board_height = 0;
// uint16_t active_color = 0;

// help functions for coordinates
uint16_t get_row(uint16_t pos){
    return (pos >> 8) & COOR_MASK;
}

uint16_t get_col(uint16_t pos){
    return pos & COOR_MASK;
}

// compress row and col into 8 bytes
uint16_t compress_pos(uint16_t row, uint16_t col){
    uint16_t pos = 0;
    pos |= ((row & COOR_MASK) << 8);
    pos |= (col & COOR_MASK);
    return pos;
}

// represent each state
class State {
public:
    // each board represents a color
    uint16_t board[MAX_SIZE][MAX_SIZE];
    // locations of current flow head
    uint16_t current_flow_locs[MAX_COLORS];

    // used to memorize last color choose
    uint16_t last_color;

    State(){
        memset(board, -1, sizeof(uint16_t)*MAX_SIZE*MAX_SIZE);
    }

    bool operator< (const State& other) const
    {
        if(last_color != other.last_color){
            return true;
        }

        for (int i = 0; i < color_size; i++)
        {
            if (current_flow_locs[i] != other.current_flow_locs[i]){
                return true;
            }
        }

        for (int i = 0; i < board_height; i++)
        {
            for(int j=0; j<board_size; j++){
                if(board[i][j] != other.board[i][j]){
                    return true;
                }
            }
        }
        return false;
    }
};

// node used for search
class Node {
public:
    State state;
    int g;
    int h;
    int f;
};

class NodeComparator
{
    public:
    bool operator() (const Node& n1, const Node& n2) const
    {
        return n1.f > n2.f;
    }
};

const int direction[4][2] = {
  {-1, 0}, //up
  {1, 0}, //down
  {0, -1}, //left
  {0, 1} //right
};

// return true if current position is inside board
bool boundaryCheck(uint16_t pos){
    uint16_t row = get_row(pos);
    uint16_t col = get_col(pos);
    if(row >= 0 && row < board_height && col >= 0 && col < board_size){
        return true;
    }else{
        return false;
    }
}

// return true if wall exits
bool wallCheck(uint16_t pos, State currState){
    uint16_t row = get_row(pos);
    uint16_t col = get_col(pos);

    uint16_t val = currState.board[row][col];

    // 65535 means empty space
    if(val == 65535){
        return false;
    }else{
        return true;
    }
}

// update position based on direction
// up down left right go from 0, 1, 2, 3
// return new position
uint16_t updatePosition(uint16_t pos, int dir){
    uint16_t row = get_row(pos);
    uint16_t col = get_col(pos);
    row += direction[dir][0];
    col += direction[dir][1];
    return compress_pos(row, col);
}

// return the color of specific location
uint16_t getColor(uint16_t pos, State currState){
    uint16_t row = get_row(pos);
    uint16_t col = get_col(pos);
    return currState.board[row][col];
}

// check for the first constraint
// after one move, the new position should have only one cell with the same color
// return true if zigzag found
bool checkZigzag(State currState, uint16_t pos, uint16_t currColor){

    uint16_t previous_pos = currState.current_flow_locs[currColor];

    for(int i=0; i<4; i++){
        uint16_t new_pos = updatePosition(pos, i);

        // inside board
        // neighbor cell has color
        // not predecessor
        // not goal
        // neighbor has the same color
        if(boundaryCheck(new_pos) && wallCheck(new_pos, currState) == true && new_pos != previous_pos && new_pos != goals[currColor] && getColor(new_pos, currState) == currColor){
            return true;
        }
    }
    return false;
}

// return true if detect, otherwise false
bool check_dead_end(State currState){
    uint16_t color = currState.last_color;

    // get the head of current flow
    uint16_t currFlowHeadPos = currState.current_flow_locs[color];

    for(int i=0; i<4; i++){
        uint16_t neighbor = updatePosition(currFlowHeadPos, i);
        // valid position and no color
        if(boundaryCheck(neighbor) && wallCheck(neighbor, currState) == false){
            int counts = 0;
            for(int j=0; j<4; j++){
                
                uint16_t new_neighbor = updatePosition(neighbor, j);
                // uint16_t row = get_row(new_neighbor);
                // uint16_t col = get_col(new_neighbor);
                // valid position
                if(boundaryCheck(new_neighbor)){ 
                    // no color
                    if(wallCheck(new_neighbor, currState) == false){
                        counts++;
                    }else{
                        // in-progress flow's head and goal
                        for(int k=0; k<color_size; k++){
                            if(currState.current_flow_locs[k] == goals[k]){
                                // such color flow reaches goal
                                continue;
                            }else{
                                if(new_neighbor == currState.current_flow_locs[k] || new_neighbor == goals[k]){
                                    counts++;
                                }
                            }
                        }
                    }
                }

            }

            if(counts <= 1){
                return true;
            }
        }
    }
    return false;
}

// return true if source has more than one cell with the same color
bool checkSource(State currState){
    for(int i=0; i<color_size; i++){
        int counts = 0;
        for(int j=0; j<4; j++){
            uint16_t pos = goals[i];
            uint16_t new_pos = updatePosition(pos, j);

            if(boundaryCheck(new_pos)){
                if(currState.board[get_row(new_pos)][get_col(new_pos)] == i){
                    counts++;
                }
            }
            if(counts > 1){
                return true;
            }
        }
    }
    return false;
}


// check if current move is valid
bool moveCheck(State currState, int dir, uint16_t currColor){
    uint16_t currFlow = currState.current_flow_locs[currColor];

    uint16_t new_pos = updatePosition(currFlow, dir);

    // board boundary check
    if(boundaryCheck(new_pos) == false){
        return false;
    }

    // new position has a color, and not goal
    if(wallCheck(new_pos, currState) == true && new_pos != goals[currColor]){
        return false;
    }

    if(checkZigzag(currState, new_pos, currColor)){
        return false;
    }

    return true;
}

// find the next color to move
// if last color flow not done yet, we just continue
// if done, choose the most constrained color
// update global variable active_color
uint16_t nextColor(State currState){

    // continue last color
    uint16_t last_color = currState.last_color;
    uint16_t currFlowHead = currState.current_flow_locs[last_color];
    // current flow not reache the goal
    if(currFlowHead != goals[last_color]){
        return last_color;
    }

    // get the most constrained color to move
    // initalize next color to last color
    uint16_t next_color = -1;
    int leastFree = 4; 

    for(int i=0; i<color_size; i++){
        uint16_t flowHeadOfThisColor = currState.current_flow_locs[i];

        // such color reaches its goal
        if(flowHeadOfThisColor == goals[i] || i == last_color){
            continue;
        }
        // get four neighbiors of current flow head
        // up, down, left, right
        int validNeighbors = 0;

        for(int j=0; j<4; j++){
            uint16_t new_pos = updatePosition(flowHeadOfThisColor, j);
            // inside board and not wall
            if(boundaryCheck(new_pos) == true && (wallCheck(new_pos, currState) == false || new_pos == goals[i])) {
                validNeighbors++;
            }
        }
        // cout << "color: " << color_map[i] << " neighbiors: " << validNeighbors << endl;
        if(validNeighbors < leastFree && validNeighbors > 0){
            leastFree = validNeighbors;
            next_color = i;
        }
    }

    return next_color;
}

// make movement and update state
void makeMove(State &currState, int dir, uint16_t color){
    uint16_t curr_pos = currState.current_flow_locs[color];
    uint16_t new_pos = updatePosition(curr_pos, dir);
    uint16_t new_x = get_row(new_pos);
    uint16_t new_y = get_col(new_pos);

    currState.board[new_x][new_y] = color;
    currState.current_flow_locs[color] = new_pos;
    currState.last_color = color;
}

bool checkDone(State currState){
    for(int i=0; i<color_size; i++){
        if(currState.current_flow_locs[i] != goals[i]){
            return false;
        }
    }
    // must all filled with colors
    for(int i=0; i<board_height; i++){
        for(int j=0; j<board_size; j++){
            if(currState.board[i][j] == 65535){
                return false;
            }
        }
    }
    return true;
}

void drawSolution(State endState){
    for(int i=0; i<board_height; i++){
        for(int j=0; j<board_size; j++){
            if(endState.board[i][j] != 65535){
                cout << color_map[endState.board[i][j]];
            }else{
                cout << "_";
            }

        }
        cout << endl;
    }
    cout << endl;
}

// difinition from wiki
// disjoint-set data structure
class Region{
public:

    Region(int size){
        this->size = size;
    }

    class region{
    public:
        uint16_t parent;
        uint16_t rank;

        region(uint16_t position){
            parent = position;
            rank = 0;
        }

        region(){}
    };

    void MakeSet(uint16_t x){
        
        uint16_t row = get_row(x);
        uint16_t col = get_col(x);
        if(v[row*size+col].parent == 65535){
            v[row*size+col].parent = x;
            v[row*size+col].rank = 0;
        }      
    }

    void addelements(int num){
        // initalize each region's parent to -1
        for(int i=0; i<num; i++){
            v.push_back(region(65535));
        }
    }

    //return the root of the up-tree in which the parameter element resides.
    uint16_t Find(uint16_t position){
        uint16_t row = get_row(position);
        uint16_t col = get_col(position);
        int idx = row*size + col;

        if(v[idx].parent != 65535 && v[idx].parent != position){
            v[idx].parent = Find(v[idx].parent);
        }
        return v[idx].parent;
    }

    void Union(uint16_t x, uint16_t y){
        uint16_t xRoot = Find(x);
        uint16_t yRoot = Find(y);

        if(xRoot == yRoot){
            return;
        }

        int xIdx = get_row(xRoot)*size + get_col(xRoot);
        int yIdx = get_row(yRoot)*size + get_col(yRoot);

        if(v[xIdx].rank < v[yIdx].rank){
            v[xIdx].parent = yRoot;
        }else if(v[xIdx].rank > v[yIdx].rank){
            v[yIdx].parent = xRoot;
        }else{
            v[yIdx].parent = xRoot;
            v[xIdx].rank += 1;
        }

    }

private:
    vector<region> v;
    int size;
};

int build_region(State currState, uint16_t remap[MAX_SIZE*MAX_SIZE]){
    // first pass
    Region r(board_size);
    r.addelements(board_size*board_height);

    for(int i=0; i<board_height; i++){
        for(int j=0; j<board_size; j++){
            uint16_t pos = compress_pos(i, j);
            // there is color, so not freespace
            if(wallCheck(pos, currState) == false){
                // set parent to pos
                r.MakeSet(pos);
                if(i){ 
                    uint16_t up = updatePosition(pos, 0);
                    // no color
                    if(wallCheck(up, currState) == false){
                        r.Union(pos, up);
                    }
                }

                if(j){
                    uint16_t left = updatePosition(pos, 2);
                    // no color
                    if(wallCheck(left, currState) == false){
                        r.Union(pos, left);
                    }
                }
            }
        }
    }

    uint16_t lookup[MAX_SIZE*MAX_SIZE];
    int counts = 0;

    memset(lookup, -1, sizeof(lookup));
    memset(remap, -1, MAX_SIZE*MAX_SIZE);

    // second pass
    // each region has unique identifier from 0
    for(int i=0; i<board_height; i++){
        for(int j=0; j<board_size; j++){
            uint16_t pos = compress_pos(i, j);
            uint16_t root = r.Find(pos);
            if(root != 65535){
                if(lookup[get_row(root)*board_size + get_col(root)] == 65535){
                    lookup[get_row(root)*board_size + get_col(root)] = counts++;
                }
                remap[i*board_size + j] = lookup[get_row(root)*board_size + get_col(root)];
            }else{
                remap[i*board_size + j] = 65535;
            }
        }
    }
    // cout << "# of regions: " << counts << endl;
    return counts;
}


bool stranded_color_region_check(State currState){
    
    uint16_t remap[MAX_SIZE*MAX_SIZE];

    int counts = build_region(currState, remap);

    if(counts == 0){
        return false;
    }

    uint16_t flowHeadRegionFlags[counts];
    uint16_t goalRegionFlags[counts];

    memset(flowHeadRegionFlags, 0, sizeof(flowHeadRegionFlags));
    memset(goalRegionFlags, 0, sizeof(goalRegionFlags));

    for(int color=0; color < color_size; color++){
        uint16_t currFlowHead = currState.current_flow_locs[color];

        if(currFlowHead == goals[color]){
            continue;
        }

        // flow head is approache goal
        int distance = abs(get_row(currFlowHead)-get_row(goals[color])) + abs(get_col(currFlowHead)-get_col(goals[color]));
        if(distance == 1){
            continue;
        }

        uint16_t color_flag = (1 << color);

        // current flow head flag
        for(int dir=0; dir<4; dir++){
            uint16_t new_pos = updatePosition(currFlowHead, dir);
            if(boundaryCheck(new_pos)){
                int neighbor_region = remap[get_row(new_pos)*board_size + get_col(new_pos)];
                if(neighbor_region != 65535){
                    flowHeadRegionFlags[neighbor_region] |= color_flag;
                }
            }
        }

        // goal flag
        for(int dir=0; dir<4; dir++){
            uint16_t new_pos = updatePosition(goals[color], dir);
            if(boundaryCheck(new_pos)){
                int neighbor_region = remap[get_row(new_pos)*board_size + get_col(new_pos)];
                if(neighbor_region != 65535){
                    goalRegionFlags[neighbor_region] |= color_flag;
                }
            }
        }

        int r = 0;
        for(; r<counts; r++){
            // such region touches both current flow's head and goal
            if((flowHeadRegionFlags[r] & color_flag) > 0 && (goalRegionFlags[r] & color_flag) > 0){
                break;
            }
        }

        if(r == counts){
            // cout << "Find stranded color" << endl;
            // cout << "Color: " << color_map[color] << endl; 
            // drawSolution(currState);
            // cout << "remap:" << endl;
            // for(int i=0; i<board_size; i++){
            //     for(int j=0; j<board_size; j++){
            //         cout << remap[i*board_size+j] << " ";
            //     }
            //     cout << endl;
            // }
            // cout << endl;
            // cout << "current flow:" << endl;
            // for(int i=0; i<counts; i++){
            //     cout << flowHeadRegionFlags[i] << " ";
            // }
            // cout << endl;
            // cout << "goal flow:" << endl;
            // for(int i=0; i<counts; i++){
            //     cout << goalRegionFlags[i] << " ";
            // }
            // cout << endl;

            // cout << (flowHeadRegionFlags[0] & (1 << 0)) << endl;
            return true;
        }

    }

    for(int r=0; r<counts; r++){
        if(!(flowHeadRegionFlags[r] & goalRegionFlags[r])){
            // cout << "Find stranded region" << endl;
            return true;
        }
    }
    return false;
}


bool stranded_color(State currState, int maxStrandedColors, int currColor){
    
    uint16_t remap[MAX_SIZE*MAX_SIZE];

    int counts = build_region(currState, remap);

    if(counts == 0){
        return false;
    }
    int color_stranded = 0;

    uint16_t flowHeadRegionFlags[counts];
    uint16_t goalRegionFlags[counts];

    memset(flowHeadRegionFlags, 0, sizeof(flowHeadRegionFlags));
    memset(goalRegionFlags, 0, sizeof(goalRegionFlags));

    for(int color=0; color < color_size; color++){
        uint16_t currFlowHead = currState.current_flow_locs[color];

        if(currFlowHead == goals[color] || color == currColor){
            continue;
        }

        // flow head is approache goal
        int distance = abs(get_row(currFlowHead)-get_row(goals[color])) + abs(get_col(currFlowHead)-get_col(goals[color]));
        if(distance == 1){
            continue;
        }

        uint16_t color_flag = (1 << color);

        // current flow head flag
        for(int dir=0; dir<4; dir++){
            uint16_t new_pos = updatePosition(currFlowHead, dir);
            if(boundaryCheck(new_pos)){
                int neighbor_region = remap[get_row(new_pos)*board_size + get_col(new_pos)];
                if(neighbor_region != 65535){
                    flowHeadRegionFlags[neighbor_region] |= color_flag;
                }
            }
        }

        // goal flag
        for(int dir=0; dir<4; dir++){
            uint16_t new_pos = updatePosition(goals[color], dir);
            if(boundaryCheck(new_pos)){
                int neighbor_region = remap[get_row(new_pos)*board_size + get_col(new_pos)];
                if(neighbor_region != 65535){
                    goalRegionFlags[neighbor_region] |= color_flag;
                }
            }
        }

        int r = 0;
        for(; r<counts; r++){
            // such region touches both current flow's head and goal
            if((flowHeadRegionFlags[r] & color_flag) > 0 && (goalRegionFlags[r] & color_flag) > 0){
                break;
            }
        }

        if(r == counts){
            // return true;
            color_stranded++;
            // cout << "Stranded color: " << color_map[color] << endl;
            if(color_stranded > maxStrandedColors){
                // cout << "Color stranded: " << color_stranded << endl;
                return true;
            }
        }

    }

    return false;
}


bool bottleneck_check(State currState){
    int color = currState.last_color;

    int max_step = 3;

    for(int dir=0; dir<4; dir++){

        uint16_t curr = currState.current_flow_locs[color];
        uint16_t next_pos = updatePosition(curr, dir);

        if(boundaryCheck(next_pos) && wallCheck(next_pos, currState) == false){
            for(int n_steps=0; n_steps < max_step; n_steps++){
                next_pos = updatePosition(next_pos, dir);
                // cannot move anymore along this direction
                if(boundaryCheck(next_pos) == false || wallCheck(next_pos, currState) == true){
                    State nextState = currState;
                    for(int i=0; i<n_steps+1; i++){
                        makeMove(nextState, dir, color);
                    }
                    if(stranded_color(nextState, n_steps+1, color)){
                        // cout << "Find chokepoint" << endl;
                        // cout << "Step taken: " << n_steps+1 << endl;
                        // drawSolution(nextState);
                        return true;
                    }    
                }
            }
        }
    }

    return false;
}

// now begin search
// DFS
State DFS(State init_state){
    stack<State> s;
    set<State> visited;

    s.push(init_state);
    visited.insert(init_state);

    State currState, nextState;
    State endState;
    int expanded_state = 0;
    clock_t t;
    t = clock();
    while(!s.empty()){
        currState = s.top();
        s.pop();

        // forward checking
        if(check_dead_end(currState)){
            continue;
        }

        // one of constraint
        if(checkSource(currState)){
            continue;
        }
        
        if(checkDone(currState)){
            cout << "Find soultion." << endl;
            endState = currState;
            break;
        }

        if(stranded_color_region_check(currState) ){
            continue;
        }

        if(bottleneck_check(currState)){
            continue;
        }

        uint16_t color = nextColor(currState);
        // no available colors for such state
        if(color == 65535){
            continue;
        }
        expanded_state++;
        cout << expanded_state << endl;
        // drawSolution(currState);

        // usleep(200000);

        for(int dir=0; dir<4; dir++){

            if(moveCheck(currState, dir, color)){
                nextState = currState;
                makeMove(nextState, dir, color);
                
                if(visited.find(nextState) == visited.end()){
                    s.push(nextState);
                    visited.insert(nextState);
                }
            }
        }
    }
    t = clock() - t;
    printf ("It took me %lu clicks (%f seconds).\n",t,((float)t)/CLOCKS_PER_SEC);
    cout << "Complete DFS Search" << endl;
    return endState;
}

State Dumb(State init_state){
    stack<State> s;
    set<State> visited;

    s.push(init_state);
    visited.insert(init_state);

    State currState, nextState;
    State endState;
    int expanded_state = 0;
    clock_t t;
    t = clock();
    while(!s.empty()){
        currState = s.top();
        s.pop();

        // one of constraint
        if(checkSource(currState)){
            continue;
        }
        
        if(checkDone(currState)){
            cout << "Find soultion." << endl;
            endState = currState;
            break;
        }

        uint16_t color = nextColor(currState);
        // no available colors for such state
        if(color == 65535){
            continue;
        }
        expanded_state++;
        cout << expanded_state << endl;
        drawSolution(currState);

        // usleep(200000);

        for(int dir=0; dir<4; dir++){

            if(moveCheck(currState, dir, color)){
                nextState = currState;
                makeMove(nextState, dir, color);
                
                if(visited.find(nextState) == visited.end()){
                    s.push(nextState);
                    visited.insert(nextState);
                }
            }
        }
    }
    t = clock() - t;
    printf ("It took me %lu clicks (%f seconds).\n",t,((float)t)/CLOCKS_PER_SEC);
    cout << "Complete DFS Search" << endl;
    return endState;
}

int h_func(State currState){
    int counts = 0;
    for(int i=0; i<board_height; i++){
        for(int j=0; j<board_size; j++){
            if(currState.board[i][j] == 65535){
                counts++;
            }
        }
    }
    return counts;
}

State Greedy_Search(State init_state){
    priority_queue<Node, vector<Node>, NodeComparator> min_heap;
    set<State> visited;

    Node init_node;

    init_node.h = h_func(init_state);
    init_node.f = init_node.h;
    init_node.state = init_state;

    visited.insert(init_state);
    min_heap.push(init_node);

    Node currNode;
    State currState;
    State nextState;
    State endState;

    int n = 0;
    clock_t t;
    t = clock();
    while(!min_heap.empty()){
        currNode = min_heap.top();
        currState = currNode.state;
        min_heap.pop();
        
        if(check_dead_end(currState)){
            continue;
        }
        // one of constraint
        if(checkSource(currState)){
            continue;
        }

        if(checkDone(currState)){
            cout << "Find soultion." << endl;
            endState = currState;
            break;
        }

        if(stranded_color_region_check(currState)){
            continue;
        }

        if(bottleneck_check(currState)){
            continue;
        }

        uint16_t color = nextColor(currState);
        // no available colors for such state
        if(color == 65535){
            continue;
        }
        n++;
        cout << n << endl;
        drawSolution(currState);

        for(int i=0; i<4; i++) {
            if(moveCheck(currState, i, color)){
                nextState = currState;
                makeMove(nextState, i, color);
                
                // did not find in closeSet or current one is better
                if(visited.find(nextState) == visited.end()){
                    Node nextNode;
                    // set up the next node and cost dict
                    nextNode.state = nextState;
                    nextNode.h = h_func(nextState);
                    nextNode.f = nextNode.h;

                    min_heap.push(nextNode);
                    visited.insert(nextState);
                }               
            }
        }
    }
    t = clock() - t;
    printf ("It took me %lu clicks (%f seconds).\n",t,((float)t)/CLOCKS_PER_SEC);
    cout << "Complete Smart Search" << endl;
    return endState;
}

// read input file and storing board size, color size and locs for start and goals
void read(string filename){
    ifstream infile;
    infile.open(filename, ios::binary);
    char c;
    int row = 0;
    int col = 0;
    map<uint16_t, bool> counts;
    map<char, int> color;
    int idx = 0;
    while(!infile.eof()){

        infile >> noskipws >> c;

        if(c != '_' && c >= 65 && c <= 90){
            if(counts.find(c) != counts.end()){
                // find this color
                uint16_t pos = compress_pos(row, col);
                goals[color[c]] = pos;
                cout << "(" << row << "," << col << ")" << " ";
                cout << c << endl;
            }else{
                uint16_t pos = compress_pos(row, col);
                cout << "(" << row << "," << col << ")" << " ";
                // cout << pos << " ";
                color[c] = idx;
                counts[c] = true;
                starts[idx] = pos;
                color_map[idx] = c;
                idx++;
                cout << c << endl;
            }
            col++;

        }else if(c == '_'){
            col++;
        }else{
            
            if(c == '\n'){
                if(row == 0){
                    board_size = col;
                }
                row++;
                col=0;
            }
        }
    }
    board_height = row+1;
    color_size = idx;
    infile.close();
}

uint16_t get_init_color(State init_state){

    int leastFree = 4;
    int next_color = 0;
    for(int i=0; i<color_size; i++){
        uint16_t flowHeadOfThisColor = starts[i];

        // get four neighbiors of current flow head
        // up, down, left, right
        int validNeighbors = 0;

        for(int j=0; j<4; j++){
            uint16_t new_pos = updatePosition(flowHeadOfThisColor, j);
            // inside board and not wall
            if(boundaryCheck(new_pos) == true && (wallCheck(new_pos, init_state) == false || new_pos == goals[i])) {
                validNeighbors++;
            }
        }
        // cout << "color: " << color_map[i] << " neighbiors: " << validNeighbors << endl;
        if(validNeighbors < leastFree && validNeighbors > 0){
            leastFree = validNeighbors;
            next_color = i;
        }
    }
    return next_color;
}

int main(void){

    string filename = "input1414.txt";
    read(filename);

    cout << "color size: " << color_size << endl;
    cout << "board size: " << board_size << endl;
    cout << "board height: " << board_height << endl;
    cout << "start locs: " << endl;
    for(int i=0; i<color_size; i++){
        uint16_t x = get_row(starts[i]);
        uint16_t y = get_col(starts[i]);
        cout << "(" << x << "," << y << ")" << " ";
    }
    cout << endl;

    cout << "goal locs: " << endl;
    for(int i=0; i<color_size; i++){
        uint16_t x = get_row(goals[i]);
        uint16_t y = get_col(goals[i]);
        cout << "(" << x << "," << y << ")" << " ";
    }
    cout << endl;

    // init state
    State init_state;
    for(int i=0; i<color_size; i++){
        init_state.current_flow_locs[i] = starts[i];
        init_state.board[get_row(starts[i])][get_col(starts[i])] = i;
        init_state.board[get_row(goals[i])][get_col(goals[i])] = i;
    }

    init_state.last_color = get_init_color(init_state);

    State solution = DFS(init_state);
    // cout << "Begin Animation!" << endl;
    // drawSolution(init_state);

    // for(int i=0; i<color_size; i++){
        
    // }

    drawSolution(solution);


    cout << "\033[1;31mbold red text\033[0m\n";

    return 0;
}
















