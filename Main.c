#include <SDL3/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <limits.h>

#define GRID_WIDTH 20
#define GRID_HEIGHT 15
#define CELL_SIZE 40
// K_PATHS is how many disjoint paths to find
#define K_PATHS 5 

typedef enum {
    CELL_EMPTY,
    CELL_WALL,
    CELL_START, // Will be Green
    CELL_END,   // Will be Red
    // Specific path types for different shades of blue
    CELL_PATH_1,
    CELL_PATH_2,
    CELL_PATH_3,
    CELL_PATH_4,
    CELL_PATH_5
} CellType;

typedef struct {
    int x, y;
} Point;

// Struct for Dijkstra's priority queue
typedef struct Node {
    Point pos;
    int cost;
} Node;

// Struct to store a single complete path
typedef struct {
    Point points[GRID_WIDTH * GRID_HEIGHT]; // Max possible path length
    int length;
    int cost;
} Path;

CellType grid[GRID_HEIGHT][GRID_WIDTH]; // Main grid for walls, start, end
CellType grid_path_type[GRID_HEIGHT][GRID_WIDTH]; // Stores which path type a cell is (CELL_PATH_1, etc.)
Point start = { -1, -1 };
Point end = { -1, -1 };
bool start_selected = false;
bool end_selected = false;
bool paths_found_and_drawn = false;

// Initialize grid with random walls
void initialize_grid() {
    srand(time(NULL));
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            grid[y][x] = (rand() % 4 == 0) ? CELL_WALL : CELL_EMPTY;
            grid_path_type[y][x] = CELL_EMPTY; // Initialize path type grid
        }
    }

    start.x = start.y = -1;
    end.x = end.y = -1;
    start_selected = false;
    end_selected = false;
    paths_found_and_drawn = false;
}

Point screen_to_grid(int screen_x, int screen_y) {
    Point grid_pos;
    grid_pos.x = screen_x / CELL_SIZE;
    grid_pos.y = screen_y / CELL_SIZE;
    return grid_pos;
}

bool is_valid_position(int x, int y) {
    // A position is valid if it's in bounds AND
    // not a wall AND not already part of any found path
    // (by checking if grid_path_type is not CELL_EMPTY).
    return (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT &&
        grid[y][x] != CELL_WALL &&
        grid_path_type[y][x] == CELL_EMPTY); // Check the path_type grid
}

/**
 * @brief Finds the single shortest path using Dijkstra's algorithm.
 * * @return Path struct. cost is -1 if no path is found.
 */
Path dijkstra_find_path() {
    Path result_path;
    result_path.length = 0;
    result_path.cost = -1;

    // Check if start/end are still valid (e.g., not walled in)
    // We check the start, but the end might be on a path (which is invalid),
    // so we need a special check for the end node during neighbor exploration.
    if (!is_valid_position(start.x, start.y)) {
        // Check if end is also invalid *for the same reason*
        if (!is_valid_position(end.x, end.y) && grid_path_type[end.y][end.x] != CELL_EMPTY) {
            // This is fine, end can be on a path
        }
        else {
            return result_path; // Start is blocked
        }
    }


    int dist[GRID_HEIGHT][GRID_WIDTH];
    bool visited[GRID_HEIGHT][GRID_WIDTH] = { false };
    // Store the parent point for each node to reconstruct the path
    Point parent[GRID_HEIGHT][GRID_WIDTH];

    // Initialize all distances to infinity and parents to -1
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            dist[y][x] = INT_MAX;
            parent[y][x] = (Point){ -1, -1 };
        }
    }

    // Priority queue (simple array of nodes to visit)
    Node* nodes[GRID_WIDTH * GRID_HEIGHT];
    int node_count = 0;

    // Add start node
    Node* start_node = malloc(sizeof(Node));
    start_node->pos = start;
    start_node->cost = 0;
    nodes[node_count++] = start_node;
    dist[start.y][start.x] = 0;

    int dx[] = { 0, 1, 0, -1 };
    int dy[] = { -1, 0, 1, 0 };

    Node* current = NULL;
    bool path_found = false;

    while (node_count > 0) {
        // Find node with minimum cost in the queue
        int min_index = 0;
        for (int i = 1; i < node_count; i++) {
            if (nodes[i]->cost < nodes[min_index]->cost)
                min_index = i;
        }

        current = nodes[min_index];
        // Remove from queue
        nodes[min_index] = nodes[--node_count];

        int cx = current->pos.x;
        int cy = current->pos.y;

        if (visited[cy][cx]) {
            free(current); // Free node, already processed
            continue;
        }
        visited[cy][cx] = true;

        // Reached destination
        if (cx == end.x && cy == end.y) {
            path_found = true;
            break; // Exit while loop
        }

        // Explore neighbors
        for (int i = 0; i < 4; i++) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];

            // Check validity of neighbor. IMPORTANT: The end node is ALWAYS a valid target,
            // even if it was part of a previous path.
            bool is_neighbor_end = (nx == end.x && ny == end.y);

            // If it's not the end, check if it's a valid position
            if (!is_neighbor_end && !is_valid_position(nx, ny))
                continue;

            if (visited[ny][nx]) // Already processed in this Dijkstra's iteration
                continue;

            int new_cost = dist[cy][cx] + 1; // cost per move = 1

            if (new_cost < dist[ny][nx]) {
                dist[ny][nx] = new_cost;
                parent[ny][nx] = (Point){ cx, cy }; // Store parent

                Node* neighbor = malloc(sizeof(Node));
                neighbor->pos.x = nx;
                neighbor->pos.y = ny;
                neighbor->cost = new_cost;

                nodes[node_count++] = neighbor;
            }
        }
        free(current); // Free processed node
        current = NULL;
    }

    // --- Path Reconstruction ---
    if (path_found && current != NULL) {
        result_path.cost = current->cost;
        Point at = end;
        int path_len = 0;

        // Store path in reverse (end to start)
        Point reverse_path[GRID_WIDTH * GRID_HEIGHT];
        while (at.x != -1 && at.y != -1) {
            reverse_path[path_len++] = at;
            if (at.x == start.x && at.y == start.y)
                break; // Reached start
            at = parent[at.y][at.x];
        }

        // Now reverse the path into the result struct
        result_path.length = path_len;
        for (int i = 0; i < path_len; i++) {
            result_path.points[i] = reverse_path[path_len - 1 - i];
        }

        free(current); // Free the end node
    }

    // --- Cleanup ---
    // Free any remaining nodes in the queue (if no path found or loop broken)
    for (int i = 0; i < node_count; i++)
        free(nodes[i]);

    return result_path;
}


// Draw the grid
void draw_grid(SDL_Renderer* renderer) {
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            SDL_FRect cell_rect = { (float)x * CELL_SIZE, (float)y * CELL_SIZE, (float)CELL_SIZE, (float)CELL_SIZE };

            // Default color for non-path cells
            SDL_Color cell_color = { 200, 200, 200, 255 }; // Empty

            switch (grid[y][x]) {
            case CELL_EMPTY:
                // Handled by default cell_color
                break;
            case CELL_WALL:
                cell_color = (SDL_Color){ 50, 50, 50, 255 };
                break;
            case CELL_START:
                cell_color = (SDL_Color){ 0, 255, 0, 255 }; // Green
                break;
            case CELL_END:
                cell_color = (SDL_Color){ 255, 0, 0, 255 }; // Red
                break;
                // No default CELL_PATH here, rely on grid_path_type for paths
            default: // Should not happen for grid[][] if only walls/empty/start/end
                break;
            }

            // Check the grid_path_type for path segments
            // Colors are from the user-provided palette
            // Shortest (Path 1) = Darkest Blue
            // Longest (Path 5) = Lightest Blue
            switch (grid_path_type[y][x]) {
            case CELL_PATH_1: cell_color = (SDL_Color){ 2, 136, 209, 255 }; break;   // Darkest
            case CELL_PATH_2: cell_color = (SDL_Color){ 41, 182, 246, 255 }; break;
            case CELL_PATH_3: cell_color = (SDL_Color){ 129, 212, 250, 255 }; break;
            case CELL_PATH_4: cell_color = (SDL_Color){ 179, 229, 252, 255 }; break;
            case CELL_PATH_5: cell_color = (SDL_Color){ 224, 247, 250, 255 }; break; // Lightest
            default: break; // No path segment or already handled by grid[][]
            }

            SDL_SetRenderDrawColor(renderer, cell_color.r, cell_color.g, cell_color.b, cell_color.a);
            SDL_RenderFillRect(renderer, &cell_rect);
            SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255); // Grid lines
            SDL_RenderRect(renderer, &cell_rect);
        }
    }
}

void handle_click(int x, int y) {
    Point grid_pos = screen_to_grid(x, y);

    if (grid_pos.x < 0 || grid_pos.x >= GRID_WIDTH || grid_pos.y < 0 || grid_pos.y >= GRID_HEIGHT)
        return;

    // Prevent clicks if pathfinding is done. Must reset.
    if (paths_found_and_drawn) {
        printf("Paths already found. Press 'C' or 'R' to reset.\n");
        return;
    }

    // Special check: don't allow clicking on a wall
    if (grid[grid_pos.y][grid_pos.x] == CELL_WALL)
        return;

    if (!start_selected) {
        start = grid_pos;
        grid[start.y][start.x] = CELL_START;
        start_selected = true;
        printf("Start set at (%d, %d)\n", start.x, start.y);
    }
    else if (!end_selected) {
        if (!(grid_pos.x == start.x && grid_pos.y == start.y)) {
            end = grid_pos;
            grid[end.y][end.x] = CELL_END;
            end_selected = true;
            printf("End set at (%d, %d)\n", end.x, end.y);

            paths_found_and_drawn = true; // Mark that we are starting the process

            printf("Finding %d shortest disjoint paths...\n", K_PATHS);
            printf("----------------------------------------\n");

            for (int i = 0; i < K_PATHS; i++) {
                Path path = dijkstra_find_path();

                if (path.cost == -1) {
                    printf("No more paths found.\n");
                    break;
                }

                // Path found! Print cost.
                printf("  Path %d Cost: %d\n", i + 1, path.cost);

                // Assign the correct CELL_PATH_N type based on iteration
                CellType current_path_type;
                switch (i) {
                case 0: current_path_type = CELL_PATH_1; break;
                case 1: current_path_type = CELL_PATH_2; break;
                case 2: current_path_type = CELL_PATH_3; break;
                case 3: current_path_type = CELL_PATH_4; break;
                case 4: current_path_type = CELL_PATH_5; break;
                default: current_path_type = CELL_EMPTY; // Should not happen
                }

                // Iterate through path to color it AND turn nodes into "walls" for future searches
                for (int p_idx = 0; p_idx < path.length; p_idx++) {
                    Point p = path.points[p_idx];

                    // Don't change start or end nodes
                    if ((p.x == start.x && p.y == start.y) || (p.x == end.x && p.y == end.y))
                        continue;

                    // Set to current_path_type.
                    // This colors it the correct shade of blue (via draw_grid)
                    // AND makes it invalid for the next search (via is_valid_position)
                    grid_path_type[p.y][p.x] = current_path_type;
                }
            }
            printf("----------------------------------------\n");
            printf("Path search complete.\n");
        }
    }
}

void reset_grid() {
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            // Only reset non-wall cells on the main grid
            if (grid[y][x] != CELL_WALL)
                grid[y][x] = CELL_EMPTY;
            // Always reset the path type grid
            grid_path_type[y][x] = CELL_EMPTY;
        }
    }

    start_selected = false;
    end_selected = false;
    paths_found_and_drawn = false;
    start.x = start.y = -1;
    end.x = end.y = -1;
}

int main(int argc, char* argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL initialization failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "SDL3 K-Shortest Paths Visualizer (Dijkstra)",
        GRID_WIDTH * CELL_SIZE,
        GRID_HEIGHT * CELL_SIZE,
        0
    );

    if (!window) {
        fprintf(stderr, "Window creation failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL);
    if (!renderer) {
        fprintf(stderr, "Renderer creation failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    initialize_grid();

    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
            case SDL_EVENT_QUIT: running = false; break;
            case SDL_EVENT_MOUSE_BUTTON_DOWN:
                if (event.button.button == SDL_BUTTON_LEFT)
                    handle_click(event.button.x, event.button.y);
                break;
            case SDL_EVENT_KEY_DOWN:
                if (event.key.key == SDLK_R) {
                    initialize_grid();
                    printf("Grid randomized and reset.\n");
                }
                else if (event.key.key == SDLK_C) {
                    reset_grid();
                    printf("Grid cleared for new pathfinding.\n");
                }
                break;
            }
        }

        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderClear(renderer);
        draw_grid(renderer);
        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}

