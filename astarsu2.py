import heapq

class Cell:
    def __init__(self):
        self.parent_x = 0
        self.parent_y = 0
        self.f = float('inf')
        self.g = float('inf')
        self.h = 0

    def is_valid(self, x, y, COL, ROW):
        return (x >= 0) and (x < COL) and (y >= 0) and (y < ROW)

    def is_unblocked(self, grid, x, y):
        return grid[y][x] == 1

    def is_destination(self, x, y, dest):
        return x == dest[0] and y == dest[1]

    def euclidean_h_value(self, x, y, dest):
        return ((x - dest[0]) ** 2 + (y - dest[1]) ** 2) ** 0.5

def trace_path(cell_details, dest):
    path = []
    x = dest[0]
    y = dest[1]
    # total_euclidean_distance = 0

    while not (cell_details[y][x].parent_x == x and cell_details[y][x].parent_y == y):
        path.append((x, y))
        temp_x = cell_details[y][x].parent_x
        temp_y = cell_details[y][x].parent_y
        # total_euclidean_distance += cell_details[temp_y][temp_x].h
        x = temp_x
        y = temp_y

    path.append((x, y))
    path.reverse()

    return path

def a_star_search(grid, src, dest, ROW, COL):
    grid_val = Cell()
    
    if not grid_val.is_valid(src[0], src[1], COL, ROW) or not grid_val.is_valid(dest[0], dest[1], COL, ROW):
        print("Source or destination is invalid")
        return
    if not grid_val.is_unblocked(grid, src[0], src[1]) or not grid_val.is_unblocked(grid, dest[0], dest[1]):
        print("Source or the destination is blocked")
        return
    if grid_val.is_destination(src[0], src[1], dest):
        print("We are already at the destination")
        return

    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
    cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]

    x = src[0]
    y = src[1]
    cell_details[y][x].f = 0
    cell_details[y][x].g = 0
    cell_details[y][x].h = 0
    cell_details[y][x].parent_x = x
    cell_details[y][x].parent_y = y

    open_list = []
    heapq.heappush(open_list, (0.0, x, y))

    found_dest = False

    directions = [
        (0, -1, 1), (-1, -1, 1.414), (-1, 0, 1), (-1, 1, 1.414),
        (0, 1, 1), (1, 1, 1.414), (1, 0, 1), (1, -1, 1.414)
    ]
    
    while len(open_list) > 0:
        p = heapq.heappop(open_list)
        x = p[1]
        y = p[2]
        closed_list[y][x] = True

        for dir in directions:
            new_x = x + dir[0]
            new_y = y + dir[1]
            move_cost = dir[2]

            if grid_val.is_valid(new_x, new_y, COL, ROW) and grid_val.is_unblocked(grid, new_x, new_y) and not closed_list[new_y][new_x]:
                if move_cost == 1.414:
                    if not (grid_val.is_unblocked(grid, x, new_y) and grid_val.is_unblocked(grid, new_x, y)):
                        continue

                if grid_val.is_destination(new_x, new_y, dest):
                    cell_details[new_y][new_x].parent_x = x
                    cell_details[new_y][new_x].parent_y = y
                    print("The destination cell is found")
                    found_dest = True
                    path= trace_path(cell_details, dest)
                    # print("Total Euclidean distance:", total_euclidean_distance)
                    return path
                else:
                    g_new = cell_details[y][x].g + move_cost
                    h_new = grid_val.euclidean_h_value(new_x, new_y, dest)
                    f_new = g_new + h_new

                    if cell_details[new_y][new_x].f == float('inf') or cell_details[new_y][new_x].f > f_new:
                        heapq.heappush(open_list, (f_new, new_x, new_y))

                        cell_details[new_y][new_x].f = f_new
                        cell_details[new_y][new_x].g = g_new
                        cell_details[new_y][new_x].h = h_new
                        cell_details[new_y][new_x].parent_x = x
                        cell_details[new_y][new_x].parent_y = y

    if not found_dest:
        print("Failed to find the destination cell")