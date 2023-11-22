import random
import matplotlib.pyplot as plt
import numpy as np

class CityGrid:
    def __init__(self, n, m, random_coverage,tower_range,budget = 1,one_tower_cost = 0):
        self.n = n
        self.m = m
        self.random_coverage = random_coverage
        self.matrix = [[0 for i in range(n)] for j in range(m)]
        self.tower_range = tower_range
        self.budget = budget
        self.one_tower_cost = one_tower_cost
        for i in range(len(self.matrix)):
            for j in range(len(self.matrix[0])):
                if random.randint(0, 100) <= self.random_coverage:
                    self.matrix[i][j] = 1
        self.tower_poses = {}
        self.print_mtx()

    def print_mtx(self):
        print('\n'.join(map(str, self.matrix)))
        print('#########################')

    def place_tower(self, row, col, tower, budget = False):
        if self.matrix[row][col] == 1:
            return None
        self.matrix[row][col] = 3
        tower_range = tower.range
        upper_cell = (max(0, row-tower_range), max(0, col-tower_range))
        lower_cell = (min(self.m, row+tower_range+1), min(self.n, col+tower_range+1))
        self.tower_poses[(row, col)] = tower
        for row in range(upper_cell[0],lower_cell[0]):
            for col in range(upper_cell[1], lower_cell[1]):
                if self.matrix[row][col] in (1, 3):
                    continue
                else:
                    self.matrix[row][col] = 2
        if budget:
            self.budget-=self.one_tower_cost


    #Оптимизация расстановки башен если не нужно связывать их маршрутами
    def optimization_towers(self):
        empty_spaces = {}
        obstacles = {}

        for i in range(len(self.matrix)):
            for j in range(len(self.matrix[0])):
                if self.matrix[i][j] == 0:
                    empty_spaces[(i,j)] = 1
                elif self.matrix[i][j] == 1:
                    obstacles[(i,j)] = 1
        while empty_spaces and self.budget>0:
            tower = Tower(self.tower_range)
            max_covering = 0
            for cell in empty_spaces:
                covering, counted_indxs = self.count_covering(cell,tower.range)
                if max_covering < covering:
                    max_covering = covering
                    cover_cell = cell
                    counted_indxs_need = counted_indxs
            tower.pos = cover_cell
            if self.one_tower_cost:
              self.place_tower(cover_cell[0],cover_cell[1],tower,budget=True)
            else:
              self.place_tower(cover_cell[0], cover_cell[1], tower)
            if empty_spaces.get(cover_cell):
                empty_spaces.pop(cover_cell)
            for indxs in counted_indxs_need:
                if empty_spaces.get(indxs):
                    empty_spaces.pop(indxs)
                if obstacles.get(indxs):
                    obstacles.pop(indxs)
        self.print_mtx()

    # Оптимизация расстановки башен если нужно связывать их маршрутами
    def optimization_for_paths(self):
        empty_spaces = {}
        tower = Tower(self.tower_range)
        for i in range(len(self.matrix)):
            for j in range(len(self.matrix[0])):
                if self.matrix[i][j] == 0:
                    empty_spaces[(i, j)] = 1
        edge_points = []
        max_covering = 0
        for cell in empty_spaces:
            covering, counted_indxs,other_counted = self.count_covering(cell, tower.range,only_edge = True)
            if max_covering < covering:
                max_covering = covering
                cover_cell = cell
                counted_indxs_need = counted_indxs
                other_counted_needed = other_counted

        edge_points += counted_indxs_need
        tower.pos = cover_cell
        if self.one_tower_cost:
            self.place_tower(cover_cell[0], cover_cell[1], tower, budget=True)
        else:
            self.place_tower(cover_cell[0], cover_cell[1], tower)
        if empty_spaces.get(cover_cell):
            empty_spaces.pop(cover_cell)
        for c_o in other_counted_needed:
            if empty_spaces.get(c_o):
                empty_spaces.pop(c_o)
        while empty_spaces and self.budget>0:
            tower = Tower(self.tower_range)
            max_covering = 0
            for i in edge_points:
                covering, counted_indxs,other_counted = self.count_covering(i, tower.range, only_edge=True)
                if max_covering < covering:
                    max_covering = covering
                    cover_cell = i
                    counted_indxs_need = counted_indxs
                    other_counted_needed = other_counted
            if max_covering == 0:
                for i in empty_spaces:
                    covering, counted_indxs, other_counted = self.count_covering(i, tower.range, only_edge=True)
                    if max_covering < covering:
                        max_covering = covering
                        cover_cell = i
                        counted_indxs_need = counted_indxs
                        other_counted_needed = other_counted

            tower.pos = cover_cell
            if self.one_tower_cost:
                self.place_tower(cover_cell[0], cover_cell[1], tower, budget=True)
            else:
                self.place_tower(cover_cell[0], cover_cell[1], tower)

            if cover_cell in edge_points:
               edge_points.remove(cover_cell)
            edge_points += counted_indxs_need
            if empty_spaces.get(cover_cell):
                empty_spaces.pop(cover_cell)
            for counted_covered in other_counted_needed:
                if empty_spaces.get(counted_covered):
                    empty_spaces.pop(counted_covered)
        self.print_mtx()

    def find_neibours_for_towers(self):
        for pos, tower  in self.tower_poses.items():
            tower_range = tower.range
            row, col = pos
            upper_cell = (max(0, row - tower_range), max(0, col - tower_range))
            lower_cell = (min(self.m, row + tower_range + 1), min(self.n, col + tower_range + 1))
            for row in range(upper_cell[0], lower_cell[0]):
                for col in range(upper_cell[1], lower_cell[1]):
                    if self.matrix[row][col] == 3:
                        tower.nei.append(self.tower_poses[(row, col)])

    def build_path(self,t):
        path = [t]
        while t.prev_tower != None:
            path.append(t.prev_tower)
            t = t.prev_tower
        return path[::-1]

    def find_path(self,tower_s,tower_d):
        for pos ,tower in self.tower_poses.items():
            tower.cost = float('inf')
            tower.prev_tower = None
        source = self.tower_poses[tower_s]
        dest = self.tower_poses[tower_d]
        removed_towers = []
        source.cost = 0
        can_travel = [i for i in self.tower_poses.values()]

        while can_travel:
            min_dist = float('inf')
            min_node = None
            for n in can_travel:
                if n.cost < min_dist and n not in removed_towers:
                    min_dist = n.cost
                    min_node = n

            can_travel.remove(min_node)
            removed_towers.append(min_node)
            for n in min_node.nei:
                total_sum = min_node.cost+1
                if total_sum < n.cost:
                    n.cost = total_sum
                    n.prev_tower = min_node

            if min_node == dest:
                return self.build_path(min_node)
        return None

    def count_covering(self, cell, r, only_edge = False):
        count = 0
        tower_range = r
        row, col = cell
        count_range = []
        upper_cell = (max(0, row - tower_range), max(0, col - tower_range))
        lower_cell = (min(self.m, row + tower_range + 1), min(self.n, col + tower_range + 1))
        for row in range(upper_cell[0], lower_cell[0]):
            for col in range(upper_cell[1], lower_cell[1]):
                if self.matrix[row][col] in (1,2,3):
                    continue
                else:
                    count_range.append((row, col))
                    count += 1
        if only_edge:
            return count, list(filter(lambda x: (x[0] == upper_cell[0] or x[0] == lower_cell[0]-1)
                                                or (x[1] == lower_cell[1]-1 or x[1] == upper_cell[1]), count_range)),\
                                                count_range
        else:
            return count, count_range

    def visualize_grid(self):
        fig, ax = plt.subplots()
        flag_free = False
        flag_block = False
        flag_covered = False
        flag_towers = False
        for row in range(self.m):
            for col in range(self.n):
                if self.matrix[row][col] == 0:
                    if not flag_free:
                      ax.scatter(col,self.m-row,s = 30,c = ['#05ed37'],marker= 'o',label = 'Свободное зона')
                    else:
                      ax.scatter(col, self.m - row, s=30, c=['#05ed37'], marker='o')
                    flag_free = True
                elif self.matrix[row][col] == 1:
                    if not flag_block:
                       ax.scatter(col,self.m-row,s = 30, c = ['#ed0505'], marker= 'o',label = 'Блокированные зоны')    # Блокированные блоки (красные)
                    else:
                       ax.scatter(col, self.m - row, s=30, c=['#ed0505'], marker='o')
                    flag_block = True
                elif self.matrix[row][col] == 2:
                    if not flag_covered:
                       ax.scatter(col,self.m-row,s = 30, c = ['#ed05d6'],marker= 'v',label = 'Покрытые башней зоны')  # Покрытые башней зоны (фиолетовые)
                    else:
                       ax.scatter(col, self.m - row, s=30, c=['#ed05d6'], marker='v')
                    flag_covered = True
                elif self.matrix[row][col] == 3:
                    if not flag_towers:
                      ax.scatter(col,self.m-row,s = 30, c = ['#05edd2'],marker= 'v',label = 'Башни')  # Башни (бирюзовые)
                    else:
                      ax.scatter(col, self.m - row, s=30, c=['#05edd2'], marker='v')  # Башни (бирюзовые)
                    flag_towers = True
        ax.legend()
        plt.show()

    def visualize_path(self,path):
        fig2, ax2 = plt.subplots()
        flag_free = False
        flag_block = False
        flag_covered = False
        flag_towers = False
        for row in range(self.m):
            for col in range(self.n):
                if self.matrix[row][col] == 0:
                    if not flag_free:
                        ax2.scatter(col, self.m - row, s=30, c=['#05ed37'], marker='o', label='Свободное зона')
                    else:
                        ax2.scatter(col, self.m - row, s=30, c=['#05ed37'], marker='o')
                    flag_free = True
                elif self.matrix[row][col] == 1:
                    if not flag_block:
                        ax2.scatter(col, self.m - row, s=30, c=['#ed0505'], marker='o',
                                   label='Блокированные зоны')  # Блокированные блоки (красные)
                    else:
                        ax2.scatter(col, self.m - row, s=30, c=['#ed0505'], marker='o')
                    flag_block = True
                elif self.matrix[row][col] == 2:
                    if not flag_covered:
                        ax2.scatter(col, self.m - row, s=30, c=['#ed05d6'], marker='v',
                                   label='Покрытые башней зоны')  # Покрытые башней зоны (фиолетовые)
                    else:
                        ax2.scatter(col, self.m - row, s=30, c=['#ed05d6'], marker='v')
                    flag_covered = True
                elif self.matrix[row][col] == 3:
                    if not flag_towers:
                        ax2.scatter(col, self.m - row, s=30, c=['#05edd2'], marker='v',
                                   label='Башни')  # Башни (бирюзовые)
                    else:
                        ax2.scatter(col, self.m - row, s=30, c=['#05edd2'], marker='v')  # Башни (бирюзовые)
                    flag_towers = True
        x = [self.m-i.pos[0] for i in path]
        y = [i.pos[1] for i in path]

        ax2.plot(y, x)

        plt.show()



class Tower:
    def __init__(self, range):
        self.range = range
        self.pos = None
        self.nei = []
        self.cost = float('inf')
        self.prev_tower = None

