import gridcell

n = int(input("Количество строк"))
m = int(input("Количество столбцов"))
random_coverage = int(input("Величина случайного распределения количества препятсвии в %"))
tower_coverage = int(input("Величина покрытия башен"))
bashni = input('Будут ли строяться маршруты между башнями y/n')
while bashni not in ['y','n']:
    print('Неправильный ввод')
budget = input('Будет ли ограниченный бюджет y/n')
while budget not in ['y','n']:
    print('Неправильный ввод')
    budget = input('Будет ли ограниченный бюджет y/n')
if budget == 'y':
    bud = input('Введите бюджет')
    while not bud.isdigit():
        print('Неправильный ввод')
        bud = input('Введите бюджет')

    cost_for_tower = input('Введите цену за башню')
    while not cost_for_tower.isdigit():
        print('Неправильный ввод')
        cost_for_tower = input('Введите цену за башню')
    grid = gridcell.CityGrid(n, m, random_coverage, tower_coverage, float(bud),float(cost_for_tower))
else:
    grid = gridcell.CityGrid(n, m, random_coverage, tower_coverage)

if bashni == 'y':

    grid.optimization_for_paths()
    grid.find_neibours_for_towers()
    tower_coord_from_1 = int(input())
    tower_coord_from_2 = int(input())
    tower_coord_to_3 = int(input())
    tower_coord_to_4 = int(input())

    path = grid.find_path((tower_coord_from_1, tower_coord_from_2), (tower_coord_to_3, tower_coord_to_4))
    print(path)
    grid.visualize_path(path)
else:

    grid.optimization_towers()
    grid.visualize_grid()
