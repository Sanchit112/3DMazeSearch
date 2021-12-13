from collections import deque
import heapq
import os, math
from bisect import bisect_left


class Node:
    def __init__(self, node: tuple, parent=None, cost: float = 0.0, heuristic: float = 0.0):
        self.cordinate = node
        self.parent = parent
        self.cost = cost
        self.h = heuristic
        self.g = 0

    def __lt__(self, other):
        return (self.g + self.h) < (other.g + other.h)


class Maze:
    def __init__(self):
        input_file = 'input.txt'
        with open(input_file, 'r') as f:
            self.algo = f.readline().strip()
            self.size = tuple(map(int, f.readline().strip().split()))
            self.start = Node(tuple(map(int, f.readline().strip().split())))
            self.goal = Node(tuple(map(int, f.readline().strip().split())))
            self.n = int(f.readline())
            self.search_list = []
            self.start_loc = {}
            self.cache_loc = None
            self.cache_loc_dict = {}
            self.dict_counter = 0
            self.LEN = None

            #
            # for i in range(self.n):
            #     temp = tuple(map(int, f.readline().strip().split()))
            #     self.options[temp[0:3]] = temp[3:]

            fileEnd = os.path.getsize(input_file)

            if self.n > 100000:
                # print(math.log10(self.n))
                z = 10 ** (math.floor(math.log10(self.n)) - 4)
            elif self.n > 10000000:
                # print(math.log10(self.n))
                z = 10 ** (math.floor(math.log10(self.n)) - 7)
                print(z)
            else:
                z = 50

            size = fileEnd // z

            # print(z)

            start = f.tell()
            end = start + size

            while end <= fileEnd:
                f.seek(start)
                # print(f.readline())
                st = tuple(map(int, f.readline().strip().split()))[0:3]
                self.search_list.append(st)
                self.start_loc[st] = start
                f.seek(end)
                f.readline()
                # print(f.readline())
                ed = tuple(map(int, f.readline().strip().split()))[0:3]
                end = f.tell()
                # print('-' * 100)
                start = end
                end = start + size

            with open(input_file, 'rb') as f:
                with open(input_file, 'rb') as f:
                    f.seek(-2, os.SEEK_END)
                    while f.read(1) != b'\n':
                        f.seek(-2, os.SEEK_CUR)
                    ed = tuple(map(int, f.readline().decode().strip().split()))[0:3]
                    self.search_list.append(ed)
                    self.start_loc[ed] = fileEnd

            self.LEN = len(self.search_list)

            self.action = {
                1: self.xp,
                2: self.xm,
                3: self.yp,
                4: self.ym,
                5: self.zp,
                6: self.zm,
                7: self.xpyp,
                8: self.xpym,
                9: self.xmyp,
                10: self.xmym,
                11: self.xpzp,
                12: self.xpzm,
                13: self.xmzp,
                14: self.xmzm,
                15: self.ypzp,
                16: self.ypzm,
                17: self.ymzp,
                18: self.ymzm
            }

    def xp(self, n: Node):
        return Node((n.cordinate[0] + 1, n.cordinate[1], n.cordinate[2]), n)

    def yp(self, n: Node):
        return Node((n.cordinate[0], n.cordinate[1] + 1, n.cordinate[2]), n)

    def zp(self, n: Node):
        return Node((n.cordinate[0], n.cordinate[1], n.cordinate[2] + 1), n)

    def xm(self, n: Node):
        return Node((n.cordinate[0] - 1, n.cordinate[1], n.cordinate[2]), n)

    def ym(self, n: Node):
        return Node((n.cordinate[0], n.cordinate[1] - 1, n.cordinate[2]))

    def zm(self, n: Node):
        return Node((n.cordinate[0], n.cordinate[1], n.cordinate[2] - 1))

    def xpyp(self, n: Node):
        return Node((n.cordinate[0] + 1, n.cordinate[1] + 1, n.cordinate[2]), n)

    def xmyp(self, n: Node):
        return Node((n.cordinate[0] - 1, n.cordinate[1] + 1, n.cordinate[2]), n)

    def xpym(self, n: Node):
        return Node((n.cordinate[0] + 1, n.cordinate[1] - 1, n.cordinate[2]), n)

    def xmym(self, n: Node):
        return Node((n.cordinate[0] - 1, n.cordinate[1] - 1, n.cordinate[2]), n)

    def xpzp(self, n: Node):
        return Node((n.cordinate[0] + 1, n.cordinate[1], n.cordinate[2] + 1), n)

    def xmzp(self, n: Node):
        return Node((n.cordinate[0] - 1, n.cordinate[1], n.cordinate[2] + 1), n)

    def xpzm(self, n: Node):
        return Node((n.cordinate[0] + 1, n.cordinate[1], n.cordinate[2] - 1), n)

    def xmzm(self, n: Node):
        return Node((n.cordinate[0] - 1, n.cordinate[1], n.cordinate[2] - 1), n)

    def ypzp(self, n: Node):
        return Node((n.cordinate[0], n.cordinate[1] + 1, n.cordinate[2] + 1), n)

    def ymzp(self, n: Node):
        return Node((n.cordinate[0], n.cordinate[1] - 1, n.cordinate[2] + 1), n)

    def ypzm(self, n: Node):
        return Node((n.cordinate[0], n.cordinate[1] + 1, n.cordinate[2] - 1), n)

    def ymzm(self, n: Node):
        return Node((n.cordinate[0], n.cordinate[1] - 1, n.cordinate[2] - 1), n)

    def search(self, key: tuple, space):
        pos = bisect_left(space, key)
        if pos == 0:
            return 0
        if pos == self.LEN:
            return pos - 1
        return pos

    def get_options(self, key):

        if key in self.cache_loc_dict: return self.cache_loc_dict[key]

        if self.dict_counter == 10:
            self.dict_counter = 0
            self.cache_loc = None
            self.cache_loc = {}

        self.cache_loc = self.search(key, self.search_list)

        if self.cache_loc - 2 <= 0:
            start = 0
        else:
            start = self.cache_loc - 2

        if self.cache_loc + 2 >= self.LEN:
            end = self.LEN - 1
        else:
            end = self.cache_loc + 2
        ## first creating a dict
        with open('input.txt', 'r') as f:
            f.seek(self.start_loc[self.search_list[start]])
            for line in f:
                temp = tuple(map(int, line.strip().split()))

                self.cache_loc_dict[temp[0:3]] = temp[3:]
                if temp[0:3] >= self.search_list[end]:
                    self.dict_counter += 1
                    break

        return self.cache_loc_dict[key]

    def is_valid(self, n):
        if 0 <= n[0] <= self.size[0] and 0 <= n[1] <= self.size[1] and 0 <= n[2] <= self.size[2]:
            return True
        else:
            return False

    def successors(self, n: Node):
        next_nodes = []

        # for i in self.get_options(n.cordinate):
        for i in self.get_options(n.cordinate):
            new = self.action[i](n)
            if not self.is_valid(new.cordinate):
                continue
            new.parent = n
            if self.algo == 'BFS':
                new.cost = 1
            else:
                if i <= 6:
                    new.cost = 10
                else:
                    new.cost = 14
            next_nodes.append(new)
        return next_nodes

    def euclideanDistance(self, n: Node, m: Node):
        return sum((x - y) ** 2 for x, y in zip(n.cordinate, m.cordinate)) ** 0.5

    def manhattanDistance(self, n: Node, m: Node):
        return sum(abs(x - y) for x, y in zip(n.cordinate, m.cordinate))

    def pathPrint(self, n: Node):
        path = [n]
        cost = n.cost
        while n.parent:
            n = n.parent
            cost += n.cost
            path.append(n)

        with open('output.txt', 'w') as f:
            print(int(cost), file=f)
            print(len(path), file=f)
            for i in path[-1::-1]:
                print(*i.cordinate, int(i.cost), file=f)

    def pathPrintAStar(self, n: Node):
        path = [n]
        cost = n.h
        while n.parent:
            n = n.parent
            cost += n.h
            path.append(n)

        with open('output.txt', 'w') as f:
            print(int(cost), file=f)
            print(len(path), file=f)
            for i in path[-1::-1]:
                print(*i.cordinate, int(i.h), file=f)

    def solve(self):
        if self.algo == 'BFS':
            self.bfs(self.start, self.goal)
        elif self.algo == 'UCS':
            self.ucs(self.start, self.goal)
        elif self.algo == 'A*':
            self.aStar(self.start, self.goal)

    def dfs(self, start: Node, goal: Node):
        frontier = deque()
        frontier.append(start)
        explored = {start.cordinate}

        while frontier:
            current = frontier.pop()
            if current.cordinate == goal.cordinate:
                self.pathPrint(current)
                return

            for nxt in self.successors(current):
                if nxt.cordinate in explored or nxt in frontier: continue
                explored.add(nxt.cordinate)
                frontier.append(nxt)
        with open('output.txt', 'w') as f:
            print('FAIL', file=f)
        return None

    def ucs(self, start: Node, goal: Node):
        frontier = []
        heapq.heappush(frontier, start)
        explored = {start.cordinate: 0}

        while frontier:
            current = heapq.heappop(frontier)
            if current.cordinate == goal.cordinate:
                temp = current

            for nxt in self.successors(current):
                nxt.g = current.g + nxt.cost
                if nxt.cordinate not in explored or explored[nxt.cordinate] > nxt.g:
                    nxt.h = self.euclideanDistance(nxt, goal)
                    explored[nxt.cordinate] = nxt.g
                    heapq.heappush(frontier, nxt)

        if goal.cordinate in explored:
            self.pathPrint(temp)
            return
        with open('output.txt', 'w') as f:
            print('FAIL', file=f)
        return None

    def aStar(self, start: Node, goal: Node):
        frontier = []
        heapq.heappush(frontier, start)
        explored = {start.cordinate: 0}

        while frontier:
            current = heapq.heappop(frontier)
            if current.cordinate == goal.cordinate:
                self.pathPrint(current)
                return

            for nxt in self.successors(current):
                nxt.g = current.g + nxt.cost
                nxt.h = self.euclideanDistance(nxt, goal)
                f = nxt.g + nxt.h
                if nxt.cordinate not in explored or explored[nxt.cordinate] > f:
                    nxt.h = self.euclideanDistance(nxt, goal)
                    explored[nxt.cordinate] = f
                    heapq.heappush(frontier, nxt)
        with open('output.txt', 'w') as f:
            print('FAIL', file=f)
        return None

    def bfs(self, start: Node, goal: Node):
        frontier = deque()
        frontier.append(start)
        explored = {start.cordinate}

        while frontier:
            current = frontier.popleft()
            if current.cordinate == goal.cordinate:
                self.pathPrint(current)
                return

            for nxt in self.successors(current):
                if nxt.cordinate in explored or nxt in frontier: continue
                explored.add(nxt.cordinate)
                frontier.append(nxt)
        with open('output.txt', 'w') as f:
            print('FAIL', file=f)
        return None


Maze().solve()
