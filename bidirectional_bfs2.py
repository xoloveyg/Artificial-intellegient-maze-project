class AdjacentNode:
    def __init__(self, vertex):

        self.vertex = vertex
        self.next = None

    def print(self):
        print(self.vertex)


class BidirectionalSearch:
    def __init__(self, vertices):

        # Initialize vertices and
        # graph with vertices
        self.vertices = vertices
        self.graph = [None] * self.vertices

        # Initializing queue for forward
        # and backward search
        self.src_queue = list()
        self.dest_queue = list()

        # Initializing source and
        # destination visited nodes as False
        self.src_visited = [False] * self.vertices
        self.dest_visited = [False] * self.vertices

        # Initializing source and destination
        # parent nodes
        self.src_parent = [None] * self.vertices
        self.dest_parent = [None] * self.vertices

    # Function for adding undirected edge
    def add_edge(self, src, dest):

        # Add edges to graph

        # Add source to destination
        node = AdjacentNode(dest)
        node.next = self.graph[src]
        self.graph[src] = node

        # Since graph is undirected add
        # destination to source
        node = AdjacentNode(src)
        node.next = self.graph[dest]
        self.graph[dest] = node

    # Function for Breadth First Search
    def bfs(self, direction="forward"):

        if direction == "forward":

            # BFS in forward direction
            current = self.src_queue.pop(0)
            connected_node = self.graph[current]

            while connected_node:
                vertex = connected_node.vertex

                if not self.src_visited[vertex]:
                    self.src_queue.append(vertex)
                    self.src_visited[vertex] = True
                    self.src_parent[vertex] = current

                connected_node = connected_node.next
        else:

            # BFS in backward direction
            current = self.dest_queue.pop(0)
            connected_node = self.graph[current]

            while connected_node:
                vertex = connected_node.vertex

                if not self.dest_visited[vertex]:
                    self.dest_queue.append(vertex)
                    self.dest_visited[vertex] = True
                    self.dest_parent[vertex] = current

                connected_node = connected_node.next

    # Check for intersecting vertex
    def is_intersecting(self):

        # Returns intersecting node
        # if present else -1
        for i in range(self.vertices):
            if self.src_visited[i] and self.dest_visited[i]:
                return i

        return -1

    # Print the path from source to target
    def print_path(self, intersecting_node, src, dest):

        # Print final path from
        # source to destination
        path = list()
        path.append(intersecting_node)
        i = intersecting_node

        while i != src:
            path.append(self.src_parent[i])
            i = self.src_parent[i]

        path = path[::-1]
        i = intersecting_node

        while i != dest:
            path.append(self.dest_parent[i])
            i = self.dest_parent[i]

        print("*****Path*****")
        path = list(map(str, path))

        print(" ".join(path))

    # Function for bidirectional searching
    def bidirectional_search(self, src, dest):

        # Add source to queue and mark
        # visited as True and add its
        # parent as -1
        self.src_queue.append(src)
        self.src_visited[src] = True
        self.src_parent[src] = -1

        # Add destination to queue and
        # mark visited as True and add
        # its parent as -1
        self.dest_queue.append(dest)
        self.dest_visited[dest] = True
        self.dest_parent[dest] = -1

        while self.src_queue and self.dest_queue:

            # BFS in forward direction from
            # Source Vertex
            self.bfs(direction="forward")

            # BFS in reverse direction
            # from Destination Vertex
            self.bfs(direction="backward")

            # Check for intersecting vertex
            intersecting_node = self.is_intersecting()

            # If intersecting vertex exists
            # then path from source to
            # destination exists
            if intersecting_node != -1:
                # print(f"Path exists between {src} and {dest}")
                # print(f"Intersection at : {intersecting_node}")
                # self.print_path(intersecting_node,
                #                src, dest)
                self.print_path_as_matrix(intersecting_node, src, dest)
                return 1
                #exit(0)
        return -1

    def print_path_as_matrix(self, intersecting_node, src, dest):
        path = list()
        path.append(intersecting_node)
        i = intersecting_node

        while i != src:
            path.append(self.src_parent[i])
            i = self.src_parent[i]

        path = path[::-1]
        i = intersecting_node

        while i != dest:
            path.append(self.dest_parent[i])
            i = self.dest_parent[i]

        for each in path:
            print(
                "[" + str(int(each / 5) + 1) + "," + str(int(each % 5) + 1) + "]",
                end="",
            )
        print()


def uniform_cost_search(goal, start):
    # minimum cost upto
    # goal state from starting
    global u_graph, cost, path_uniform_cost
    answer = []

    # create a priority queue
    queue = []

    # set the answer vector to max value
    for i in range(len(goal)):
        answer.append(10 ** 8)

    # insert the starting index
    queue.append([0, start])

    # map to store visited node
    visited = {}

    # count
    count = 0

    # while the queue is not empty
    while len(queue) > 0:

        # get the top element of the
        queue = sorted(queue)
        p = queue[-1]

        # pop the element
        del queue[-1]

        # get the original value
        p[0] *= -1

        # check if the element is part of
        # the goal list
        if p[1] in goal:

            # get the position
            index = goal.index(p[1])

            # if a new goal is reached
            if answer[index] == 10 ** 8:
                count += 1

            # if the cost is less
            if answer[index] > p[0]:
                answer[index] = p[0]

            # pop the element
            del queue[-1]

            queue = sorted(queue)
            if count == len(goal):
                return answer

        # check for the non visited nodes
        # which are adjacent to present node

        if p[1] not in visited:
            for i in range(len(u_graph[p[1]])):
                # value is multiplied by -1 so that
                # least priority is at the top
                queue.append(
                    [(p[0] + cost[(p[1], u_graph[p[1]][i])]) * -1, u_graph[p[1]][i]]
                )
        # mark as visited
        visited[p[1]] = 1

    return answer


def print(src, dest):
    print(src, "---->", dest)


if __name__ == "__main__":
    matrix = []
    for i in range(8):
        row = [int(each) for each in input().split(" ")]
        matrix.append(row)

    n = 25
    src = 0
    dest = 24
    graph = BidirectionalSearch(n)

    for i in range(4):
        for j in range(5):
            if matrix[i][j] == 0:
                if i + 1 < 4:
                    graph.add_edge(i * 5 + j, (i + 1) * 5 + j)
                    graph.add_edge((i + 1) * 5 + j, i * 5 + j)
                    # print(i * 5 + j, (i + 1) * 5 + j)
                    # print((i + 1) * 5 + j, i * 5 + j)
                if i == 3:
                    graph.add_edge(i * 5 + j, (i + 1) * 5 + j)
                    graph.add_edge((i + 1) * 5 + j, i * 5 + j)
                    # print(i * 5 + j, (i + 1) * 5 + j)
                    # print((i + 1) * 5 + j, i * 5 + j)

    for i in range(4, 8, 1):
        for j in range(5):
            if matrix[i][j] == 0:
                graph.add_edge(j * 5 + (i - 4), j * 5 + (i - 4) + 1)
                graph.add_edge(j * 5 + (i - 4) + 1, j * 5 + (i - 4))
                # print(j * 5 + (i - 4), j * 5 + (i - 4) + 1)
                # print(j * 5 + (i - 4) + 1, j * 5 + (i - 4))
    cost = {}
    u_graph = [[] for i in range(25)]
    path_uniform_cost = []

    for i in range(5):
        for j in range(4):
            cost[(i * 5 + j, i * 5 + j + 1)] = 1
            cost[(i * 5 + j + 1, i * 5 + j)] = 1

    for i in range(4):
        for j in range(5):
            cost[(i * 5 + j, (i + 1) * 5 + j)] = 3
            cost[((i + 1) * 5 + j, i * 5 + j)] = 3

    for i in range(4):
        for j in range(5):
            if matrix[i][j] == 0:
                if i + 1 < 4:
                    u_graph[i * 5 + j].append((i + 1) * 5 + j)
                    u_graph[(i + 1) * 5 + j].append(i * 5 + j)
                    # print(i * 5 + j, (i + 1) * 5 + j)
                    # print((i + 1) * 5 + j, i * 5 + j)
                if i == 3:
                    u_graph[i * 5 + j].append((i + 1) * 5 + j)
                    u_graph[(i + 1) * 5 + j].append(i * 5 + j)
                    # print(i * 5 + j, (i + 1) * 5 + j)
                    # print((i + 1) * 5 + j, i * 5 + j)

    for i in range(4, 8, 1):
        for j in range(5):
            if matrix[i][j] == 0:
                u_graph[j * 5 + (i - 4)].append(j * 5 + (i - 4) + 1)
                u_graph[j * 5 + (i - 4) + 1].append(j * 5 + (i - 4))
                # print(j * 5 + (i - 4), j * 5 + (i - 4) + 1)
                # print(j * 5 + (i - 4) + 1, j * 5 + (i - 4))

    print("Output(UCS):")
    out = graph.bidirectional_search(src, dest)
    print(uniform_cost_search([dest], src)[0])
    print("Output(Bidir):")
    out = graph.bidirectional_search(src, dest)
