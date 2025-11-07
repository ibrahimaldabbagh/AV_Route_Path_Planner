import networkx as nx
import matplotlib.pyplot as plt

def read_map_file(filename):
    nodes = []
    edges = []
    with open(filename, 'r') as file:
        lines = file.readlines()
        section = None
        for line in lines:
            line = line.strip()
            if not line:
                continue
            if line == "NODES":
                section = "NODES"
                continue
            elif line == "EDGES":
                section = "EDGES"
                continue
            elif line == "PARKING_SPOTS":
                section = "PARKING_SPOTS"
                continue
            parts = line.replace(',', ' ').split()
            if section == "NODES":
                x, y, node_id = map(float, parts)
                nodes.append((node_id, (x, y)))  # Store node_id and coordinates
            elif section == "EDGES":
                node_id, neighbor_id, direction = int(parts[0]), int(parts[1]), parts[2]
                edges.append((node_id, neighbor_id, direction))
    return nodes, edges

def draw_graph(nodes, edges):
    G = nx.DiGraph()
    pos = {node_id: coords for node_id, coords in nodes}  # Use dictionary comprehension for positions

    for node_id, neighbor_id, direction in edges:
        color = 'black'
        if direction == 'E':
            color = 'red'
        elif direction == 'W':
            color = 'blue'
        elif direction == 'N':
            color = 'green'
        elif direction == 'S':
            color = 'purple'
        G.add_edge(node_id, neighbor_id, color=color)

    plt.figure(figsize=(12, 12))
    nx.draw(G, pos, with_labels=True, node_size=700, node_color='lightblue', edge_color=[data['color'] for _, _, data in G.edges(data=True)],
            arrows=True, arrowsize=20, font_weight='bold', font_color='darkred', font_size=12)

    # Create a legend for the colors, positioned on the upper right
    labels = {'East (← E)': 'red', 'West (→  W)': 'blue', 'North (↑ N)': 'green', 'South (↓ S)': 'purple'}
    for label, color in labels.items():
        plt.plot([], [], color=color, label=label)
    plt.legend(loc='upper right', title='Cardinal Directions\n(West is Yaw = 0)')

    plt.title("Graph with Cardinal Directions")
    plt.axis('off')  # Hide the axis
    plt.show()

if __name__ == "__main__":
    filename = "/home/ibrahim/git_ws/src/adapt_roucomp/config/map_cardinal.txt"
    nodes, edges = read_map_file(filename)
    draw_graph(nodes, edges)
