import json
import random
import math

def generate_random_graph(num_nodes, num_edges, graph_id="sample_graph"):
    pois_list = ["restaurant" , "petrol_station" , "hospital" , "atm" , "hotel" , "pharmacy"]
    road_types = ["primary","secondary","tertiary","expressway","local"]

    nodes = []
    edges = []

    # Generate nodes
    for i in range(num_nodes):
        lat = 19.00 + random.random() * 0.2
        lon = 72.80 + random.random() * 0.2
        pois = random.sample(pois_list, random.randint(0, 2))
        nodes.append({
            "id": i,
            "lat": round(lat, 6),
            "lon": round(lon, 6),
            "pois": pois
        })

    # Generate edges
    for i in range(num_edges):
        u = random.randint(0, num_nodes - 1)
        v = random.randint(0, num_nodes - 1)
        while v == u:
            v = random.randint(0, num_nodes - 1)

        length = random.uniform(50, 1000)
        avg_time = length / random.uniform(15, 60)  # derived roughly by speed

        edge = {
            "id": 1000 + i,
            "u": u,
            "v": v,
            "length": round(length, 2),
            "average_time": round(avg_time, 2),
            "speed_profile": [round(random.uniform(20, 80), 1) for _ in range(96)],
            "oneway": random.choice([True, False]),
            "road_type": random.choice(road_types)
        }
        edges.append(edge)

    graph = {
        "meta": {
            "id": graph_id,
            "nodes": num_nodes,
            "description": "Auto-generated map"
        },
        "nodes": nodes,
        "edges": edges
    }

    with open("graph.json", "w") as f:
        json.dump(graph, f, indent=4)

    print("✅ graph.json generated successfully.")
    return graph


def generate_queries(num_queries=10, graph=None, qid="qset1"):
    events = []
    node_ids = [n["id"] for n in graph["nodes"]]
    edge_ids = [e["id"] for e in graph["edges"]]

    for i in range(num_queries):
        qtype = random.choice(["shortest_path", "knn", "remove_edge", "modify_edge"])

        if qtype == "remove_edge":
            events.append({
                "id": i,
                "type": "remove_edge",
                "edge_id": random.choice(edge_ids)
            })

        elif qtype == "modify_edge":
            events.append({
                "id": i,
                "type": "modify_edge",
                "edge_id": random.choice(edge_ids),
                "patch": {"length": round(random.uniform(50, 150), 2)}
            })

        elif qtype == "shortest_path":
            source = random.choice(node_ids)
            target = random.choice(node_ids)
            while target == source:
                target = random.choice(node_ids)

            event = {
                "type": "shortest_path",
                "id": i,
                "source": source,
                "target": target,
                "mode": random.choice(["time", "distance"])
            }

            # Optional constraints
            if random.choice([True, False]):
                event["constraints"] = {}
                if random.choice([True, False]):
                    event["constraints"]["forbidden_nodes"] = random.sample(node_ids, min(2, len(node_ids)))
                if random.choice([True, False]):
                    event["constraints"]["forbidden_road_types"] = random.sample(["primary", "secondary", "residential"], 1)

            events.append(event)

        elif qtype == "knn":
            events.append({
                "type": "knn",
                "id": i,
                "poi": random.choice(["Restaurant", "Hospital", "School"]),
                "query_point": {
                    "lat": 19.05,
                    "lon": 72.88
                },
                "k": random.randint(3, 7),
                "metric": random.choice(["shortest_path", "euclidean"])
            })

    queries = {"meta": {"id": qid}, "events": events}

    with open("queries.json", "w") as f:
        json.dump(queries, f, indent=4)

    print("✅ queries.json generated successfully.")


if __name__ == "__main__":
    n = int(input("Enter number of nodes: "))
    m = int(input("Enter number of edges: "))
    q = int(input("Enter number of queries: "))

    graph = generate_random_graph(n, m)
    generate_queries(q, graph)
