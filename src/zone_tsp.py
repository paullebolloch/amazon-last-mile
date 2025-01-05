# Importing the libraries
import json
import pandas as pd
import numpy as np

# TSP
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp



def get_average_distances_between_zones(df, index):

    # Étape 1 : Extraire la correspondance double-lettre -> zone_id
    stops_mapping = df['stops'].iloc[index]
    stop_to_zone = {key: value['zone_id'] for key, value in stops_mapping.items()}

    # Étape 2 : Réorganiser les distances en format tabulaire
    distance_rows = []

    for col in df.columns[9:]:  # Que les colonnes de distances
        distance_dict = df[col].iloc[0]  # Accéder au dictionnaire dans la colonne
        
        # Vérifie si la cellule contient un dictionnaire
        if isinstance(distance_dict, dict):
            for target, distance in distance_dict.items():

                distance_rows.append({
                        'from': col,
                        'to': target,
                        'distance': distance
                    })

    distances_df = pd.DataFrame(distance_rows)

    # Ajouter les zones associées à chaque double lettre
    distances_df['from_zone'] = distances_df['from'].map(stop_to_zone)
    distances_df['to_zone'] = distances_df['to'].map(stop_to_zone)

    # Étape 3 : Calculer la moyenne des distances entre zones
    average_distances = distances_df.groupby(['from_zone', 'to_zone'])['distance'].mean().reset_index()
    average_distances.rename(columns={'distance': 'avg_distance'}, inplace=True)

    # Etape 4 : Mettre sous forme de dictionnaire
    zone_distances = average_distances.set_index(['from_zone', 'to_zone']).to_dict()['avg_distance']

    return stop_to_zone , zone_distances, average_distances



# Créer une matrice de distances entre les zones

def create_distance_matrix(stop_to_zone, zone_distances, average_distances):
    # Étape 1 : Créer une liste des zones uniques
    zones = list(set(stop_to_zone.values()))
    zone_to_index = {zone: idx for idx, zone in enumerate(zones)}  # Mapping zone -> index pour la matrice

    # Étape 2 : Construire la matrice de distances
    num_zones = len(zones)
    distance_matrix = [[10**6] * num_zones for _ in range(num_zones)]  # Initialisation avec une grande valeur

    # Remplir la matrice avec les distances moyennes
    for _, row in average_distances.iterrows():
        from_idx = zone_to_index[row["from_zone"]]
        to_idx = zone_to_index[row["to_zone"]]
        distance_matrix[from_idx][to_idx] = row["avg_distance"]
        distance_matrix[to_idx][from_idx] = row["avg_distance"]  # Symétrique

    return distance_matrix



# Étape 3 : Configurer OR-Tools pour résoudre le TSP
def solve_tsp_with_ortools(distance_matrix):
    """Résout le problème TSP avec OR-Tools."""
    num_locations = len(distance_matrix)
    manager = pywrapcp.RoutingIndexManager(num_locations, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    # Définir la fonction de coût
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Stratégie de recherche
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Résoudre
    solution = routing.SolveWithParameters(search_parameters)
    if not solution:
        print("Aucune solution trouvée.")
        return None

    # Affichage de la solution
    route = []
    index = routing.Start(0)
    while not routing.IsEnd(index):
        route.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))
    route.append(manager.IndexToNode(index))
    return route



def insert_tsp_zone_sequence(df, optimal_route_indices, stop_to_zone, index):
    zones = list(set(stop_to_zone.values()))
    # Conversion des indices de la route optimale en zones
    if optimal_route_indices:
        optimal_route_zones = [zones[idx] for idx in optimal_route_indices if pd.notna(zones[idx])]
        route_sequence_dict = {zone: i for i, zone in enumerate(optimal_route_zones)}
        # Ajouter route_sequence_dict à route_data
        route_sequence_json = json.dumps(route_sequence_dict)
        df.loc[index, 'tsp_zone_sequence'] = route_sequence_json