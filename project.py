import flet as ft  # Import the necessary GUI components from flet
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# Function to parse input and validate data integrity
def parse_input(num_customers_str, vehicle_capacity_str, customer_demands_str, edges_str):
    try:
        # Convert input strings to appropriate data types
        num_customers = int(num_customers_str)
        vehicle_capacity = int(vehicle_capacity_str)
        customer_demands = list(map(int, customer_demands_str.split(',')))

        # Check if the number of customer demands matches num_customers
        if len(customer_demands) != num_customers:
            raise ValueError("The length of customer demands must be equal to the number of customers.")
        
        edges = []
        # Parse the edges string to form a matrix
        for line in edges_str.split(';'):
            row = list(map(int, line.split(',')))
            edges.append(row)
        
        # Validate the edges matrix properties
        if len(edges) != num_customers + 1:
            raise ValueError("The edges matrix must be a square matrix of size (num_customers + 1) x (num_customers + 1).")
        
        for row in edges:
            if len(row) != len(edges):
                raise ValueError("The edges matrix must be a square matrix of size (num_customers + 1) x (num_customers + 1).")
        
        # Check symmetry and diagonal conditions for the edges matrix
        for i in range(len(edges)):
            for j in range(len(edges)):
                if edges[i][j] != edges[j][i]:
                    raise ValueError("The edges matrix must be symmetric.")
                if i == j and edges[i][j] != 0:
                    raise ValueError("The diagonal elements of the edges matrix must be zero.")
                if i != j and edges[i][j] == 0:
                    raise ValueError("The non-diagonal elements of the edges matrix must be non-zero.")
        
        # Return validated input data
        return num_customers, vehicle_capacity, customer_demands, edges
    except ValueError as ex:
        # Raise an error with a clear message if input validation fails
        raise ValueError(f"Invalid input: {str(ex)}")

# Function to create the data model for ORTools
def create_data_model(num_customers, vehicle_capacity, customer_demands, edges):
    data = {}
    data['distance_matrix'] = edges
    data['demands'] = [0] + customer_demands  # Include depot demand as 0
    data['vehicle_capacities'] = [vehicle_capacity] * num_customers  # All vehicles have same capacity
    data['num_vehicles'] = num_customers  # Number of vehicles equals number of customers
    data['depot'] = 0  # Starting and ending point (depot) is index 0
    
    # Validate that demands array length matches num_customers + 1
    assert len(data['demands']) == num_customers + 1, "Customer demands length should be num_customers + 1"

    return data

# Function to print the solution obtained from ORTools
def print_solution(data, manager, routing, solution):
    total_distance = 0
    total_load = 0
    result_text = ""

    trip_number = 1

    # Iterate over each vehicle's route in the solution
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        route_distance = 0
        route_load = 0
        route = []
        previous_index = index
        
        # Traverse each node in the vehicle's route
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data['demands'][node_index]
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            route.append((node_index, route_load, route_distance))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
        
        # Include returning to depot in the route
        node_index = manager.IndexToNode(index)
        route_load += data['demands'][node_index]
        route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        route.append((node_index, route_load, route_distance))

        # Only print routes that visit at least one customer (route length > 2 means it visits at least two nodes)
        if len(route) > 2:
            plan_output = f'Route for trip {trip_number}:\n'
            trip_number += 1
            for i, (node, load, distance) in enumerate(route[:-1]):
                if i == 0:
                    plan_output += f' {node} Load({load}) Distance(0) ->'
                else:
                    plan_output += f' {node} Load({load}) Distance({distance}) ->'
            plan_output += f' {manager.IndexToNode(index)} Load({route[-1][1]}) Distance({route[-1][2]})\n'
            plan_output += f'Distance of the route: {route[-1][2]}m\n'
            plan_output += f'Load of the route: {route[-1][1]}\n\n'
            result_text += plan_output
        
        # Accumulate total distance and load for all routes
        total_distance += route[-1][2] if route else 0
        total_load += route[-1][1] if route else 0

    # Append total distance and load to result_text
    result_text += f'\nTotal distance of all routes: {total_distance}m\n'
    result_text += f'Total load of all routes: {total_load}'
    return result_text

# Function to solve the routing problem using ORTools
def solve_routing_problem(num_customers, vehicle_capacity, customer_demands, edges):
    # Create the data model for ORTools
    data = create_data_model(num_customers, vehicle_capacity, customer_demands, edges)

    # Initialize routing index manager and routing model
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)

    # Define distance callback function
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    # Register distance callback
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Define demand callback function
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    # Register demand callback
    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,
        data['vehicle_capacities'],
        True,
        'Capacity')

    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem using ORTools
    solution = routing.SolveWithParameters(search_parameters)

    # Generate and return the solution text
    if solution:
        result_text = print_solution(data, manager, routing, solution)
    else:
        result_text = "No solution found."

    return result_text

# Main function to set up the GUI and handle user interactions
def main(page: ft.Page):
    page.title = "Vehicle Routing Problem Solver"
    page.vertical_alignment = ft.MainAxisAlignment.CENTER
    page.horizontal_alignment = ft.CrossAxisAlignment.CENTER

    # Define input fields using flet components
    num_customers = ft.TextField(label="Number of Customers", width=200)
    vehicle_capacity = ft.TextField(label="Vehicle Capacity", width=200)
    customer_demands = ft.TextField(label="Customer Demands (comma separated)", width=200)
    edges = ft.TextField(label="Edges (semicolon and comma separated)", width=200, max_lines=10, multiline=True)

    result_text_control = ft.Text("", size=14)  # Text control to display the solution
    result_dialog = ft.AlertDialog(
        title=ft.Text("Solution"),  # Dialog title
        content=ft.Container(
            ft.Column(
                controls=[result_text_control],  # Display the result text in a column
                scroll=ft.ScrollMode.ALWAYS
            ),
            width=400,
            height=300
        ),
        actions=[
            ft.TextButton("Close", on_click=lambda e: (setattr(result_dialog, 'open', False), page.update()))
        ],
        open=False  # Initially dialog is closed
    )

    # Function to handle solve button click event
    def solve_and_display(e):
        try:
            # Parse input values
            num_customers_val, vehicle_capacity_val, customer_demands_val, edges_val = parse_input(
                num_customers.value, vehicle_capacity.value, customer_demands.value, edges.value)
            
            # Solve routing problem and get result text
            result_text = solve_routing_problem(num_customers_val, vehicle_capacity_val, customer_demands_val, edges_val)
            
            # Update result text control and open result dialog
            result_text_control.value = result_text
            result_dialog.open = True
            page.update()
        
        # Handle invalid input exceptions
        except ValueError as ex:
            # Display error message in an alert dialog
            error_dialog = ft.AlertDialog(
                title=ft.Text("Error"),
                content=ft.Text(str(ex)),
                actions=[
                    ft.TextButton("Close", on_click=lambda e: (setattr(error_dialog, 'open', False), page.update()))
                ],
                open=True
            )
            page.dialog = error_dialog
            page.update()

    # Create the solve button and bind it to the solve_and_display function
    solve_button = ft.ElevatedButton(text="Solve", on_click=solve_and_display)

    # Initialize the result dialog
    page.dialog = result_dialog

    # Construct the layout using flet components
    page.add(
        ft.Container(
            content=ft.Column(
                [
                    num_customers,
                    vehicle_capacity,
                    customer_demands,
                    edges,
                    solve_button,
                ],
                alignment=ft.MainAxisAlignment.CENTER,
                spacing=20,
                expand=True,
            ),
        )
    )

# Start the flet application with the main function as the target
ft.app(target=main)
