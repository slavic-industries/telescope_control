import requests


def get_stellarium_object_data(object_name):
    # Stellarium Remote Control URL (assuming it runs on the default localhost and port)
    url = 'http://localhost:8090/api/objects/info'
    params = {
        'name': object_name,
        'format': 'json'
    }

    try:
        # Send a GET request to the Stellarium API
        response = requests.get(url, params=params)
        response.raise_for_status()  # Raise an error for bad status codes
        object_data = response.json()

        # Print the object data
        print("Object Data:")
        for key, value in object_data.items():
            print(f"{key}: {value}")

    except requests.exceptions.RequestException as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":
    # Example object name, e.g., "Mars"
    object_name = "Mars"
    get_stellarium_object_data(object_name)
