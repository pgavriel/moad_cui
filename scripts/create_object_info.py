import os
from os.path import join
import json
from datetime import datetime

def create_template_json(directory,object_name=""):
    # Check if the directory exists
    if not os.path.exists(directory):
        print(f"Directory '{directory}' does not exist.")
        return
    
    # Define the template JSON data
    template_data = {
        "object_name": object_name,
        "scan_date": datetime.now().strftime("%Y-%m-%d"),  # Default to current date
        "scan_location": "NERVE @ UMass Lowell",
        "dimensions": {
            "unit": "cm",
            "width": 0.0,   # X
            "depth": 0.0,   # Y
            "height": 0.0   # Z
        },
        "weight": {
            "unit": "kg",  # Default weight in kilograms
            "weight_min": 0.0,
            "weight_max": 0.0
        },
        "description": "",
        "tags": "",
        "poses": [
            {
                "name": "",
                "description": "",
                "transform": {
                    "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            }
        ]
    }
    
    # File path for the JSON file
    file_path = join(directory, "object_info.json")
    
    # Write the template JSON to the file
    with open(file_path, "w") as json_file:
        json.dump(template_data, json_file, indent=4)
    
    print(f"Template JSON file created at: {file_path}")

# Example usage:
if __name__ == "__main__":
    root_dir = "/home/csrobot/data-mount"
    target_object = "a3_mustard"  # Replace with the desired directory
    target_path = join(root_dir,target_object)


    if os.path.exists(target_path):
        if not os.path.isfile(join(target_path,"object_info.json")):
            create_template_json(target_path,target_object)
            print(f"Created object_info.json for {target_object}: {target_path}")
        else:
            print(f"ERROR: object_info.json for {target_object} already exists. Not created.")
    else:
        print(f"Target path does not exist: {target_path}")
        
