import cv2
import numpy as np
import yaml
import os

# Configuration variables
MARKER_SIZE = 200  # Size of the marker in pixels
NUM_MARKERS = 10   # Number of markers to generate
ARUCO_DICT = cv2.aruco.DICT_6X6_250  # 6x6 dictionary
OUTPUT_DIR = "aruco_markers"  # Directory to save markers and YAML files

def generate_aruco_markers(marker_size=MARKER_SIZE, num_markers=NUM_MARKERS, 
                          aruco_dict=ARUCO_DICT, output_dir=OUTPUT_DIR):
    """
    Generate ArUco markers with corresponding YAML configuration files.
    
    Args:
        marker_size: Size of the marker in pixels
        num_markers: Number of markers to generate
        aruco_dict: ArUco dictionary to use
        output_dir: Directory to save output files
    """
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Get the ArUco dictionary
    aruco_dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict)
    
    print(f"Generating {num_markers} ArUco markers...")
    
    for marker_id in range(num_markers):
        # Generate the marker image
        marker_image = cv2.aruco.generateImageMarker(aruco_dictionary, marker_id, marker_size)
        
        # Save the marker image
        image_filename = os.path.join(output_dir, f"marker_{marker_id}.png")
        cv2.imwrite(image_filename, marker_image)
        
        # Create YAML configuration
        yaml_data = {
            f"marker_{marker_id}": {
                "aruco_id": marker_id,
                "position": [0.0, 0.0, 0.0],  # Default position
                "orientation": [0.0, 0.0, 0.0]  # Default orientation
            }
        }
        
        # Save YAML file
        yaml_filename = os.path.join(output_dir, f"marker_{marker_id}.yaml")
        with open(yaml_filename, 'w') as yaml_file:
            yaml.dump(yaml_data, yaml_file, default_flow_style=False, sort_keys=False)
        
        print(f"Generated marker {marker_id}: {image_filename} and {yaml_filename}")
    
    print(f"\nAll markers generated successfully in '{output_dir}' directory!")
    print(f"Marker size: {marker_size}x{marker_size} pixels")
    print(f"Dictionary: 6x6")

if __name__ == "__main__":
    # Generate markers with default settings
    generate_aruco_markers()
    
    # Example: Generate with custom settings
    # generate_aruco_markers(marker_size=300, num_markers=5, output_dir="custom_markers")
