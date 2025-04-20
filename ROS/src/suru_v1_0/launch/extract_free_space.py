import cv2
import numpy as np
import yaml
import geojson
import argparse
import os

# Load YAML metadata
def load_yaml(yaml_path):
    with open(yaml_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

# Convert map to binary image
def load_map(map_path):
    yaml_path = os.path.splitext(map_path)[0] + ".yaml"
    pgm_path = os.path.splitext(map_path)[0] + ".pgm"
    
    yaml_data = load_yaml(yaml_path)
    image = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    resolution = yaml_data['resolution']
    origin = yaml_data['origin']
    
    # Threshold the image (assuming free space is white and obstacles are black)
    _, binary_map = cv2.threshold(image, 250, 255, cv2.THRESH_BINARY)
    return binary_map, resolution, origin

# Extract free space boundary as polygon
def extract_polygon(binary_map, resolution, origin):
    contours, _ = cv2.findContours(binary_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    polygons = []
    
    for contour in contours:
        polygon = []
        for point in contour:
            x, y = point[0]
            # Convert pixel coordinates to real-world coordinates
            real_x = origin[0] + (x * resolution)
            real_y = origin[1] + ((binary_map.shape[0] - y) * resolution)
            polygon.append((real_x, real_y))
        polygons.append(polygon)
    return polygons

# Save polygon as GeoJSON
def save_geojson(polygons, output_file):
    features = []
    for polygon in polygons:
        features.append(geojson.Feature(geometry=geojson.Polygon([polygon])))
    feature_collection = geojson.FeatureCollection(features)
    with open(output_file, 'w') as f:
        geojson.dump(feature_collection, f)
    print(f"Saved free-space polygon to {output_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, help="Path to the map file (without extension)")
    parser.add_argument("--output", required=True, help="Output GeoJSON file")
    args = parser.parse_args()

    binary_map, resolution, origin = load_map(args.map)
    polygons = extract_polygon(binary_map, resolution, origin)
    save_geojson(polygons, args.output)
