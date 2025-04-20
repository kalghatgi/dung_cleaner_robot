import folium
import geojson

# Load the GeoJSON file
with open("c:\\Users\\cheta\\Desktop\\dung_cleaner_robot\\tests\\test_output.geojson", "r") as f:
    data = geojson.load(f)

# Create a map
m = folium.Map(location=[0, 0], zoom_start=2)

# Add GeoJSON to the map
folium.GeoJson(data).add_to(m)

# Save the map to an HTML file
m.save("map.html")
print("Map saved as map.html")