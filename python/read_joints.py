import json

# Define the path to your text file
file_path = 'test.txt'

# Initialize an empty dictionary to store the results
result_dict = {}

# Open the file and read its contents
with open(file_path, 'r') as file:
    # Iterate through each line in the file
    for row_number, line in enumerate(file, start=1):
        # Split the line into elements by comma
        elements = line.strip().split(',')
        
        # Extract the last 16 elements
        last_16_elements = elements[14:30] # elements[-16:]
        
        # Add the last 16 elements to the dictionary with the row number as the key
        result_dict[row_number] = last_16_elements

# Convert the dictionary to a JSON string
result_json = json.dumps(result_dict, indent=4)

# Print the JSON string
print(result_json)

# Optionally, save the JSON string to a file
with open('test.json', 'w') as json_file:
    json_file.write(result_json)
