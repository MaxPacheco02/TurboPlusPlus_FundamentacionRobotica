import sys
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
import speech_recognition as sr
import subprocess

SPACE = 50
LAST_WORD_ENDING = 0 # To calculate where the letter is going to end
HEIGHT = 300
phrase = "HEISE"

# Arguments for running with speech recognition or without
if __name__ == "__main__":
    num1 = sys.argv[1]

if (num1 == 1):
    recognized = False
    while not recognized:
        r= sr.Recognizer()
        with sr.Microphone() as source:
                print('Say my name ...')
                r.pause_threshold = 1
                r.adjust_for_ambient_noise(source)
                audio = r.listen(source)
        try:
                    phrase = (r.recognize_google(audio).lower()).upper()
                    print('You said: ' + phrase + '\n')
                    recognized = True

        except sr.UnknownValueError:
                print('Your last command couldn\'t be heard')

# Function to calculate distance between two points
def dist(p0, p1):
    return ((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)**(1/2)

# Function to calculate a linspace with n points between p0 and p1
def between(p0, p1, n):
    rx = np.linspace(p0[0],p1[0],n)
    ry = np.linspace(p0[1],p1[1],n)
    r = np.append([rx], [ry], axis=0)
    return np.transpose(r)[1:-1,:]

# Function to format each letter
def format(arr, d):
    # Make the first value of the Array the nearest coordinate from the origin
    min_i = 0
    min_dist = dist([0,0],arr[min_i])
    furthest_lateral_dist = 0 # To calculate where the letter is going to end
    for i in range(len(arr)):
        if arr[i,0] > furthest_lateral_dist:
            furthest_lateral_dist = arr[i,0]
        di = dist([0,0],arr[i])
        if di < min_dist:
            min_dist = di
            min_i = i
    arr = np.append(arr[min_i:],arr[:min_i],axis=0)
    arr = np.append([np.array([0,0])], arr, axis=0) # Add a starting point for the letter at 0,0

    # Translate all the points so that it starts to draw the closes to the origin
    base_x = min(arr[1:,0])
    base_y = min(arr[1:,1])
    arr[0,0] = arr[0,0] + d
    for i in range(1,len(arr)):
        arr[i,0] = arr[i,0] - base_x + d
        arr[i,1] = arr[i,1] - base_y

    # Multiply the height by a relation to make all the letters the same height
    height_relation = HEIGHT / max(arr[1:,1])
    for i in range(1,len(arr)):
        arr[i,1] = arr[i,1]*height_relation

    # Include the spacing between letters and current translation
    furthest_lateral_dist = furthest_lateral_dist + SPACE + d
    return arr, furthest_lateral_dist

# Read the JSON file with the letters coordinates
with open('letter_coords.json') as file:
    letters_obj = json.load(file)

# Load each letter
first_point_list = [0]
space_count = 0
lett = np.array(letters_obj[phrase[0]])
prev_len = len(lett)
word, LAST_WORD_ENDING = format(lett, LAST_WORD_ENDING) # Load the first letter
for i in range(1,len(phrase)): # Load the rest of the letters
    if phrase[i] == " ":
        LAST_WORD_ENDING = LAST_WORD_ENDING + 3 * SPACE
    else:
        lett = np.array(letters_obj[phrase[i]])
        letter_next, LAST_WORD_ENDING = format(lett, LAST_WORD_ENDING)
        word = np.append(word, letter_next, axis = 0)
        first_point_list.append(prev_len + first_point_list[-1] + 1) # Every letter 'starts' at 0,0
        prev_len = len(lett)

# Don't draw the beginning of each letter (it's 0,0)
filled_list = np.ones(len(word)) # List that defines whether a coord should be drawn or not
for i in range(len(word) - 1):
    if(i in first_point_list):
        filled_list_t = np.append(filled_list[0:i], np.zeros(1), axis=0)
        filled_list = np.append(filled_list_t, filled_list[i+1:], axis=0)

# Check if additional coordinates are required
dist_list = []
i = 0
while(i < len(word) - 1):
    d = dist(word[i], word[i+1])
    dist_list.append(d)
    if(d > 11): # If the next point is further than that this...
        arr_to_append = between(word[i], word[i+1], math.ceil(d/10)+1)
        word_t = np.append(word[0:i+1], arr_to_append, axis=0)
        word = np.append(word_t, word[i+1:], axis=0) # Add the linspace between the gap
        filled_list_t = np.append(filled_list[0:i+1], np.zeros(len(arr_to_append)), axis=0)
        filled_list = np.append(filled_list_t, filled_list[i+1:], axis=0) # Update the Fill List
    i=i+1

# Adding z-axis
word = np.append(word, np.transpose([np.zeros(len(word))]), axis=1)
# Append the 'Fill' List to the Coordinate Array
word = np.append(word, np.transpose([filled_list]), axis=1)

# Function for rotating in axis by angle
# 2-x 1-y 0-z
def rotate_coordinates(coordinates, axis, angle, translation, resize):
    # Extracting coordinates without flag
    coords = coordinates[:,:3]

    # Rotation Matrix
    rotation_matrix = np.eye(3)
    rotation_matrix[axis, axis] = np.cos(angle)
    rotation_matrix[axis, (axis+1) % 3] = -np.sin(angle)
    rotation_matrix[(axis+1) % 3, axis] = np.sin(angle)
    rotation_matrix[(axis+1) % 3, (axis+1) % 3] = np.cos(angle)
    
    # Resizing coordinates
    if(resize != 1):
        coords = coords * resize

    # Rotate coordinates
    if(angle != 0):
        coords = np.dot(coords, rotation_matrix)

    # Translated coordinates
    if(math.sqrt(sum(pow(element, 2) for element in translation)) != 0):
        coords = coords + translation[np.newaxis, :]


    # Return rotated with flag
    return np.hstack((coords, coordinates[:, 3:]))

# Only for rotating a vector
def rotate_matrix(matrix, axis, angle):

    # Rotation Matrix
    rotation_matrix = np.eye(3)
    rotation_matrix[axis, axis] = np.cos(angle)
    rotation_matrix[axis, (axis+1) % 3] = -np.sin(angle)
    rotation_matrix[(axis+1) % 3, axis] = np.sin(angle)
    rotation_matrix[(axis+1) % 3, (axis+1) % 3] = np.cos(angle)

    matrix = np.dot(matrix, rotation_matrix)

    return matrix 

# Rotating coordinates
# 2-x 1-y 0-z 
angle = 90 #For choosing face of cube
axis = 1  #For choosing face of cube
boxAngleY = -30
boxAxisY = 1
boxAngleX = -30
boxAxisX = 2
boxAngleZ = -30
boxAxisZ = 0

# Offset of box
box_offset = np.array([55,55,0])
#box_offset = rotate_matrix(box_offset, boxAxisY, (boxAngleY)*(np.pi)/180)
#box_offset = rotate_matrix(box_offset, boxAxisX, (boxAngleX)*(np.pi)/180)

# Translation of coordinates
zerotranslation = np.array([0,0,0])
translation_top = np.array([-25,0,-45])
translation_bottom = np.array([-25,0,25])
translation_ff = np.array([-25,0,35])
translation_bf = np.array([-25,-35,0])
translation_lf = np.array([-35,0,-35])
translation_rf = np.array([35,0,-35])

# Rotating translation vetor
rotated_translation = rotate_matrix(translation_ff, axis, (angle)*(np.pi)/180)
box_translation_y = rotate_matrix(rotated_translation, boxAxisY, (boxAngleY)*(np.pi)/180)
box_translation_x = rotate_matrix(rotated_translation, boxAxisX, (boxAngleX)*(np.pi)/180)

# First rotate (Word coordinates)
rotated_word = rotate_coordinates(word, axis, (angle)*(np.pi)/180, zerotranslation, .03) #Choosing face of cube
box_rotated_word = rotate_coordinates(rotated_word, boxAxisY, (boxAngleY)*(np.pi)/180, zerotranslation, 1) #Angle of boxY
box_rotated_word = rotate_coordinates(box_rotated_word, boxAxisX, (boxAngleX)*(np.pi)/180, zerotranslation, 1) #Angle of boxX
# Second translate
rotated_word = rotate_coordinates(box_rotated_word, axis, 0, box_translation_x + box_offset, 1) 

#Base case
#word = rotate_coordinates(word , axis, (angle)*(np.pi)/180, zerotranslation, 1)
#translation_ff = rotate_matrix(translation_ff, axis, (angle)*(np.pi)/180)
#rotated_word = rotate_coordinates(word, 1, 0, translation_ff, 0.03) 

# # Plot 2D
# plt.plot(word[:,0], word[:,1], 'ro') # Draw the whole trajectory
# for i in range(len(word)):
#     if(word[i,2] == 1):
#         plt.plot(word[i,0], word[i,1], 'bo') # Draw 'added' points
# plt.grid(True)
# plt.show()

# # Plot 3D
# fig = plt.figure(figsize=(12, 12))
# ax = fig.add_subplot(projection='3d')
# ax.scatter(rotated_word[:,0], rotated_word[:,1], rotated_word[:,2], c='b')
# plt.show()

## Converting array into json file
data2json_array = []
for i in range(len(word)):
    row_data = [rotated_word[i][0],rotated_word[i][1],rotated_word[i][2],rotated_word[i][3]]
    data2json_array.append(row_data)

json_string = json.dumps(data2json_array)

# Relative path. Only need to change in processing
with open("./Robot/new_coordinates.json", "w") as fn:
    fn.write(json_string)


process = subprocess.run(["bash", "run_sketch.sh"])

sys.exit()
