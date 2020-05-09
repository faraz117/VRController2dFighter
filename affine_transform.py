# Read and display points from the templates file
# read json file from templates folder
import json
import numpy as np

def slice_per(source, chunck_size):
    return [source[i:i+chunck_size] for i in range(0,len(source),chunck_size)]

def get_features_from_file(filename):
    with open(filename) as json_file:
        data = json.load(json_file)
        for p in data['people']:
            key_points = p['pose_keypoints_2d']
            print(key_points)
            print('')
            print(len(key_points))
    # slice the list into pair of 3s
    body_list = slice_per(key_points,3)

    # use body_list when you need confidence values too
    body_list_without_confidence_value = body_list

    for part in body_list_without_confidence_value:
        part.pop()

    return body_list_without_confidence_value

# Shout out to https://stackoverflow.com/a/20555267/8770351
# 2D array containing the 18 corresponding feature points
model_features = np.array(get_features_from_file('templates//json//000000000151_keypoints.json'))
input_features = np.array(get_features_from_file('templates//json//000000000187_keypoints.json'))

# In order to solve the augmented matrix (incl translation),
# it's required all vectors are augmented with a "1" at the end
# -> Pad the features with ones, so that our transformation can do translations too
pad = lambda x: np.hstack([x, np.ones((x.shape[0], 1))]) 
unpad = lambda x: x[:, :-1]

# Pad to [[ x y 1] , [x y 1]]
Y = pad(model_features)
X = pad(input_features)

# Solve the least squares problem X * A = Y
# and find the affine transformation matrix A. 
A, res, rank, s = np.linalg.lstsq(X, Y)
A[np.abs(A) < 1e-10] = 0  # set really small values to zero

transform = lambda x: unpad(np.dot(pad(x), A))

#Image of input pose onto model pose
input_transform = transform(input_features)

print(input_transform)

dist = np.linalg.norm(input_transform-model_features)
print(dist)
