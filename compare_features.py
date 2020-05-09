# Split features in three parts
(model_face, model_torso, model_legs) = split_in_face_legs_torso(model_features)
(input_face, input_torso, input_legs) = split_in_face_legs_torso(input_features)

# 3x3 the same; not the best code design I know ... 
#   But the illustrative factor prevails here

# Solve the least-sqaures problem and find the transformation matrix 
# and the corresponding image of input
(input_transformed_face,  A_face)   = affine_transformation(model_face,input_face)
(input_transformed_torso, A_torso)  = affine_transformation(model_torso,input_torso)
(input_transformed_legs,  A_legs)   = affine_transformation(model_legs,input_legs)

(max_distance_face, rotation_face)   = max_distance_and_rotation(model_face, 
                                                                 input_transformed_face, 
                                                                 A_face)
(max_distance_torso, rotation_torso) = max_distance_and_rotation(model_torso, 
                                                                 input_transformed_torso, 
                                                                 A_torso)
(max_distance_legs, rotation_legs)   = max_distance_and_rotation(model_legs, 
                                                                 input_transformed_legs, 
                                                                 A_legs)

# Evaluate thresholds: euclidean distance & rotation
result_face  = decide_face(max_distance_face, rotation_face)
result_torso = decide_torso(max_distance_torso, rotation_torso)
result_legs  = decide_legs(max_distance_legs, rotation_legs)
