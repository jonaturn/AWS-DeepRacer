    # Import package (needed for heading)
import math



class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index=0
        # self.verbose = verbose

    def reward_function(self, params):

    
        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[-0.11087, -3.28924, 4.0, 0.04281],
                [0.00309, -3.3518, 4.0, 0.0325],
                [0.11704, -3.41436, 4.0, 0.0325],
                [0.26716, -3.49678, 4.0, 0.04281],
                [0.53122, -3.64176, 4.0, 0.07531],
                [0.79529, -3.78674, 4.0, 0.07531],
                [1.05936, -3.93172, 4.0, 0.07531],
                [1.32343, -4.07665, 4.0, 0.07531],
                [1.58753, -4.22125, 4.0, 0.07527],
                [1.85165, -4.36552, 4.0, 0.07524],
                [2.11583, -4.50908, 4.0, 0.07517],
                [2.38008, -4.65153, 4.0, 0.07505],
                [2.64443, -4.79248, 4.0, 0.07489],
                [2.9089, -4.93143, 4.0, 0.07469],
                [3.17352, -5.0679, 3.6788, 0.08093],
                [3.4383, -5.20132, 3.36753, 0.08805],
                [3.70327, -5.33109, 2.97392, 0.09921],
                [3.96842, -5.45652, 2.6001, 0.11281],
                [4.23376, -5.57679, 2.28825, 0.12731],
                [4.49927, -5.69095, 2.00013, 0.14449],
                [4.7649, -5.79779, 1.75394, 0.16324],
                [5.03058, -5.89589, 1.50925, 0.18765],
                [5.29616, -5.98353, 1.50925, 0.1853],
                [5.56143, -6.05912, 1.50925, 0.18276],
                [5.82602, -6.11984, 1.50925, 0.17987],
                [6.08928, -6.16196, 1.50925, 0.17665],
                [6.35015, -6.18126, 1.50925, 0.17332],
                [6.60677, -6.17225, 1.50925, 0.17014],
                [6.85592, -6.12833, 1.50925, 0.16763],
                [7.09069, -6.03978, 1.67086, 0.15017],
                [7.30962, -5.91676, 1.81244, 0.13856],
                [7.51197, -5.76582, 1.99508, 0.12653],
                [7.69813, -5.59313, 2.13021, 0.1192],
                [7.86807, -5.40269, 2.26025, 0.11293],
                [8.02199, -5.19785, 2.39254, 0.10709],
                [8.16039, -4.98148, 2.48746, 0.10325],
                [8.28351, -4.75571, 2.57427, 0.0999],
                [8.39191, -4.52241, 2.3327, 0.11028],
                [8.48584, -4.28301, 2.13225, 0.12061],
                [8.56549, -4.03873, 1.925, 0.13347],
                [8.63107, -3.79067, 1.72485, 0.14875],
                [8.68269, -3.53983, 1.5, 0.17073],
                [8.7201, -3.28709, 1.5, 0.17033],
                [8.7424, -3.03333, 1.5, 0.16982],
                [8.74828, -2.77959, 1.5, 0.16921],
                [8.73437, -2.52737, 1.5, 0.1684],
                [8.69728, -2.27898, 1.5, 0.16743],
                [8.63268, -2.03794, 1.5, 0.16637],
                [8.53541, -1.80953, 1.5, 0.16551],
                [8.3978, -1.60322, 1.52064, 0.16309],
                [8.22408, -1.42384, 1.70525, 0.14643],
                [8.02359, -1.26953, 1.92875, 0.13117],
                [7.80301, -1.13712, 2.1372, 0.12037],
                [7.56643, -1.02428, 2.37844, 0.1102],
                [7.31709, -0.92861, 2.65578, 0.10056],
                [7.05757, -0.84772, 2.9972, 0.09069],
                [6.7901, -0.77917, 3.45568, 0.0799],
                [6.51664, -0.72038, 4.0, 0.06993],
                [6.23909, -0.66848, 4.0, 0.07059],
                [5.95923, -0.62058, 4.0, 0.07098],
                [5.67404, -0.57346, 4.0, 0.07226],
                [5.38853, -0.52356, 3.87896, 0.07472],
                [5.10327, -0.47022, 3.71504, 0.07812],
                [4.81885, -0.41263, 3.58437, 0.08096],
                [4.53585, -0.35016, 3.48205, 0.08323],
                [4.25478, -0.28203, 3.40302, 0.08499],
                [3.97612, -0.20756, 3.3436, 0.08627],
                [3.70031, -0.12609, 3.30165, 0.0871],
                [3.42785, -0.03693, 3.27634, 0.0875],
                [3.15927, 0.06058, 3.26789, 0.08744],
                [2.89511, 0.167, 3.26789, 0.08715],
                [2.63591, 0.28281, 3.26789, 0.08687],
                [2.38222, 0.40834, 3.26789, 0.08662],
                [2.13455, 0.54381, 3.26789, 0.08639],
                [1.8934, 0.68933, 3.26789, 0.08619],
                [1.65921, 0.84489, 3.26789, 0.08603],
                [1.43239, 1.01038, 3.26789, 0.08592],
                [1.21324, 1.18562, 3.27745, 0.08561],
                [1.002, 1.3703, 3.30703, 0.08485],
                [0.79878, 1.56405, 3.35971, 0.08357],
                [0.60359, 1.7664, 3.43978, 0.08173],
                [0.41629, 1.9768, 3.55317, 0.07928],
                [0.23659, 2.19459, 3.70779, 0.07615],
                [0.06406, 2.41907, 3.91423, 0.07233],
                [-0.10183, 2.6494, 4.0, 0.07096],
                [-0.26173, 2.88466, 4.0, 0.07111],
                [-0.41636, 3.12383, 3.57481, 0.07967],
                [-0.56655, 3.3658, 3.0572, 0.09316],
                [-0.71319, 3.60959, 2.62835, 0.10824],
                [-0.85695, 3.85444, 2.30561, 0.12315],
                [-0.99841, 4.0998, 2.11499, 0.13391],
                [-1.13344, 4.33761, 2.06573, 0.13238],
                [-1.27082, 4.57302, 2.06573, 0.13195],
                [-1.41287, 4.80369, 2.06573, 0.13114],
                [-1.56179, 5.02734, 2.06573, 0.13007],
                [-1.71981, 5.24156, 2.06573, 0.12886],
                [-1.88965, 5.44319, 2.06573, 0.12762],
                [-2.07414, 5.62854, 2.06573, 0.1266],
                [-2.27515, 5.7943, 2.06573, 0.12613],
                [-2.49259, 5.93888, 2.11292, 0.12358],
                [-2.72489, 6.06222, 2.29674, 0.11452],
                [-2.96908, 6.16657, 2.52831, 0.10503],
                [-3.22246, 6.25439, 2.71401, 0.09881],
                [-3.48321, 6.32716, 2.86022, 0.09465],
                [-3.74989, 6.38576, 2.97175, 0.09188],
                [-4.0213, 6.43071, 3.07007, 0.08961],
                [-4.29627, 6.46244, 3.04889, 0.09079],
                [-4.57371, 6.48136, 2.82159, 0.09855],
                [-4.85257, 6.48782, 2.59795, 0.10737],
                [-5.13189, 6.48213, 2.3163, 0.12061],
                [-5.41076, 6.46471, 2.05338, 0.13608],
                [-5.68835, 6.43588, 1.83045, 0.15247],
                [-5.96385, 6.39579, 1.83045, 0.1521],
                [-6.2362, 6.34351, 1.83045, 0.1515],
                [-6.50412, 6.27804, 1.83045, 0.15068],
                [-6.76592, 6.19771, 1.83045, 0.14961],
                [-7.01953, 6.10061, 1.83045, 0.14835],
                [-7.26176, 5.9835, 1.83045, 0.14699],
                [-7.48825, 5.84237, 1.83045, 0.14579],
                [-7.69283, 5.67322, 2.0543, 0.12922],
                [-7.8781, 5.48378, 2.19979, 0.12045],
                [-8.04505, 5.27825, 2.32242, 0.11401],
                [-8.19438, 5.05983, 2.42668, 0.10903],
                [-8.32664, 4.83106, 2.40565, 0.10985],
                [-8.44245, 4.59403, 2.20018, 0.1199],
                [-8.54228, 4.3504, 2.00363, 0.1314],
                [-8.62619, 4.10145, 1.8008, 0.14588],
                [-8.69436, 3.84838, 1.56779, 0.16717],
                [-8.7465, 3.59221, 1.56779, 0.16675],
                [-8.78216, 3.33397, 1.56779, 0.16628],
                [-8.80097, 3.07474, 1.56779, 0.16578],
                [-8.80019, 2.81573, 1.56779, 0.16521],
                [-8.77629, 2.55882, 1.56779, 0.16458],
                [-8.72501, 2.30677, 1.56779, 0.16406],
                [-8.64067, 2.0638, 1.56779, 0.16405],
                [-8.51386, 1.8376, 1.98415, 0.1307],
                [-8.36087, 1.6244, 2.20742, 0.11888],
                [-8.18646, 1.42298, 2.47681, 0.10757],
                [-7.99463, 1.23186, 2.715, 0.09974],
                [-7.78788, 1.05008, 2.97826, 0.09244],
                [-7.56838, 0.87666, 3.27255, 0.08548],
                [-7.33802, 0.71066, 3.61916, 0.07845],
                [-7.09855, 0.55111, 4.0, 0.07194],
                [-6.85166, 0.39694, 4.0, 0.07277],
                [-6.5988, 0.24716, 4.0, 0.07347],
                [-6.34144, 0.10063, 4.0, 0.07404],
                [-6.08078, -0.04361, 4.0, 0.07448],
                [-5.81773, -0.18639, 4.0, 0.07482],
                [-5.55307, -0.32843, 4.0, 0.07509],
                [-5.2875, -0.47045, 4.0, 0.07529],
                [-5.02202, -0.61263, 4.0, 0.07529],
                [-4.75667, -0.75506, 4.0, 0.07529],
                [-4.49142, -0.89771, 4.0, 0.07529],
                [-4.22629, -1.0406, 4.0, 0.07529],
                [-3.96128, -1.18371, 4.0, 0.0753],
                [-3.69639, -1.32706, 4.0, 0.0753],
                [-3.43162, -1.47064, 4.0, 0.0753],
                [-3.16695, -1.61445, 4.0, 0.0753],
                [-2.90241, -1.75849, 4.0, 0.0753],
                [-2.63798, -1.90275, 4.0, 0.07531],
                [-2.37366, -2.04724, 4.0, 0.07531],
                [-2.10947, -2.19197, 4.0, 0.07531],
                [-1.84539, -2.33694, 4.0, 0.07531],
                [-1.58133, -2.48192, 4.0, 0.07531],
                [-1.31726, -2.6269, 4.0, 0.07531],
                [-1.05319, -2.77189, 4.0, 0.07531],
                [-0.78912, -2.91687, 4.0, 0.07531],
                [-0.52505, -3.06184, 4.0, 0.07531],
                [-0.26098, -3.20682, 4.0, 0.07531]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        # if self.verbose == True:
        #     self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        # if self.verbose == True:
        #     print("Closest index: %i" % closest_index)
        #     print("Distance to racing line: %f" % dist)
        #     print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
        #     print("Optimal speed: %f" % optimals[2])
        #     print("Speed difference: %f" % speed_diff)
        #     print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
        #     print("Direction difference: %f" % direction_diff)
        #     print("Predicted time: %f" % projected_time)
        #     print("=== Steps reward: %f ===" % steps_reward)
        #     print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)