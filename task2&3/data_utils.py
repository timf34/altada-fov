from with_camera.data_classes import Detections
from issia_utils import _load_groundtruth
from dataclasses import asdict

import json
import os

# Read data from ground truth files
# Create Detections object for each x y that is found
# Find homographied x y coordinates for Detections object ... actually not, havent decided whether to do homography
# conversion on the camera or on the server. Will probably do it on the camera but leave it for no.
# Convert this Detections object to a dict and then add to a json file

DATA = 'test_detections.json'


def create_json_gt_files():
    # Messy af but it'll do the job

    cameras = [5, 6]
    dataset_path = r'C:\Users\timf3\Desktop\Datasets\Sequences'
    annotation_path = os.path.join(dataset_path, 'Annotation Files')
    frame_shape = 1920
    delta = -8
    ball_data = {5: [], 6: []}

    for camera_ids in cameras:
        annotation_file = 'Film Role-0 ID-' + str(camera_ids) + ' T-0 m00s00-026-m00s01-020.xgtf'
        annotation_filepath = os.path.join(annotation_path, annotation_file)
        gt = _load_groundtruth(annotation_filepath)

        for (start_frame, end_frame, x, y) in gt['BallPos']:
            if camera_ids == 6:
                # For some reason ball coordinates for camera 2 and 6 have reversed in x-coordinate
                x = frame_shape - x
            start_frame += delta
            dets = Detections(camera_id=camera_ids, probability=.9, timestamp=start_frame, x=x, y=y)
            ball_data[camera_ids].append(asdict(dets))

    with open('test_detections.json', 'w') as write_file:
        json.dump(ball_data, write_file, indent=2)

    return


create_json_gt_files()


def read_json_gt_files():

    with open(DATA,'r') as data:
        data = json.load(data)
        for i in data:
            detection = Detections(camera_id=i['camera_id'], timestamp=i['timestamp'], probability=i['probability'],
                                   x=i['x'], y=i['y'])
            print(detection)


# read_json_gt_files()

