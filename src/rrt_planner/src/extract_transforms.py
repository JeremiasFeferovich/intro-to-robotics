import rosbag
import rospy
from tf.msg import tfMessage

# Function to extract transformations


def extract_transformations(bag_file):
    transformations = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/tf']):
            if isinstance(msg, tfMessage):
                for transform in msg.transforms:
                    transformations.append({
                        'timestamp': t.to_sec(),
                        'frame_id': transform.header.frame_id,
                        'child_frame_id': transform.child_frame_id,
                        'translation': (transform.transform.translation.x,
                                        transform.transform.translation.y,
                                        transform.transform.translation.z),
                        'rotation': (transform.transform.rotation.x,
                                     transform.transform.rotation.y,
                                     transform.transform.rotation.z,
                                     transform.transform.rotation.w)
                    })

    return transformations


# Replace 'your_bag_file.bag' with your actual bag file name
bag_file = '../results/2024-10-17-17-28-24.bag'
transformations = extract_transformations(bag_file)

# Print the extracted transformations
for transform in transformations:
    print(transform)
