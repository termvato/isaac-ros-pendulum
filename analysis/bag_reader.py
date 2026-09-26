import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def open_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    t, theta, theta_dot, w_arm = [], [], [], []
    t_cmd, effort = [], []

    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic not in ('/joint_states', '/joint_command'):
            continue
        msg = deserialize_message(raw, get_message(types[topic]))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if topic == '/joint_states':
            b = msg.name.index('base_body')
            a = msg.name.index('body_arm')
            t.append(stamp)
            theta.append(msg.position[b])
            theta_dot.append(msg.velocity[b])
            w_arm.append(msg.velocity[a])
        else:
            a = msg.name.index('body_arm')
            t_cmd.append(stamp)
            effort.append(msg.effort[a])

    return {
        't': np.array(t), 'theta': np.array(theta),
        'theta_dot': np.array(theta_dot), 'w_arm': np.array(w_arm),
        't_cmd': np.array(t_cmd), 'effort': np.array(effort)
    }


def find_teleport(d):
    theta=d['theta']
    chunk_id=[]
    i=0
    flag=False
    step = float(np.deg2rad(2))
    while i < len(theta)-1:
        delta=abs(theta[i]-theta[i+1])
        if delta>=step and flag is False:
            chunk_id.append(i+1)
            flag=True
        elif delta<step and flag is True:
            flag=False
        i+=1
    
    return chunk_id

def cut(d, chunk_id):
    bounds = chunk_id + [len(d['theta'])]
    chunks=[]
    for i in range(len(chunk_id)):
        chunk={}
        for key in ['t', 'theta', 'theta_dot', 'w_arm']:
            chunk[key] = d[key][bounds[i]:bounds[i+1]]
        chunks.append(chunk)
    return chunks

if __name__ == '__main__':
    import sys
    d = open_bag(sys.argv[1])
    starts = find_teleport(d)
    print(starts)
    chunks = cut(d, starts)
    print(len(chunks))
    print(chunks[0]['theta'][:5])
  