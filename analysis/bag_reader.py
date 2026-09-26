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
        mask = (d['t_cmd'] >= chunk['t'][0]) & (d['t_cmd'] <= chunk['t'][-1])
        chunk['t_cmd'] = d['t_cmd'][mask]
        chunk['effort'] = d['effort'][mask]
        chunks.append(chunk)
    return chunks

def fell(theta):
    if max(abs(theta))>np.deg2rad(45):
        return True
    return False

def settling_time(t, theta):
    if fell(theta) is True:
        return None
    bound=2*(abs(theta[0])/100)
    i=len(theta)-1
    while theta[i]<bound and theta[i]>-bound:
        i=i-1
    return t[i+1]-t[0]

def time_above(t, param, bound):
    total = 0.0
    for i in range(len(param) - 1):
        if abs(param[i]) >= bound:
            total += t[i+1] - t[i]
    return total

def metrics(chunk, max_torque, max_arm_speed):
    return {
        'theta0': np.rad2deg(chunk['theta'][0]),
        'fell': fell(chunk['theta']),
        'settling': settling_time(chunk['t'], chunk['theta']),
        'arm_sat': time_above(chunk['t'], chunk['w_arm'], 0.99*max_arm_speed),
        'torque_sat': time_above(chunk['t_cmd'], chunk['effort'], 0.99*max_torque),
    }

def print_table(rows):
    print(f"{'theta0 (deg)':>12} {'fell':>5} {'settle (s)':>10} {'arm sat (s)':>11} {'torque sat (s)':>14}")
    for r in rows:
        settle = '-' if r['settling'] is None else f"{r['settling']:.3f}"
        print(f"{r['theta0']:>12.2f} {str(r['fell']):>5} {settle:>10} {r['arm_sat']:>11.3f} {r['torque_sat']:>14.3f}")


if __name__ == '__main__':
    import sys
    d = open_bag(sys.argv[1])
    max_torque = float(sys.argv[2]) if len(sys.argv) > 2 else float('inf')
    max_arm_speed = float(sys.argv[3]) if len(sys.argv) > 3 else 100.0
    chunks = cut(d, find_teleport(d))
    rows = [metrics(c, max_torque, max_arm_speed) for c in chunks]
    print_table(rows)
