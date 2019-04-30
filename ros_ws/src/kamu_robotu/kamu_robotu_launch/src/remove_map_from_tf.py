import rosbag
import sys
from tf.msg import tfMessage


bag_dir = sys.argv[1]
bagfile_name = sys.argv[2]

input_bagfile = bag_dir + "/" + bagfile_name + ".bag"

with rosbag.Bag('output.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag(input_bagfile).read_messages():
        if topic == "/tf" and msg.transforms:
            filteredTransforms = [];
            for m in msg.transforms:
                if m.header.frame_id != "map":
                    filteredTransforms.append(m)
                else:
                    print 'map frame removed!'
            if len(filteredTransforms)>0:
                msg.transforms = filteredTransforms
                outbag.write(topic, msg, t)
        else:
            outbag.write(topic, msg, t)
