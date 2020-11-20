#!/usr/bin/env python

import rospy

from knowrob_refills.knowrob_wrapper import KnowRob
# from refills_perception_interface.tfwrapper import lookup_pose

db = '~/mongo_logs/2020-11-20_14-48-12/roslog'

rospy.init_node('test')
knowrob = KnowRob(initial_mongo_db=db,
                  clear_roslog=True)
# knowrob = KnowRob(initial_mongo_db=None,
#                   clear_roslog=False)
shelf_ids = knowrob.get_shelf_system_ids(False)
# print(shelf_ids)
for shelf_id in shelf_ids:
    print('shelf center frame id {}'.format(knowrob.get_object_frame_id(shelf_id)))
    print('shelf corner frame id {}'.format(knowrob.get_perceived_frame_id(shelf_id)))
    # print(knowrob.get_shelf_pose(shelf_id))
    # print(lookup_pose('map', perceived_frame_id))