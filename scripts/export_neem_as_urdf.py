import rospy
from giskard_msgs.msg import WorldBody

from giskardpy import tfwrapper as tf
from giskardpy.urdf_object import URDFObject
from knowrob_refills.knowrob_wrapper import KnowRob

rospy.init_node('neem_as_urdf_exporter')
tf.init(5)

knowrob = KnowRob(
    # initial_mongo_db=None,
    # initial_mongo_db='/home/stelter/mongo_logs/2020-12-18_18-12-01/roslog',
    initial_mongo_db='/home/stelter/mongo_logs/2020-12-18_18-12-012/',
    clear_roslog=True,
    # clear_roslog=True,
    # republish_tf=True,
    neem_mode=True)


class Store(object):
    def __init__(self, knowrob):
        self.store = URDFObject(store_urdf)
        self.knowrob = knowrob
        self.root_name = self.store.get_root()

    def obj_to_urdf(self, knowrob_id, obj_name):
        path = knowrob.get_mesh(knowrob_id)
        obj = WorldBody()
        obj.type = WorldBody.MESH_BODY
        obj.mesh = path
        obj.name = obj_name
        return URDFObject.from_world_body(obj)

    def add_mesh_part(self, knowrob_id, parent):
        obj_frame_id = knowrob.get_object_frame_id(knowrob_id)
        try:
            urdf_obj = self.obj_to_urdf(knowrob_id, obj_frame_id)
            if parent != 'map':
                parent = knowrob.get_object_frame_id(parent)
            pose_stamped = tf.lookup_pose(parent, obj_frame_id)
            if parent == 'map':
                pose_stamped.header.frame_id = self.root_name
            self.store.attach_urdf_object(urdf_obj, pose_stamped.header.frame_id, pose_stamped.pose)
        except:
            print('failed to add unknown shape object')

    def get_urdf_str(self):
        return self.store.get_urdf_str()


with open('empty_store.urdf', 'r') as f:
    store_urdf = f.read()

store = Store(knowrob)

for shelf_id in knowrob.get_shelf_system_ids(False):
    store.add_mesh_part(shelf_id, 'map')
    for layer_id in knowrob.get_shelf_layer_from_system(shelf_id):
        store.add_mesh_part(layer_id, shelf_id)
        for separator_id in knowrob.get_separator_from_layer(layer_id):
            store.add_mesh_part(separator_id, layer_id)
        # for label_id in knowrob.get_label_from_layer(layer_id):
        #     store.add_mesh_part(label_id, layer_id)
        for facing_id in store.knowrob.get_facing_ids_from_layer(layer_id):
            for product_id in store.knowrob.get_products_in_facing(facing_id):
            # p = store.knowrob.prolog_to_pose_msg(bindings['Pose'])
                store.add_mesh_part(product_id, layer_id)
    # break

with open('full_store.urdf', 'w') as f:
    f.write(store.get_urdf_str())
print('saved file. done.')
