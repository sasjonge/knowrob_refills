#!/usr/bin/env python

import rospy
from collections import defaultdict

from geometry_msgs.msg import Vector3, Transform, TransformStamped
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._Quaternion import Quaternion
from std_msgs.msg._ColorRGBA import ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion
from json_prolog import json_prolog
from random import random

from unreal_msgs.msg import ModelDescription, InstanceId, MeshDescription, Tag
from unreal_msgs.srv import SpawnMultipleModels

class UnrealObject(object):
    def __init__(self):
        self.transform = None
        self.mesh_path = ''
        self.object_name = ''
        self.object_type = ''
        self.ref_frame = ''
        self.initialized = False
        self.visualize = False
        self.color = ColorRGBA(0, 0, 0, 1)
        self.scale = Vector3(0.05, 0.05, 0.05)

    def update_color(self, r, g, b, a):
        self.color = ColorRGBA()
        self.color.r = float(r)
        self.color.g = float(g)
        self.color.b = float(b)
        self.color.a = float(a)
    
    def update_transform(self, ref_frame, object_name, translation, rotation):
        self.ref_frame = str(ref_frame)
        self.object_name = str(object_name)
        self.transform = [self.ref_frame, self.object_name, translation, rotation]

    def update_dimensions(self, depth, width, height):
        self.scale = Vector3(width, depth, height)

    def get_message(self):
        object_id = self.object_name.split("_")[-1]
        msg = ModelDescription()
        # generate InstanceId
        msg.instance_id = InstanceId()
        msg.instance_id.class_name = self.object_type
        msg.instance_id.id = object_id
        # HACK this information should be in some ontology
        if 'ProductWithAN' in self.object_type:
          msg.instance_id.ns = '/IAISupermarket/Catalog'
        elif 'ShelfLabel' in self.object_type:
          msg.instance_id.ns = '/IAISupermarket/ShelfLabes'
        else:
          msg.instance_id.ns = '/IAISupermarket/Shelves'
        # generate MeshDescription
        # NOTE: default is to use the class name
        msg.mesh_description = MeshDescription()
        msg.mesh_description.path_to_mesh     = ''
        msg.mesh_description.path_to_material = ''
        # generate Tag's
        msg.tags.append(self.get_tag('SemLog','LogType','Static'))
        msg.tags.append(self.get_tag('SemLog','Id', object_id))
        msg.tags.append(self.get_tag('SemLog','Class', self.object_type))
        # set the pose
        msg.pose = Pose()
        msg.pose.position = Point()
        msg.pose.position.x = self.transform[2][0]
        msg.pose.position.y = self.transform[2][1]
        msg.pose.position.z = self.transform[2][2]
        msg.pose.orientation = Quaternion()
        msg.pose.orientation.x = self.transform[3][0]
        msg.pose.orientation.y = self.transform[3][1]
        msg.pose.orientation.z = self.transform[3][2]
        msg.pose.orientation.w = self.transform[3][3]
        return msg

    def get_tag(self,tag_type,key,value):
        tag = Tag()
        tag.tag_type = tag_type
        tag.key = key
        tag.value = value
        return tag

    def __repr__(self):
        return 'obj('+str(self.object_name)+','+\
                      str(self.mesh_path)+','+\
                      str([self.scale.x,self.scale.y,self.scale.z])+','+\
                      str([self.color.r,self.color.g,self.color.b,self.color.a])+')'


class UnrealShop(object):
    def __init__(self):
        rospy.wait_for_service('/json_prolog/query')
        self.prolog = json_prolog.Prolog()
        self.objects = defaultdict(lambda: UnrealObject())
        self.unreal_publisher = rospy.Publisher("/unreal", ModelDescription, queue_size=100)
        self.unreal_service = rospy.ServiceProxy('unreal/spawn_multiple_models', SpawnMultipleModels)
        rospy.loginfo('unreal publisher is running')

    def prolog_query(self, q, verbose=False):
        query = self.prolog.query(q)
        solutions = [x for x in query.solutions()]
        if verbose:
            if len(solutions) > 1:
                rospy.logwarn('{} returned more than one result'.format(q))
            elif len(solutions) == 0:
                rospy.logwarn('{} returned nothing'.format(q))
        query.finish()
        return solutions

    def load_objects(self, object_ids=None):
        self.load_object_ids()
        if object_ids is None:
            object_ids = self.objects.keys()
        self.load_object_information(object_ids)

    def load_object_ids(self):
        # TODO: replace with service calls telling the publisher that objects were created/destroyed.
        #       then this lookup has only to be done once initially
        q = 'belief_existing_objects(A)'
        solutions = self.prolog_query(q)
        for object_id in solutions[0]['A']:
            
            name = object_id.split("#")[-1]
            # HACK
            if name.startswith("Lightbulb"):       continue
            elif name.startswith("DMShelfSystem"): continue
            elif name.startswith("CeilingLight"):  continue
            
            if object_id not in self.objects.keys():
                self.objects[object_id] = UnrealObject()
        for object_id in self.objects.keys():
            if object_id not in solutions[0]['A']:
                self.marker_publisher.publish(self.objects[object_id].get_del_marker())
                self.objects.pop(object_id)
        rospy.logdebug('Loaded object ids: {}'.format([str(x) for x in self.objects.keys()]))

    def load_object_information(self, object_ids):
        q = "member(Obj,['"+"','".join(object_ids)+"']),object_information(Obj,Type,_,Color,Mesh,[D,W,H],_,_),belief_at(Obj,Pose)"
        solutions = self.prolog_query(q, verbose=False)
        # TODO: would be nice to use the INCREMENTAL mode of json_prolog here
        for x in solutions:
            object_id = str(x['Obj']).replace('\'', '')
            obj = self.objects[object_id]
            obj.object_type = (str(x['Type']).replace('\'', ''))
            obj.update_transform(*x['Pose'])
            obj.update_color(*x['Color'])
            obj.mesh_path = str(x['Mesh'])
            obj.update_dimensions(depth=x['D'],width=x['W'],height=x['H'])
            obj.initialized = True
            obj.object_name = object_id
            rospy.logdebug('Updated object: {}'.format(str(obj)))

    def spawn(self):
        self.load_objects()
        msgs = map(lambda o: o.get_message(), self.objects.values())
        print(msgs)
        self.unreal_service(msgs)

if __name__ == '__main__':
    rospy.init_node('unreal_shop')
    unreal_shop = UnrealShop()
    unreal_shop.spawn()
