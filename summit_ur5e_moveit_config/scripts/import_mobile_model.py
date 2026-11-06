#!/usr/bin/env python3
"""Import the supplied description without registering duplicate ROS packages."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import numpy as np
from xml.dom import minidom

ARM=["ur5e_shoulder_pan_joint","ur5e_shoulder_lift_joint","ur5e_elbow_joint",
     "ur5e_wrist_1_joint","ur5e_wrist_2_joint","ur5e_wrist_3_joint"]
MJ_ARM=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
def xml(path,root):
    path.write_text(minidom.parseString(ET.tostring(root)).toprettyxml())
def origin(node):
    o=node.find("origin")
    return dict(pos=o.get("xyz","0 0 0"),euler=o.get("rpy","0 0 0")) if o is not None else dict(pos="0 0 0",euler="0 0 0")
def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("source",type=Path)
    args=parser.parse_args()
    pkg=Path(__file__).resolve().parents[1]
    config=pkg/"config"
    source=ET.parse(args.source/"summit_ur5e.urdf").getroot()
    model=copy.deepcopy(source)
    for child in list(model):
        if child.tag in ("gazebo","transmission"): model.remove(child)
    for mesh in model.findall(".//mesh"):
        uri=mesh.get("filename")
        if not uri.startswith("package://"): raise ValueError(uri)
        relative=uri[len("package://"):]
        src=args.source/relative
        if not src.is_file(): raise FileNotFoundError(src)
        dest=pkg/"mobile_assets"/relative
        dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(src,dest)
        mesh.set("filename","package://summit_ur5e_moveit_config/mobile_assets/"+relative)
    xml(config/"mobile_manipulator.urdf",model)

    # Legacy actions command x/y/yaw plus the same six arm joints.
    action=copy.deepcopy(model)
    action.insert(0,ET.Element("link",name="world"))
    action.insert(1,ET.Element("link",name="mobile_x_link"))
    action.insert(2,ET.Element("link",name="mobile_y_link"))
    for name,parent,child,kind,axis,lower,upper,speed in [
        ("v_x_joint","world","mobile_x_link","prismatic","1 0 0","-100","100",".2"),
        ("v_y_joint","mobile_x_link","mobile_y_link","prismatic","0 1 0","-100","100",".2"),
        ("v_t_joint","mobile_y_link","summit_xl_base_footprint","revolute","0 0 1","-100","100",".3")]:
        j=ET.SubElement(action,"joint",name=name,type=kind)
        ET.SubElement(j,"parent",link=parent);ET.SubElement(j,"child",link=child)
        ET.SubElement(j,"axis",xyz=axis)
        ET.SubElement(j,"limit",lower=lower,upper=upper,velocity=speed,effort="1000")
    for joint in action.findall("joint"):
        if joint.get("name") not in ARM+["v_x_joint","v_y_joint","v_t_joint"]:
            joint.set("type","fixed")
            for tag in ["mimic","limit","axis","dynamics"]:
                for item in joint.findall(tag):joint.remove(item)
    xml(config/"mobile_action_control.urdf",action)

    srdf=ET.Element("robot",name=source.get("name"))
    group=ET.SubElement(srdf,"group",name="manipulator")
    ET.SubElement(group,"chain",base_link="ur5e_base_link",tip_link="graps_link")
    state=ET.SubElement(srdf,"group_state",name="home",group="manipulator")
    for name,q in zip(ARM,[0,-1.57,1.57,-1.57,-1.57,0]):
        ET.SubElement(state,"joint",name=name,value=str(q))
    links=[l.get("name") for l in model.findall("link")]
    parent={n:n for n in links}
    def find(x):
        while parent[x]!=x:x=parent[x]
        return x
    def union(a,b):parent[find(a)]=find(b)
    edges={}
    for joint in model.findall("joint"):
        a=joint.find("parent").get("link");b=joint.find("child").get("link")
        edges.setdefault(a,set()).add(b);edges.setdefault(b,set()).add(a)
        if joint.get("name") not in ARM:
            union(a,b)
            if joint.get("type")!="fixed":
                ET.SubElement(srdf,"passive_joint",name=joint.get("name"))
    adjacent=set()
    for joint in model.findall("joint"):
        a=find(joint.find("parent").get("link"));b=find(joint.find("child").get("link"))
        if a!=b:adjacent.add(tuple(sorted((a,b))))
    for i,a in enumerate(links):
        near=set(edges.get(a,()))|{c for b in edges.get(a,()) for c in edges.get(b,())}
        for b in links[i+1:]:
            ca,cb=find(a),find(b)
            if ca==cb or tuple(sorted((ca,cb))) in adjacent or b in near:
                ET.SubElement(srdf,"disable_collisions",link1=a,link2=b,
                              reason="LockedAssembly" if ca==cb else "Adjacent")
    xml(config/"mobile_manipulator.srdf",srdf)

    # Preserve the existing nine-axis MuJoCo environment and add the supplied
    # fixed camera/gripper attachment at its actual URDF transforms.
    old=pkg.parent/"robot_description/universal_robots_ur5e"
    mj=ET.parse(old/"summit_ur5e.xml").getroot()
    asset_dir=pkg/"mobile_sim_assets"
    asset_dir.mkdir(exist_ok=True)
    for item in mj.findall("asset/mesh"):
        shutil.copy2(old/"assets"/item.get("file"),asset_dir/item.get("file"))
    mj.find("compiler").set("eulerseq","XYZ") # URDF fixed-axis RPY
    mj.find("compiler").set("meshdir",os.path.relpath(asset_dir,config))
    # The legacy chassis is one convex mesh (including its wheels), which
    # penetrates the floor and the adjacent shoulder capsule at nominal home.
    # These are representation overlaps, not contacts in the source assembly.
    contact=ET.SubElement(mj,"contact")
    ET.SubElement(contact,"exclude",body1="world",body2="base_link")
    ET.SubElement(contact,"exclude",body1="base_link",body2="upper_arm_link")
    wrist=mj.find(".//body[@name='wrist_3_link']")
    asset=mj.find("asset")
    by_link={l.get("name"):l for l in model.findall("link")}
    by_parent={}
    for joint in model.findall("joint"):
        by_parent.setdefault(joint.find("parent").get("link"),[]).append(joint)
    mesh_count=[0]
    def attach(parent_body,parent_link):
        for joint in by_parent.get(parent_link,[]):
            name=joint.find("child").get("link")
            link=by_link[name]
            body=ET.SubElement(parent_body,"body",name=name,**origin(joint))
            inertia=link.find("inertial")
            if inertia is not None:
                tensor=inertia.find("inertia")
                attrs=origin(inertia)
                roll,pitch,yaw=[float(v) for v in attrs.pop("euler").split()]
                cr,sr,cp,sp,cy,sy=np.cos(roll),np.sin(roll),np.cos(pitch),np.sin(pitch),np.cos(yaw),np.sin(yaw)
                R=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]]) @ np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]]) @ np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
                xx,yy,zz,xy,xz,yz=[float(tensor.get(n)) for n in ["ixx","iyy","izz","ixy","ixz","iyz"]]
                I=R @ np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]]) @ R.T
                attrs["mass"]=inertia.find("mass").get("value")
                attrs["fullinertia"]=" ".join(str(I[i,j]) for i,j in [(0,0),(1,1),(2,2),(0,1),(0,2),(1,2)])
                ET.SubElement(body,"inertial",**attrs)
            for geometry in link.findall("collision"):
                shape=geometry.find("geometry")
                attrs=origin(geometry)
                # Imported concave tool meshes become convex hulls in MuJoCo and
                # intersect the legacy arm capsules. Keep attachments visual
                # and inertial; object/gripper contact is outside these tests.
                attrs.update(group="2",rgba=".65 .68 .72 1",density="0",contype="0",conaffinity="0")
                mesh=shape.find("mesh")
                if mesh is not None:
                    uri=mesh.get("filename")
                    f=pkg/uri[len("package://summit_ur5e_moveit_config/"):]
                    name_mesh="imported_tool_"+str(mesh_count[0]);mesh_count[0]+=1
                    if f.suffix.lower()!=".stl": raise ValueError("Tool collision mesh must be STL: "+str(f))
                    ET.SubElement(asset,"mesh",name=name_mesh,file=os.path.relpath(f,asset_dir),
                                  scale=mesh.get("scale","1 1 1"))
                    attrs.update(type="mesh",mesh=name_mesh)
                elif shape.find("box") is not None:
                    attrs.update(type="box",size=" ".join(str(float(v)/2) for v in shape.find("box").get("size").split()))
                elif shape.find("cylinder") is not None:
                    c=shape.find("cylinder");attrs.update(type="cylinder",size=c.get("radius")+" "+str(float(c.get("length"))/2))
                elif shape.find("sphere") is not None:
                    attrs.update(type="sphere",size=shape.find("sphere").get("radius"))
                else:continue
                # Fixed attachment geometry has no independent DOF or actuator.
                ET.SubElement(body,"geom",**attrs)
            attach(body,name)
    attach(wrist,"ur5e_wrist_3_link")
    xml(config/"mobile_mujoco_model.xml",mj)
    scene=ET.parse(old/"summit_ur5e_sim.xml").getroot()
    # Stiff base velocity servos require implicit actuator damping integration.
    scene.find("option").set("integrator","implicitfast")
    scene.find("include").set("file","mobile_mujoco_model.xml")
    xml(config/"mobile_mujoco_scene.xml",scene)
    mapping=dict(source=str(args.source.resolve()),source_sha256=hashlib.sha256((args.source/"summit_ur5e.urdf").read_bytes()).hexdigest(),
                 robot=source.get("name"),links=len(links),
                 arm_joints=dict(zip(ARM,MJ_ARM)),
                 base=dict(v_x_joint="base_x_slide_joint",v_y_joint="base_y_slide_joint",v_t_joint="base_z_hinge_joint"),
                 arm_mount_world=[.225,0,.522],sensor_to_wrist_xyz=[0,0,0],
                 sensor_to_wrist_rpy=[-1.5707963267948966,0,0],
                 passive_joints=[j.get("name") for j in model.findall("joint") if j.get("type")!="fixed" and j.get("name") not in ARM],
                 moveit_base_mode="fixed",action_base_mode="commanded",gripper="locked at zero")
    (config/"mobile_mapping.json").write_text(json.dumps(mapping,indent=2))
    print(json.dumps(mapping,indent=2))
if __name__=="__main__":main()
