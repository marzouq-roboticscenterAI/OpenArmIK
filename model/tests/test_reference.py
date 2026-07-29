#!/usr/bin/env python3
import math
import pathlib
import random
import subprocess
import sys
import xml.etree.ElementTree as ET

def vec(text): return [float(x) for x in text.split()]
def mul(a, b): return [sum(a[i*4+k]*b[k*4+j] for k in range(4)) for i in range(4) for j in range(4)]
def origin(node):
    value = node.find("origin")
    xyz = vec(value.get("xyz", "0 0 0")); r, p, y = vec(value.get("rpy", "0 0 0"))
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return [cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,xyz[0],sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,xyz[1],-sp,cp*sr,cp*cr,xyz[2],0,0,0,1]
def rotate(axis, q):
    x,y,z=axis;c=math.cos(q);s=math.sin(q);d=1-c
    return [c+x*x*d,x*y*d-z*s,x*z*d+y*s,0,y*x*d+z*s,c+y*y*d,y*z*d-x*s,0,z*x*d-y*s,z*y*d+x*s,c+z*z*d,0,0,0,0,1]
def reference(base, chain, tcp, q):
    current=base; axes=[]; origins=[]
    for joint, angle in zip(chain,q):
        pre=mul(current,origin(joint)); a=vec(joint.find("axis").get("xyz"))
        axes.append([sum(pre[r*4+k]*a[k] for k in range(3)) for r in range(3)])
        origins.append([pre[3],pre[7],pre[11]]);current=mul(pre,rotate(a,angle))
    tip=mul(current,tcp);p=[tip[3],tip[7],tip[11]];jac=[[0.0]*7 for _ in range(6)]
    for i,(a,o) in enumerate(zip(axes,origins)):
        d=[p[k]-o[k] for k in range(3)];jac[0][i]=a[1]*d[2]-a[2]*d[1];jac[1][i]=a[2]*d[0]-a[0]*d[2];jac[2][i]=a[0]*d[1]-a[1]*d[0]
        for r in range(3):jac[3+r][i]=a[r]
    return tip+[v for row in jac for v in row]

urdf, driver = map(pathlib.Path,sys.argv[1:3]); joints={j.get("name"):j for j in ET.parse(urdf).getroot().findall("joint")};rng=random.Random(0x5A17);inputs=[];expected=[]
for side_index,side in enumerate(("left","right")):
    prefix=f"openarm_{side}_";base=origin(joints[prefix+"openarm_body_link0_joint"]);chain=[joints[prefix+f"joint{i}"] for i in range(1,8)]
    tcp=mul(origin(joints[prefix+"hand_joint"]),origin(joints[prefix+"hand_tcp_joint"]))
    for _ in range(300):
        q=[rng.uniform(float(j.find("limit").get("lower")),float(j.find("limit").get("upper"))) for j in chain]
        inputs.append(str(side_index)+" "+" ".join(format(x,".17g") for x in q));expected.append(reference(base,chain,tcp,q))
process=subprocess.run([str(driver)],input="\n".join(inputs)+"\n",text=True,capture_output=True,check=True);actual=[[float(x) for x in line.split()] for line in process.stdout.splitlines()]
if len(actual)!=len(expected):raise SystemExit("reference driver result count differs")
maximum=max(abs(a-b) for got,want in zip(actual,expected) for a,b in zip(got,want))
if maximum>5e-12:raise SystemExit(f"independent URDF reference error {maximum}")
print(f"600 random URDF FK/Jacobian cases, max error {maximum:.3g}")
