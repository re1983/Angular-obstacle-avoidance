from vpython import *

tgraph=graph(xtitle="Bearing [deg]",ytitle="Time [s]",xmin=-180,xmax=180)
f1=gcurve(color=color.blue)
vsub=10
vboat=5
thetas=40*pi/180
thetab=-20*pi/180

sub=cylinder(pos=vector(-500,-600,0),radius=8,axis=50*vector(sin(thetas),cos(thetas),0), color=color.yellow,make_trail=True)
boat=box(pos=vector(700,0,0), size=vector(50,50,5), make_trail=True,color=color.cyan)

sub.v=vsub*vector(sin(thetas),cos(thetas),0)
boat.v=vboat*vector(sin(thetab),cos(thetab),0)

t=0
dt=5

while t<100:
    rate(10)
    sub.pos=sub.pos+sub.v*dt
    boat.pos=boat.pos+boat.v*dt
    r=boat.pos-sub.pos
    bearing=acos(dot(sub.v,r)/(mag(sub.v)*mag(r)))*180/pi
    t=t+dt
    f1.plot(bearing,-t)