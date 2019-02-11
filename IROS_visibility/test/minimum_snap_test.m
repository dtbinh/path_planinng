
waypts = [0,0;
          1,2;
          2,0;
          4,5;
          5,2]';
v0 = [0,0];
a0 = [0,0];
v1 = [0,0];
a1 = [0,0];
n_order = 5;
r = 0.4;
ts = [0 1 2 3 4];
[xx,yy,tt]=minimum_snap_corridor(ts,waypts,v0,a0,v1,a1,n_order,r);

plot(xx,yy,'r');

