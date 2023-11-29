from roblib import *

def angle_to_index(angle,resolution):
    return int(angle*resolution/(2*pi))

def index_to_angle(index,resolution):
    return (index*2*pi)/resolution

def get_pos(angle,val,x_off,y_off):
    return [val*cos(angle)+x_off,val*sin(angle)+y_off]

class Lidar():
    def __init__(self,posx,posy,datas):
        self.datas = datas
        self.posx = posx
        self.posy = posy

    def show_lidar(self,ax,col,r=0.1,alpha=1.0):
        draw_disk(ax, [self.posx,self.posy], 1, col,alpha)
        for i in range(len(self.datas)):
            if self.datas[i] != inf:
                pos=get_pos(index_to_angle(i,len(self.datas)),self.datas[i],self.posx,self.posy)
                draw_disk(ax, pos, r, col,alpha)

def transform_lidar(lidar,new_origin):
    resolution = len(lidar.datas)
    new_data = [inf*i for i in range(resolution)]
    off_vect = [lidar.posx-new_origin[0], lidar.posy-new_origin[1]] #origin=>lidar vector
    for i in range(resolution):
        init_angle= index_to_angle(i,resolution=resolution)
        init_val= lidar.datas[i]
        if init_val != inf: #the value stay inf wathever the new reference
            init_pos= get_pos(init_angle,init_val,0,0) #pos in lidar frame, lidar=>point vector
            new_pos = [init_pos[0]+off_vect[0],init_pos[1]+off_vect[1]] #pos in new_origin frame, origin=>point vector
            new_val = sqrt((new_pos[0]**2)+(new_pos[1]**2))
            new_angle = arctan2(new_pos[1],new_pos[0])
            if new_angle<0:
                new_angle=2*pi+new_angle
            if new_angle == 2*pi:
                new_angle=0
            new_index = angle_to_index(new_angle,len(lidar2.datas))
            new_data[new_index] = min(new_val,new_data[new_index]) #in case some area are no more accessible from the new origin
            #print("Debug: ",[i,init_angle,init_val,init_pos,new_pos,new_val,new_angle,new_index,off_vect])
    return new_data
def fuse_lidar(lidar1,lidar2,new_origin):
    #transform lidar2 values in lidar1 frame
    merged_data = []
    new_data1 = transform_lidar(lidar1,new_origin)
    new_data2 = transform_lidar(lidar2,new_origin)

    for i in range(len(new_data1)):
        merged_data.append(min(new_data1[i],new_data2[i]))

    return Lidar(new_origin[0],new_origin[1],merged_data)

lidar_ranges1 = [inf if 0 <= angle <= 180 else 2 if 180 <= angle <= 225 else 10 for angle in range(0, 360,int(360/360))]
lidar_ranges2 = [inf if 180 <= angle <= 360 else 3 if 65 <= angle <= 180 else 10 for angle in range(0, 360, int(360/360))]

#lidar_ranges1 = [10,10,10,inf,inf,inf]
#lidar_ranges2 = [inf,inf,inf,10,5,5]

lidar1 = Lidar(-3,-3,lidar_ranges1)
lidar2 = Lidar(3,3,lidar_ranges2)

ax = init_figure(-20, 20, -20, 20)
lidar1.show_lidar(ax,"red")
lidar2.show_lidar(ax,"green")

new_origin = [0,0]
lidar3 = fuse_lidar(lidar1,lidar2,new_origin)
lidar3.show_lidar(ax,"purple",r=0.5,alpha=0.2)

pause(50)