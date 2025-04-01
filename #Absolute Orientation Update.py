#Absolute Orientation Update

#10ms polling frequency
#Angular Velocity Vector (100Hz)
#Three axis of 'rotation speed' in rad/s
#Linear Acceleration Vector (100Hz)
#Three axis of linear acceleration data (acceleration minus gravity) in m/s^2
#Return values and a category (Due North,East,South, or West)

x_pos = 0 #base station location
y_pos = 0 
vel_x = 0 #robot starting velocity
vel_y = 0 
angle = 0 #starting orientation
time = 0.001 #polling time in s


def orient(x_accel,y_accel,roll_speed) :
    l_adj = 1 #linear error adjustement parameter
    r_adj = 1 #rotational error adjustment paramet
    vel_x += x_accel*time
    vel_y += y_accel*time
    x_pos += l_adj*(vel_x*time + 0.5*x_accel*time^2)
    y_pos += l_adj*(vel_y*time + 0.5*y_accel*time^2)
    a_vel = 57.295779513082 * roll_speed    #Converts roll speed from rad/s to deg/s for angular velocity
    angle += r_adj*(a_vel*time)
    
    if angle > 360:         #Resets angle to stay within range of 0-360
        angle-=360
    if angle < 0:
        angle+=360
    
    match angle:
        case 0 if angle <= 5 | angle >=355 :
            heading = 0 #North
        case 1 if 85<=angle<=95:
            heading = 1 #East
        case 2 if 175<=angle<=185:
            heading = 2 #South
        case 3 if 265<=angle<=275:
            heading = 3 #West
        case 4:
            heading= 4 #Off course
            
    return heading
        
        
    
    
    
    
    