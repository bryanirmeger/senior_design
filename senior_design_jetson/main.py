import j_serial
import Jetson.GPIO as GPIO
import time
from math import pi, sin, cos, radians

#transmit string
jr_con = "00jrdy"
poll = "00poll"
ping = "00ping"
move = "0mve"

#rx strings
br_con = "000brdyr"
rr_con = "000rrdyr"

#address bytes
j_ad = '0'
b_ad = '1'
r_ad = '2'

#status bytes
ready = "r"
non_ready = "n"

#robot data
us_sensors = b'0000'
x = 0
y = 0
heading = 0
status = 0
motor_v = 0
accel_x = 0
accel_y = 0

#data structures

class MAP_NODE:
    def __init__(self, x, y, px, nx, py, ny):
        self.x = x
        self.y = y
        self.px = px
        self.nx = nx
        self.py = py
        self.ny = ny
        self.sensors = [px, nx, py, ny]
        self.dmove = 0

    def __str__(self):
        return f"x: {self.x}, y: {self.y}\n {self.nx}\n{self.ny} {self.py}\n {self.px}"

class ROBOT:
    def __init__(self, port):
        #init a 1x1 map that will be added to as tid-e travels
        self.map = [[0,0],[0,0]]
        self.map_index = [0,0]
        #for when columns are added to the left of the home index, or rows added above
        self.home_index = [0,0]
        #tid-e absolute orientation
        self.position = [0,0]
        #robot params
        self.port = port
        #velocity relative to tid-e heading angle
        self.lvel = 0
        self.time = 0
        self.motor_v = 0
        self.heading = 0
        self.init_map()
   
    def command(self, ad, com):
        j_serial.Tx_Rx(self.port, ad + com + ready, False, True)
    
    def receive(self):
        return j_serial.Tx_Rx(self.port, "", True, False)

    def movement_rq(self, x, y, heading):
        #convert heading float to uint8_t to set over
        heading = (int)((heading / 360) * 0xFF)

        if x != 0:
            x = "1"
        else:
            x = "0"
        print(r_ad + move + x + chr(heading)+ ready)
        self.command(r_ad, move + x + chr(heading))
        while self.port.in_waiting == 0:
            time.sleep(0.1)
        rx = self.receive()
        #operation successful
        if "succ" in rx.decode():
            # if motor controller works
            #self.position = [x,y]
            self.heading = heading
            self.update_position() 

            # if motor controller DOES NOT WORK WITH FEEDBACK (constant v)
            return rx
        else:
            return "fail"

    def poll_r(self):
        self.command(r_ad, poll)
        while self.port.in_waiting == 0:
            time.sleep(0.1)
        rx = (self.receive()).decode()
        return [int(rx[1]), int(rx[2]), int(rx[3]), int(rx[4])]
    
    #from x and y acceleration as well as motor_v, and heading, update the position
    #motor_v, accel_x, accel_y are updated by parse_data
    #data is sensor data data
    #this is assuming that the motor controller doesn't work
        #from measured gear ratio 65:1 and v to motor speed model 1V -> 60Hz
    def update_position(self):
        delta_h = 0.152
        #need heading angle to determine direction. assuming that initial starting direction is positive x and to the right is positive y
        self.position[0] = self.position[0] + delta_h * cos(radians(self.heading))
        self.position[1] = self.position[1] + delta_h * sin(radians(self.heading))

    def reset_position(self):
        self.position = [0,0]
        self.heading = 0
        self.map_index = self.home_index

    def init_map(self):
        self.map[0][0] =  MAP_NODE(self.position[0], self.position[1], 0, 0, 0, 0)

    def update_map(self, sensor_data):
        sensor_data = rotate_sensors(sensor_data, self.heading)
        d_t = 0.150 #6in
        #assume position is up to date
        #check last position against current position
        last_pos = [self.map[self.map_index[0]][self.map_index[1]].x, self.map[self.map_index[0]][self.map_index[1]].y]

        dpos = [self.position[0] - last_pos[0], self.position[1] - last_pos[1]]
        #expected behavior is that tide moves in a single direction at a time when this is called
        #size of current map
        r = len(self.map)
        c = len(self.map[0])
        new_index = [0,0]

        #x moveent in meters
        if dpos[0] > d_t:
            new_index = [self.map_index[0] + 1, self.map_index[1]]
           #if map is large enough to house new point, add it in
            if new_index[0] <= r - 1:
                self.map[new_index[0]][new_index[1]] = MAP_NODE(self.position[0], self.position[1], sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3])
            #else, add in a new row
            else:
                #create empty row with column indexes
                new_row = [0] * c
                new_row[new_index[1]] =  MAP_NODE(self.position[0], self.position[1], sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3])
                self.map.append(new_row)

        #-x direction
        elif dpos[0] < -d_t:
            new_index = [self.map_index[0] - 1, self.map_index[1]]
            #check if enough rows from edge to add in above
            if new_index[0] >= 0:
                self.map[new_index[0]][new_index[1]] = MAP_NODE(self.position[0], self.position[1], sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3])
            #if not, add a column above by appending map to bottom of a new list. update home coords
            else:
                #self.index[0] = -1, need to account for
                map_copy = self.map.copy()
                new_row = [0] * c
                new_row[new_index[1]] =  MAP_NODE(self.position[0], self.position[1], sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3])
                new_map = [new_row]
                #append each row to new row 
                for row in range(r):
                    new_map.append(map_copy[row])
                self.map = new_map.copy()
                #this operation shifts home index down one coord (x + 1)
                self.home_index[0] = self.home_index[0] + 1
                new_index[0] = 0

        #movment in positive y
        elif dpos[1] > d_t:
            #next index is same x, next y
            new_index = [self.map_index[0], self.map_index[1] + 1]
            #check if map is large enough to accomodate, if so add, if not add new column
            if new_index[1] <= c - 1:
                self.map[new_index[0]][new_index[1]] = MAP_NODE(self.position[0], self.position[1], sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3])
            #else, add new column
            else:
                #add zero to each row
                for row in range(r):
                    self.map[row].append(0)
                #add new data to self.map
                self.map[new_index[0]][new_index[1]] = MAP_NODE(self.position[0], self.position[1], sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3])
                
        #-y movement
        elif dpos[1] < -d_t:
            #next index is same x, prev y
            new_index = [self.map_index[0], self.map_index[1] - 1]
            #check if map is far enough from left to accomodate, if so add, if not add column and shift positions to the the right
            if new_index[1] >= 0:
                self.map[new_index[0]][new_index[1]] = MAP_NODE(self.position[0], self.position[1], sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3])
            #else, add new column to the left, update the home node index
            else:
                #add zero to each row
                for row in range(r):
                    self.map[row].insert(0, 0)
                #add new data to self.map
                new_index = [new_index[0] , 0]
                self.map[new_index[0]][new_index[1]] = MAP_NODE(self.position[0], self.position[1], sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3])
                self.home_index[1] = self.home_index[1] + 1
                new_index[1] = 0

        self.map_index = new_index


    def __str__(self):
        #make a rxc map
        print(f"\nmap: {self.map}\nposition = {self.position}\nmap index = self.map_index\nhome_index = {self.home_index}\mheading = {self.heading}\n")


#helper functions

def rotate_sensors(sensor_data, heading):
    tolerance = 5
    aboslute_sensor_data = [0,0,0,0]
    #x will be zero degrees (initial direction), y is 90 degrees to "left" (around z)
    #sensor 0 is heading,s1 in -x, s2 in +y, s3, -y, 
    #no change, moving in positive x
    if (heading > -tolerance) and (heading < tolerance):
        return sensor_data
    #heading in positive y, rotate 90 deg (x = -y, -x = y, y = x, -y = -x)
    elif (heading > (-tolerance + 90)) and (heading < (tolerance + 90)):
        return  [sensor_data[3], sensor_data[2], sensor_data[0], sensor_data[1]]
    #heading in neg x, rotate 180 degrees
    elif (heading > (-tolerance + 180)) and (heading < (tolerance + 180)):
        return  [sensor_data[1], sensor_data[0], sensor_data[3], sensor_data[2]]
    #heading in neg y, rotate -90 degrees. 
    elif ((heading > (-tolerance - 90)) and (heading < (tolerance - 90))) or ((heading > (-tolerance + 270)) and (heading < (tolerance + 270))):
        return  [sensor_data[2], sensor_data[3], sensor_data[1], sensor_data[0]]


##################################################
#
# startup confirmation function with base micro
#
##################################################

def startup_base(port):
    #transmit ready confirmation seqeunce
    command(port, b_ad, jr_con)
    print("Jetson Startup Confirmation Sent to Base")

    #wait for micro ready confirmation
    while port.in_waiting == 0:
        time.sleep(1)

    #get data from buffer. if it doesn't match micro r confirmation then we have an issue
    rx = receive(port)

    #check if data matches, if it doesn't retry the ping
    if rx.decode()  == br_con:
        print("Base Startup Confirmation Successful")
        return True
    else:
        print("Base Startup Confirmation Unsuccessful")
        return False

#################################################
#
# Startup confirmation with robot
#
#################################################

def startup_robot(port):
    #send ready confirmation to the robot
    command(port, r_ad, jr_con)
    print("Jetson Confirmation sent to Robot")
   
    #wait for return transmission from robot
    while port.in_waiting == 0:
        time.sleep(1)

    #buffer full
    rx = receive(port)
    if rx.decode() == rr_con:
        print("Robot Startup Confirmation Successful")
        return True
    else:
        print("Robot Startup Confirmation Unsuccessful")
        return False

#############################################
#
# Parse data from a data RX
#
###########################################3#

def parse_data(rx, port):
    #check if the data is meant for the jetson
    rx = rx.decode()
    if rx[0] == '0':
        if "deez" in rx:
            print("from robot: " + rx)
            x = rx[1]
            y = rx[2]
            heading = rx[3]
            heading = rx[7]
        if "lol" in rx:
            print("from base: " + rx)
        
    #if command is for 1, 2, transmit it back out
    else:
        j_serial.Tx_Rx(port, rx, False, True)

#######################################
#
# package packets to send specific commands
#
#######################################

def command(port, ad, command):
    j_serial.Tx_Rx(port, ad + command + ready, False, True)

########################################
#
# recieve data from serial port buffer
#
#######################################

def receive(port):
    return j_serial.Tx_Rx(port, "", True, False)

#########################################
#
# Startup handshake function for base and robot
#
#########################################

def startup_hs():
    #initialize the serial port
    port = j_serial.init()
    print("Port Init Successful")

    #startup function
    success = startup_base(port)
    #if first try di dnot work, try again
    while success == False:
        success = startup_base(port)
    success = startup_robot(port)
    while success == False:
        success = startup_robot(port)
    return port

#test function to test handshake
def test_handshake():
    port = startup_hs()
    while(1):
        #poll command to robot
        command(port, r_ad, poll)
        while(port.in_waiting == 0):
            time.sleep(0.5)
        rx = receive(port)
        #parse_data(rx, port)
        print(rx.decode())
        time.sleep(1)

        #poll command to base
        #command(port, b_ad, poll)
        #while(port.in_waiting == 0):
            #time.sleep(0.5)
        #parse_data(receive(port), port)
        #time.sleep(1)

def final_test():
    #init handshake with robot and base
    d = 0.152
    port = startup_hs()
    tide = ROBOT(port)
    tide.heading = 0

    user_input = 0
    #create map by user stepping
    while user_input != "c":
        user_input = input("Poll? -y, -c ")
        if user_input == "y":
            #get data at the current location
            sensor_data = tide.poll_r()
            #check if at a corner. if so, rotate and move other the direction
            #if sensor_data[0] == 1:
               # tide.movement_rq(0, 0, -90)
               # tide.movement_rq(d, 0, -90)
               # tide.movement_rq(0,0, 180)
            #move the robot to the next 6 inch increment
                        #movment rq updates position
            rotate_sensors(sensor_data, tide.heading)

            tide.update_map(sensor_data)
            print(f'sensor_data{sensor_data}')
            print(tide.map)
            tide.movement_rq(0.152, 0, tide.heading)

    user_input = input("Compare map?! :? -y, -c ")
    #re run through the same distance but with obstacles changed
    tide.reset_position()
    tide.heading = 0
    tide.map_index[0] = 1
    while user_input != "c":
        user_input = input("Poll? -y, -c")
        if user_input == "y":
            sensor_data = tide.poll_r()
            #compare sensor data to whats in the node
            ix = tide.map_index[0]
            iy = tide.map_index[1]
            print([ix, iy])
            print(sensor_data)
            node_data = tide.map[ix][iy].sensors
            if sensor_data[0] != node_data[0]:
                if sensor_data[0] == 1:
                    print("new object detected in pos x")
                else:
                    print("object removed in pos x")
            if sensor_data[1] != node_data[1]:
                 if sensor_data[1] == 1:
                    print("new object detected in neg x")
                 else:
                    print("object removed in neg x")
            if sensor_data[2] != node_data[2]:
                if sensor_data[2] == 1:
                    print("new object detected in pos y")
                else:
                    print("object removed in pos y")
            if sensor_data[3] != node_data[3]:
                if sensor_data[3] == 1:
                    print("new object detected in neg y")
                else:
                    print("object removed in neg y")
 
            #move to next position in the map in the x direction
            #check if at corner
           # if sensor_data[0] == 1:
              #  tide.movement_rq(0, 0, -90)
              #  tide.movement_rq(d, 0, -90)
              #  tide.movement_rq(0,0, 180)
            tide.movement_rq(d, 0, tide.heading)


            

#####################################################################################################
#
# Algorithm to request tid-e to explore the room and send back the data in regular distance intervals
# Creates map from the data packets 
#
######################################################################################################

def map(robot):
    complete = 0
    #request movement out of base. facing same direction. after transmitted should save to the correct value in the robot
    robot.movement_rq(robot.position[0] + 0.305, robot.position[1], 0)
    #rotate to the left
    robot.movement_rq(robot.position[0], robot.position[1], 90)

    heading_r = 90
    #while the map is NOT complete
    while complete == 0:
        at_wall = 0
        #move until a wall is reached in this direction. then if heading y, rotate to 0 then -90. if heading -y rotate to 0 then 90
        while at_wall == 0:
            #check sensor data. no need to rotate for first check, just check first byte
            sensor_data = robot.poll_r()
            sensor_data_rot = rotate_sensors(sensor_data, robot.heading)
            if sensor_data[0] == 1:
                at_wall = 0
            else:
                if heading_r == 90:
                    robot.movement_rq(robot.position[0], robot.position[1] + 0.152, self.heading)
                elif heading_r == -90:
                    robot.movement_rq(robot.position[0], robot.position[1] - 0.152, self.heading)
            robot.update_map(sensor_data)
        #create map of the -x data to see if any spots where missed to the -x direction
        nx_data = [0]
        m_row = robot.map[robot.map_index[0]]
        for c in m_row:
            nx_data.append(c.nx)
        #at a wall. check if in a corner using the rotated sensor data (check for positive x direction. 
        #not in corner
        if sensor_data_rot[0] == 0:
            #rotate in x direction
            h = robot.heading
            #rotate to face x direction
            robot.movement_rq(robot.position[0], robot.position[1], 0)
            #move 6 inch in x direction
            robot.movement_rq(robot.position[0] + 0.152, robot.position[0], 0)
            #rotate to faceother direction
            robot.movement_rq(robot.position[0], robot.position[0], -1 * h)

            at_wall = 0
        #if in a corner, check x row to see if there is a gap in the sensors. if there is, try to go into it. if that doesn't work, move up to the first row in the cut out (HOW DO WE TELL WHEN ITS OVER OR WHWHERE THE CUTOUT IS IN SPACE?)
        
def full_com_test():
    port = startup_hs()
    tide = ROBOT(port)
    while(1):
        #test move rq
        print("test m rq")
        print(tide.movement_rq(1, 0, 90))
        #test poll
        print("test poll")
        sensor_data = tide.poll_r()
        print(sensor_data)

def rip_power_board_test():
    #init handshake with robot and base
    d = 0.152
    port = startup_hs()
    tide = ROBOT(port)
    tide.heading = 0

    #FINGERS CROSSED
    #tide.movement_rq(-d,0,0)


    user_input = 0
    # Move on command
    while user_input != "c":
        user_input = input("Move? -y, -c ")
        if user_input == "y":
            tide.movement_rq(1,0,0);

    #create map by user stepping
    user_input = 0
    while user_input != "c":
        user_input = input("Poll? -y, -c ")
        if user_input == "y":
            #get data at the current location
            sensor_data = tide.poll_r()
            #check if at a corner. if so, rotate and move other the direction
            #if sensor_data[0] == 1:
               # tide.movement_rq(0, 0, -90)
               # tide.movement_rq(d, 0, -90)
               # tide.movement_rq(0,0, 180)
            #move the robot to the next 6 inch increment
                        #movment rq updates position
            rotate_sensors(sensor_data, tide.heading)

            tide.update_map(sensor_data)
            print(f'sensor_data{sensor_data}')
            print(tide.map)
            #tide.movement_rq(0.152, 0, tide.heading)
            tide.update_position()

    user_input = input("Compare map?! :? -y, -c ")
    #re run through the same distance but with obstacles changed
    tide.reset_position()
    tide.heading = 0
    tide.map_index[0] = 1
    while user_input != "c":
        user_input = input("Poll?! -y, -c")
        if user_input == "y":
            sensor_data = tide.poll_r()
            #compare sensor data to whats in the node
            ix = tide.map_index[0]
            iy = tide.map_index[1]
            print([ix, iy])
            print(sensor_data)
            node_data = tide.map[ix][iy].sensors
            if sensor_data[0] != node_data[0]:
                if sensor_data[0] == 1:
                    print("new object detected in pos x")
                else:
                    print("object removed in pos x")
            if sensor_data[1] != node_data[1]:
                 if sensor_data[1] == 1:
                    print("new object detected in neg x")
                 else:
                    print("object removed in neg x")
            if sensor_data[2] != node_data[2]:
                if sensor_data[2] == 1:
                    print("new object detected in pos y")
                else:
                    print("object removed in pos y")
            if sensor_data[3] != node_data[3]:
                if sensor_data[3] == 1:
                    print("new object detected in neg y")
                else:
                    print("object removed in neg y")
 
            #move to next position in the map in the x direction
            #check if at corner
           # if sensor_data[0] == 1:
              #  tide.movement_rq(0, 0, -90)
              #  tide.movement_rq(d, 0, -90)
              #  tide.movement_rq(0,0, 180)
            #tide.movement_rq(d, 0, tide.heading)
            tide.update_position()

            print("testing movement request")
            #tide.movement_rq(1,0,0)
            #tide.movement_rq(0,0,90)



def main():
    #startup system
    rip_power_board_test()
    
if __name__ == "__main__":
    main()
